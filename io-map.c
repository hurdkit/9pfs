/* 9P translator

   Copyright (C) 2021-2026 Sergey Bugaev <bugaevc@gmail.com>

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <https://www.gnu.org/licenses/>. */

/* Proudly written in GNU nano. */

#define _GNU_SOURCE 1

#include "9pfs.h"
#include "9p-rpc.h"
#include "io_S.h"
#include <hurd/pager.h>
#include <errno.h>

/* Open FID on the server, read-write when WRITABLE and read-only
   otherwise.  The core protocol and 9P2000.L number the access mode
   differently from the Hurd, so both values come from the shared
   converters.  */
static error_t
open_pager_fid (uint32_t fid, int writable)
{
  int flags = writable ? O_RDWR : O_READ;
  uint32_t l_flags = p9_open_flags_to_lflags (flags);
  unsigned char core_mode = p9_open_flags_to_core_mode (flags);
  struct p9_qid qid;
  uint32_t max_message_size;

  if (p9_version >= P9_VERSION_2000_L)
    return p9_rpc (P9_LOPEN_REQUEST,
                   "44", fid, l_flags,
                   "1484", &qid.type, &qid.version, &qid.path,
                   &max_message_size);
  else
    return p9_rpc (P9_OPEN_REQUEST,
                   "41", fid, core_mode,
                   "1484", &qid.type, &qid.version, &qid.path,
                   &max_message_size);
}

/* Create the pager of NP on its first mapping.  PROTID is the client
   asking for the map and FLAGS its open mode.  Only a mapping that
   carries write access can lead to dirty pages, and the server checks
   the fid user against the file at open time.  So a read-only mapping
   gets a read-only fid, which is the common case and skips the write
   permission round trip.  When the server refuses write access to this
   user, the fid is opened read-only instead and the caller refuses the
   write half of the map.  The node lock must be held.  */
static error_t
create_node_pager (struct protid *protid, struct node *np, int flags)
{
  error_t err;
  int writable = !p9_readonly && (flags & O_WRITE);
  uint16_t n_qids;
  struct pager *pager;

  assert_backtrace (np->pager_fid == P9_NO_FID);
  if (protid->walk_fid == P9_NO_FID)
    return EBADF;

  np->pager_fid = p9_fid_alloc ();
  err = p9_rpc (P9_WALK_REQUEST,
                "442", protid->walk_fid, np->pager_fid, 0,
                "2", &n_qids);
  if (err)
    return err;

  /* FIXME: Do this more robustly, perhaps let the pager hold its own protid */
  err = open_pager_fid (np->pager_fid, writable);
  if (err && writable && (err == EACCES || err == EPERM))
    {
      /* The server refuses write access to this user.  Retry
         read-only rather than fail the mapping, and let the caller
         refuse the write half of the map.  */
      writable = 0;
      err = open_pager_fid (np->pager_fid, 0);
    }
  if (err)
    {
      p9_rpc (P9_CLUNK_REQUEST,
              "4", np->pager_fid, "");
      np->pager_fid = P9_NO_FID;
      return err;
    }

  /* XXX: MEMORY_OBJECT_COPY_DELAY is not correct here, since the remote
     server could also be modifying the files and we wouldn't know.  */
  pager = pager_create ((struct user_pager_info *) np, p9_pager_bucket,
                        FALSE, MEMORY_OBJECT_COPY_DELAY, FALSE);
  if (!pager)
    {
      err = errno;
      p9_rpc (P9_CLUNK_REQUEST,
              "4", np->pager_fid, "");
      np->pager_fid = P9_NO_FID;
      return err;
    }

  /* Pager holds a strong reference to its node.  The node holds
     a weak reference back to the pager.  */
  ports_port_ref_weak (pager);
  p9_nref (np);
  np->pager = pager;
  np->pager_writable = writable;
  return 0;
}

/* Reopen the pager fid of NP for writing.  The pager exists with a
   read-only fid, opened by an earlier read-only mapping, and this
   mapping wants write access.  Walk a fresh fid from PROTID, so the
   server checks the writing user, open it read-write, then swap it in.

   The old fid is deliberately left open rather than clunked.  A pager
   callback thread may hold the old fid number across an in flight
   T-read, and a clunk that lands first would fail that read and error
   the page.  The leaked fid stays valid for the life of the
   connection, and this upgrade happens at most once per pager.  The
   node lock must be held.  */
static error_t
reopen_pager_writable (struct protid *protid, struct node *np)
{
  error_t err;
  uint32_t new_fid = p9_fid_alloc ();
  uint16_t n_qids;

  err = p9_rpc (P9_WALK_REQUEST,
                "442", protid->walk_fid, new_fid, 0,
                "2", &n_qids);
  if (!err)
    err = open_pager_fid (new_fid, 1);
  if (err)
    {
      p9_rpc (P9_CLUNK_REQUEST,
              "4", new_fid, "");
      return err;
    }

  /* Plain aligned word stores.  Callbacks running without the
     node lock see either fid, and both remain valid on the
     server.  No write back can race the swap, since pages were
     supplied read-only until now.  */
  np->pager_fid = new_fid;
  np->pager_writable = 1;
  return 0;
}

error_t
S_io_map (struct protid *protid,
          mach_port_t *rdobj,
          mach_msg_type_name_t *rdtype,
          mach_port_t *wrobj,
          mach_msg_type_name_t *wrtype)
{
  error_t err = 0;
  int flags;
  struct node *np;
  struct pager *pager;
  boolean_t deref_pager = FALSE;

  if (!protid)
    return EOPNOTSUPP;

  flags = protid->po->user_open_flags & O_RDWR;
  if (!flags)
    {
      /* Return null ports, successfully.  The client
         translates this to EACCES from mmap.  */
      *rdobj = *wrobj = MACH_PORT_NULL;
      *rdtype = *wrtype = MACH_MSG_TYPE_COPY_SEND;
      return 0;
    }

  np = protid->po->np;
  pthread_mutex_lock (&np->lock);

  /* The first mapping creates the pager.  A mapping that wants write
     access through a pager opened read-only gets the fid reopened.  */
  pager = np->pager;
  if (!pager)
    {
      err = create_node_pager (protid, np, flags);
      if (err)
        goto out;
      pager = np->pager;
      deref_pager = TRUE;
    }
  else if ((flags & O_WRITE) && !np->pager_writable && !p9_readonly)
    {
      err = reopen_pager_writable (protid, np);
      if (err)
        goto out;
    }

  /* Hand out the write half of the map only when the pager fid can
     carry the writes back.  A null write object makes the client mmap
     fail with EACCES, which is the honest answer when the server
     refused write access to the pager fid.  */
  switch (flags)
    {
    case O_RDWR:
      /* Call pager_get_port twice to account for two make-sends.  */
      *wrobj = np->pager_writable ? pager_get_port (pager) : MACH_PORT_NULL;
      *rdobj = pager_get_port (pager);
      *rdtype = *wrtype = MACH_MSG_TYPE_MAKE_SEND;
      break;
    case O_READ:
      *wrobj = MACH_PORT_NULL;
      *rdobj = pager_create_ro_port (pager);
      *rdtype = *wrtype = MACH_MSG_TYPE_MOVE_SEND;
      break;
    case O_WRITE:
      /* We'll return null in *rdport, but that's mostly informative,
         it doesn't really prevent the client from reading.  */
      *wrobj = np->pager_writable ? pager_get_port (pager) : MACH_PORT_NULL;
      *rdobj = MACH_PORT_NULL;
      *rdtype = *wrtype = MACH_MSG_TYPE_MAKE_SEND;
      break;
    default:
      assert_backtrace (0);
    }

 out:
  pthread_mutex_unlock (&np->lock);

  if (deref_pager)
    ports_port_deref (pager);

  return err;
}
