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
#include <fcntl.h>
#include <errno.h>
#include <mach.h>
#include <hurd/pager.h>

error_t
S_io_write (struct protid *protid,
            const char *data,
            mach_msg_type_number_t datalen,
            loff_t offset,
            vm_size_t *amount)
{
  error_t err;
  struct peropen *po;
  struct node *np;
  struct pager *pager;
  uint64_t start;
  uint32_t nwritten;

  if (protid == NULL)
    return EOPNOTSUPP;

  po = protid->po;
  np = po->np;

  if (!(po->user_open_flags & O_WRITE))
    return EBADF;
  else if (p9_readonly)
    return EROFS;

  pthread_mutex_lock (&np->lock);

  err = p9_ensure_open (protid, O_WRITE);
  if (err)
    goto out;

  /* Cap amount to the size we can fit in one message.  */
  if (datalen > np->max_message_size - 24)
    datalen = np->max_message_size - 24;

  /* FIXME: Respect O_APPEND.  In 9P2000.L, perhaps we can use
     the native O_APPEND flags when opening?  */
  start = (offset == -1) ? po->offset : offset;

  /* Preemptively invalidate the stat cache.  */
  np->last_stat = 0;

  /* Coherence with the kernel page cache.

     Direct writes go to the server through the io fid, while a
     writable pager may hold dirty pages for the same range in the
     kernel.  Policy choice: flush and drop the affected kernel pages
     before the write.  pager_return_some writes any dirty pages back
     and then frees the kernel copies, so a fault after this write
     refetches the fresh server data and ordering is preserved.  A
     plain invalidate without write back would lose the dirty pager
     pages, which is why return and not flush is used here.

     This runs under the node lock.  That is safe because the pager
     callbacks never take it, see pager.c.  */
  pager = np->pager;
  if (pager && np->pager_writable)
    {
      /* Round the range out to whole pages.  The pager address space
         is 32 bit, so clamp a range that reaches past 4 GiB.  */
      uint64_t flush_to = ((uint64_t) start + datalen + vm_page_size - 1)
                          & ~((uint64_t) vm_page_size - 1);
      uint64_t max_to = (uint64_t) ~(vm_offset_t) 0 - (vm_page_size - 1);
      uint64_t flush_from = (uint64_t) start
                            & ~((uint64_t) vm_page_size - 1);

      if (flush_to > max_to)
        flush_to = max_to;

      ports_port_ref (pager);
      if (flush_to > flush_from)
        pager_return_some (pager, flush_from, flush_to - flush_from, TRUE);
      ports_port_deref (pager);
    }

  /* Write-behind for unsynced opens.  O_SYNC and O_DSYNC alias the
     same bit on the Hurd, so one test covers both.  The flags arrive
     with the open call and live in user_open_flags.  The
     io_set_all_openmodes family keeps them current, since O_FSYNC is
     one of the honored state modes.

     With neither flag set, send the T-write and reply at once.  The
     server's reply is collected in the background.  Error semantics
     change only for this unsynced case, and that is the point of the
     feature: a write that later fails at the server still reports
     success here.  The failure is logged, and a following sync will
     report errors on the connection.  */
  if (!(po->user_open_flags & (O_SYNC | O_DSYNC))
      && p9_bg_writes_usable ())
    {
      /* The payload is borrowed from the MIG request buffer.  The
         borrow ends when the send call returns, which is still
         inside this handler.  */
      err = p9_rpc_noreply (P9_WRITE_REQUEST,
                            "48D", protid->io_fid, start, datalen, data);
      if (err)
        goto out;

      /* The real count comes back later.  Reply with what was
         requested, which is what a successful write returns.  */
      *amount = datalen;
      if (offset == -1)
        po->offset += datalen;

      /* The write may extend the file, so raise the write-back clip.  */
      if (start + datalen > (uint64_t) np->pager_size)
        np->pager_size = start + datalen;
      goto out;
    }

  /* Send the data straight out of the MIG request buffer as a
     borrowed blob, saving a copy of the payload into the send buffer.
     The request buffer lives until this handler returns, and the
     writev completes before we return, so the borrow stays valid.  */
  err = p9_rpc (P9_WRITE_REQUEST,
                "48D", protid->io_fid, start, datalen, data,
                "4", &nwritten);
  if (err)
    goto out;
  if (nwritten > datalen)
    return EIO;

  *amount = nwritten;
  if (offset == -1)
    po->offset += nwritten;

  /* The write may extend the file, so raise the write-back clip.  */
  if (start + nwritten > (uint64_t) np->pager_size)
    np->pager_size = start + nwritten;

 out:
  pthread_mutex_unlock (&np->lock);

  return err;
}
