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
#include "fs_S.h"
#include <errno.h>
#include <hurd/pager.h>

error_t
S_file_sync (struct protid *pi, int wait, int omit_metadata)
{
  error_t err;
  struct p9_stat s;
  struct node *np;
  struct pager *pager;

  if (!pi)
    return EOPNOTSUPP;
  if (p9_readonly)
    /* Should we return EROFS?  */
    return 0;

  np = pi->po->np;

  /* Sync the pager first, before the drain below.  pager_sync makes
     the kernel write its dirty pages of this node back through
     pager_write_pages, which queues them as background T-writes.
     With wait set it returns once every write back has been sent.
     The extent it covers comes from pager_report_extent, which uses
     the size high water mark, so every dirty page of the node is
     included.  */
  pthread_mutex_lock (&np->lock);
  pager = np->pager;
  if (pager)
    ports_port_ref (pager);
  pthread_mutex_unlock (&np->lock);

  if (pager)
    {
      pager_sync (pager, TRUE);
      ports_port_deref (pager);
    }

  /* Drain the write-behind queue next.  The outstanding count is
     global, so this waits for every fire-and-forget write in the
     filesystem, not just this file, including the pager flushes sent
     just above.  That is the scope choice made in 9p-rpc.c.  Sending
     the sync request afterwards guarantees the server sees the
     writes before the sync.  */
  p9_wait_bg_writes ();


  if (p9_version >= P9_VERSION_2000_L)
    {
      /* Ensure we're open at least for something.  */
      if (!pi->server_open_flags)
        {
          err = p9_ensure_open (pi, O_READ);
          if (err)
            return err;
        }
      return p9_rpc (P9_FSYNC_REQUEST,
                     "44", pi->io_fid, omit_metadata,
                     "");
    }

  /* Use the special case of wstat.  */
  p9_stat_dont_touch (&s);
  err = p9_wstat (pi->walk_fid, &s);
  p9_stat_free (&s);
  return err;
}
