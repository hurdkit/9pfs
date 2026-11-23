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
#include <mach.h>

/* Push the kernel page cache of NP in line with a new file size.
   Called with the node lock held.  Growth only raises the write-back
   clip.  A shrink also has to clear the pages of the range being
   cut, or a write back of a stale dirty page would resurrect the
   truncated data.  */
static void
pager_resize (struct node *np, loff_t new_size)
{
  struct pager *pager = np->pager;
  loff_t old_size = np->pager_size;

  if (!pager)
    {
      np->pager_size = new_size;
      return;
    }

  if (new_size >= old_size)
    {
      np->pager_size = new_size;
      return;
    }

  ports_port_ref (pager);

  /* Drop the clip before any page goes back, so the partial page that
     holds the new end writes only its valid prefix.  The tail bytes
     of that page would otherwise re-extend the file with garbage.
     Any write back already queued when the clip drops still writes to
     the old limit, and those bytes then precede the size request on
     the connection, so the server applies them first and the
     truncate last.  The final size is correct either way.  */
  np->pager_size = new_size;

  /* Write the partial page back and free it, so its valid prefix
     survives and its tail does not linger as stale kernel data.  */
  pager_return_some (pager, trunc_page ((vm_offset_t) new_size),
                     vm_page_size, TRUE);

  /* Free the whole pages past the new end without writing them, they
     hold no valid data any more.  A following fault reads the shorter
     file and zero fills the tail.  */
  {
    vm_offset_t from = round_page ((vm_offset_t) new_size);
    uint64_t to = ((uint64_t) old_size + vm_page_size - 1)
                  & ~((uint64_t) vm_page_size - 1);
    uint64_t max_to = (uint64_t) ~(vm_offset_t) 0 - (vm_page_size - 1);

    if (to > max_to)
      to = max_to;
    if (to > from)
      pager_flush_some (pager, from, to - from, TRUE);
  }

  ports_port_deref (pager);
}

error_t
S_file_set_size (struct protid *pi, loff_t new_size)
{
  error_t err;
  struct p9_stat s;

  if (!pi)
    return EOPNOTSUPP;
  if (new_size < 0)
    return EINVAL;
  if (!(pi->po->user_open_flags & O_WRITE))
    return EBADF;
  /* Should not really happen due to the above check.  */
  if (p9_readonly)
    return EROFS;

  pi->po->np->last_stat = 0;

  pthread_mutex_lock (&pi->po->np->lock);
  pager_resize (pi->po->np, new_size);
  pthread_mutex_unlock (&pi->po->np->lock);

  /* Failure of the size request below leaves the server file at the
     old size while the clip and the kernel pages already match the
     new one.  Later faults re-read the real server contents, so the
     error is reported and the two views converge on the next stat.  */
  if (p9_version >= P9_VERSION_2000_L)
    return p9_rpc (P9_SETATTR_REQUEST,
                   "4444488888", pi->walk_fid, P9_SETATTR_MASK_SIZE,
                   0, 0, 0, new_size, 0, 0, 0, 0,
                   "");

  p9_stat_dont_touch (&s);
  s.length = new_size;
  err = p9_wstat (pi->walk_fid, &s);
  p9_stat_free (&s);

  return err;
}
