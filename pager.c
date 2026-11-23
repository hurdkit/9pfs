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
#include <hurd/pager.h>
#include <string.h>

/* Pager write-back.

   io-map.c opens the pager fid with write access whenever a mapping
   wants write, and these callbacks then let the kernel keep dirty
   pages of the file and clean them in batches.  The kernel batches
   up to 32 contiguous pages per data return, and pager_write_pages
   turns each batch into a few large T-writes instead of one rpc per
   page.

   Ordering.  Every chunk goes out on the single shared connection,
   so the server applies them in send order.  A page fault that
   follows an in flight write back re-reads through a T-read issued
   after the T-write, and sees the written data.  The same argument
   covers S_io_write and S_file_set_size ordering below.

   Durability window.  A page the kernel has written back may still
   sit in the fire-and-forget queue when pager_write_pages returns.
   The fsync path drains that queue after syncing the pager, so a
   sync still covers pager flushes.  Between write back and drain the
   data is no more durable than the write-behind window S_io_write
   already opened for direct writes, and a server or connection
   failure in that window loses the page silently, with the error
   only logged by the background reader.  Crash consistency is
   unchanged from that existing window.  */

error_t
pager_read_page (struct user_pager_info *upi,
                 vm_offset_t page,
                 vm_address_t *buf,
                 int *write_lock)
{
  error_t err, err2;
  struct node *np = (struct node *) upi;
  uint32_t size;

  if (np->pager_fid == P9_NO_FID)
    return EGRATUITOUS;

  size = vm_page_size;
  *buf = 0;
  err = vm_allocate (mach_task_self (), buf, size, TRUE);
  if (err)
    return err;

  /* TODO: cap the message size, retry short reads in a loop */
  err = p9_rpc (P9_READ_REQUEST,
                "484", np->pager_fid, (uint64_t) page, size,
                "d", &size, *buf);
  if (err)
    {
      err2 = vm_deallocate (mach_task_self (), *buf, vm_page_size);
      assert_perror_backtrace (err2);
      return err;
    }
  memset ((char *) *buf + size, 0, vm_page_size - size);
  /* A set write_lock hands the page to the kernel read-only.  The
     kernel then has to ask pager_unlock_page before it may dirty the
     page.  When the pager fid is open for writing there is nothing
     to allocate or reserve, unlike ext2fs which needs block
     allocation first, so supply writable pages right away.  That is
     the acceleration this pager exists for.  */
  *write_lock = !np->pager_writable;
  return 0;
}

/* Shared write path for pager_write_page and pager_write_pages.

   Splits the byte range into T-write messages no larger than the
   node message limit and pushes each out with p9_rpc_noreply, so the
   writes join the fire-and-forget queue the S_io_write write-behind
   path uses.  The count, cap and drain machinery in 9p-rpc.c apply
   unchanged.  When the background machinery is down the chunks go
   out as ordinary synchronous rpcs instead.

   The caller guarantees whole pages: libpager checks offset and
   length for page alignment in data-return.c before calling us, and
   its bulk caller coalesces only page aligned runs.  There is no
   partial page tail to handle, so no read modify write here.

   Clipping.  The kernel can hand back the whole dirty page that
   holds the end of the file, tail bytes past the end included, and a
   full page write would extend the file to the page boundary with
   garbage.  So every chunk is clipped at np->pager_size.  The 9P
   protocol has no partial write barrier for page granularity, the
   clip supplies one by simply writing fewer bytes.  Pages entirely
   past the limit are dropped with a success return, they hold no
   valid file data.  */
static error_t
pager_write_range (struct user_pager_info *upi,
                   vm_offset_t offset,
                   vm_address_t data,
                   vm_size_t length,
                   vm_size_t *written)
{
  struct node *np = (struct node *) upi;
  uint32_t fid = np->pager_fid;
  uint64_t eof = np->pager_size;
  vm_size_t cap;
  vm_size_t done = 0;
  error_t err = 0;
  int background;

  if (fid == P9_NO_FID)
    /* No pager fid means no way to reach the file.  */
    return EIO;

  /* 24 is the message overhead of one payload byte in a write, the
     same budget S_io_write uses.  A server that shrank the node limit
     below that cannot carry any payload at all.  */
  if (np->max_message_size < 24)
    return EIO;
  cap = np->max_message_size - 24;

  background = p9_bg_writes_usable ();

  while (done < length)
    {
      uint64_t off = (uint64_t) offset + done;
      vm_size_t chunk;

      if (off >= eof)
        /* The rest of the range lies past the end of the file.  */
        break;

      chunk = length - done;
      if (chunk > eof - off)
        /* Never write past the end of the file.  */
        chunk = eof - off;

      /* Keep chunk boundaries on page edges while the message limit
         allows it, so a later chunk never straddles two different
         send outcomes.  Servers with a sub page message limit get
         sub page chunks, and the page granular progress report below
         absorbs the difference.  */
      if (cap >= vm_page_size)
        {
          vm_size_t aligned_cap = cap & ~(vm_page_size - 1);
          if (chunk > aligned_cap)
            chunk = aligned_cap;
        }
      else if (chunk > cap)
        chunk = cap;

      if (background)
        {
          /* The blob is borrowed for the duration of the send.  The
             writev inside completes before this call returns, and
             libpager keeps the pages mapped until we return, so the
             borrow stays valid.  */
          err = p9_rpc_noreply (P9_WRITE_REQUEST,
                                "48D", fid, off, chunk, data + done);
        }
      else
        {
          uint32_t nwritten;

          err = p9_rpc (P9_WRITE_REQUEST,
                        "48D", fid, off, chunk, data + done,
                        "4", &nwritten);
          if (!err && nwritten < chunk)
            err = EIO;
        }

      if (err)
        break;

      done += chunk;
    }

  /* Libpager counts progress in whole pages and falls back to
     pager_write_page for the rest, so report a page multiple.  A
     partial page at the end of the file clips to a non page multiple
     byte count, which this rounding drops.  The per page fallback
     then repeats that page clipped the same way, an idempotent
     rewrite of the same bytes at the same offset.  */
  done -= done % vm_page_size;
  *written = done;

  if (err)
    return (err == ENOSPC || err == EDQUOT) ? err : EIO;
  return 0;
}

error_t
pager_write_page (struct user_pager_info *upi,
                  vm_offset_t page,
                  vm_address_t buf)
{
  vm_size_t written;

  /* The bulk path above handles a single page as a special case, so
     one implementation serves both entry points.  Synchronous in the
     libpager sense: the buffer is fully consumed by the send before
     this returns, even when the server reply is still on its way.  */
  return pager_write_range (upi, page, buf, vm_page_size, &written);
}

error_t
pager_write_pages (struct user_pager_info *upi,
                   vm_offset_t offset,
                   vm_address_t data,
                   vm_size_t length,
                   vm_size_t *written)
{
  return pager_write_range (upi, offset, data, length, written);
}

error_t
pager_unlock_page (struct user_pager_info *upi,
                   vm_offset_t page)
{
  struct node *np = (struct node *) upi;

  /* The kernel only asks this for pages supplied read-only, which
     happens when the server refused write access for the pager fid or
     when a mapping faulted the page in before an upgrade to write
     access.  Grant the unlock once the fid can carry the write back.
     Refuse otherwise, data-unlock.c then flushes the page and marks
     the next fault on it with our error.  A read-only mount never
     opens or upgrades the fid for writing, so its write faults fail
     here.  */
  return np->pager_writable ? 0 : EACCES;
}


void
pager_notify_evict (struct user_pager_info *upi,
                    vm_offset_t page)
{
  assert_backtrace (!"unrequested notifications");
}

error_t
pager_report_extent (struct user_pager_info *upi,
                     vm_address_t *offset,
                     vm_size_t *size)
{
  struct node *np = (struct node *) upi;

  /* Dirty pages cannot exist beyond the high water mark, so whole
     object pager operations only need to cover up to it.  Reporting
     SIZE_MAX is not an option: the kernel rounds a vm_size_t extent
     up to the page grid, and SIZE_MAX wraps that round to zero, which
     turns every whole object lock request into a silent no-op.  A
     caller that asks before any size is known gets an empty object,
     which is correct, nothing can be dirty yet.  */
  {
    uint64_t extent = ((uint64_t) np->pager_size + vm_page_size - 1)
                      & ~(uint64_t) (vm_page_size - 1);
    /* Clamp to the largest page aligned value that fits a vm_size_t,
       rather than letting the cast wrap a size near 4 GiB to zero.  */
    uint64_t max_extent = (uint64_t) ~(vm_size_t) 0 - (vm_page_size - 1);
    if (extent > max_extent)
      extent = max_extent;
    *offset = 0;
    *size = extent;
  }
  return 0;
}

void
pager_clear_user_data (struct user_pager_info *upi)
{
}

void
pager_dropweak (struct user_pager_info *upi)
{
  struct node *np = (struct node *) upi;
  struct pager *pager;

  pthread_mutex_lock (&np->lock);
  pager = np->pager;
  assert_backtrace (pager_get_upi (pager) == upi);

  np->pager = NULL;
  ports_port_deref_weak (pager);
  p9_nput (np);
}
