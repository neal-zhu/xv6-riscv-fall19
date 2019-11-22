// Buffer cache.
//
// The buffer cache is a linked list of buf structures holding
// cached copies of disk block contents.  Caching disk blocks
// in memory reduces the number of disk reads and also provides
// a synchronization point for disk blocks used by multiple processes.
//
// Interface:
// * To get a buffer for a particular disk block, call bread.
// * After changing buffer data, call bwrite to write it to disk.
// * When done with the buffer, call brelse.
// * Do not use the buffer after calling brelse.
// * Only one process at a time can use a buffer,
//     so do not keep them longer than necessary.


#include "types.h"
#include "param.h"
#include "spinlock.h"
#include "sleeplock.h"
#include "riscv.h"
#include "defs.h"
#include "fs.h"
#include "buf.h"

#define NBUCKETS 13

struct {
  struct spinlock lock;
  struct buf      buf[NBUF];
  struct spinlock locks[NBUCKETS];
  struct buf      lrus[NBUCKETS];
  struct buf     *elems[NBUCKETS];
} bcache;

struct buf **find_ptr(uint dev, uint blockno) {
  uint i = blockno % NBUCKETS;
  struct buf **p = &bcache.elems[i];
  while (*p != 0 &&
          ((*p)->blockno != blockno || (*p)->dev != dev))
    p = &(*p)->next_hash;
  return p;
}

void btable_insert(struct buf *buf) {
  struct buf **p = find_ptr(buf->dev, buf->blockno);
  *p = buf;
  buf->next_hash = 0;
}

void btable_remove(struct buf *buf) {
  struct buf **p = find_ptr(buf->dev, buf->blockno);
  if (*p != 0) {
    *p = (*p)->next_hash;
  }
}

struct buf *btable_find(uint dev, uint blockno) {
  struct buf **p = find_ptr(dev, blockno);
  return *p;
}

void lru_remove(struct buf *b) {
  b->next->prev = b->prev;
  b->prev->next = b->next;
}

void lru_add(struct buf *b, struct buf *head) {
  b->next = head;
  b->prev = head->prev;
  b->prev->next = b;
  b->next->prev = b;
}

void
binit(void)
{
  initlock(&bcache.lock, "bcache");
  for (int i = 0; i < NBUCKETS; i++) {
    initlock(&bcache.locks[i], "bcache.bucket");
    bcache.lrus[i].next = &bcache.lrus[i];
    bcache.lrus[i].prev = &bcache.lrus[i];
  }

  struct buf *b;
  struct buf *head;
  for (int i = 0; i < NBUF; i++) {
    head = &bcache.lrus[i%NBUCKETS];
    b = bcache.buf + i;
    lru_add(b, head);
  }
}

// Look through buffer cache for block on device dev.
// If not found, allocate a buffer.
// In either case, return locked buffer.
static struct buf*
bget(uint dev, uint blockno)
{
  struct buf *b;

  int idx = blockno % NBUCKETS;
  struct spinlock *lk = &bcache.locks[idx];
  acquire(lk);

  // Is the block already cached?
  b = btable_find(dev, blockno);
  if (b != 0) {
    b->refcnt++;
    release(lk);
    acquiresleep(&b->lock);
    return b;
  }

  // find available buf in its own lru
  if (bcache.lrus[idx].next != &bcache.lrus[idx]) {
    b = bcache.lrus[idx].next;
    b->refcnt = 1;
    b->blockno = blockno;
    b->dev = dev;
    b->valid = 0;
    btable_insert(b);
    lru_remove(b);
    release(lk);
    acquiresleep(&b->lock);
    return b;
  }

  for (int i = 0; i < NBUCKETS; i++) {
    if (i == idx)
      continue;
    if (i < idx) {
      acquire(&bcache.locks[i]);
    } else {
      release(lk);
      acquire(&bcache.locks[i]);
      acquire(lk);
    }

    // double check because we release lk and re-acquire
    b = btable_find(dev, blockno);
    if (b != 0) {
      b->refcnt++;
      release(lk);
      release(&bcache.locks[i]);
      acquiresleep(&b->lock);
      return b;
    }

    if (bcache.lrus[i].next == &bcache.lrus[i]) {
      release(&bcache.locks[i]);
      continue;
    }

    b = bcache.lrus[i].next;
    lru_remove(bcache.lrus[i].next);
    btable_insert(b);

    b->refcnt = 1;
    b->blockno = blockno;
    b->dev = dev;
    b->valid = 0;
    release(lk);
    release(&bcache.locks[i]);
    acquiresleep(&b->lock);

    return b;
  }

  panic("bget: no buffers");
}

// Return a locked buf with the contents of the indicated block.
struct buf*
bread(uint dev, uint blockno)
{
  struct buf *b;

  b = bget(dev, blockno);
  if(!b->valid) {
    virtio_disk_rw(b->dev, b, 0);
    b->valid = 1;
  }
  return b;
}

// Write b's contents to disk.  Must be locked.
void
bwrite(struct buf *b)
{
  if(!holdingsleep(&b->lock))
    panic("bwrite");
  virtio_disk_rw(b->dev, b, 1);
}

// Release a locked buffer.
// Move to the head of the MRU list.
void
brelse(struct buf *b)
{
  if(!holdingsleep(&b->lock))
    panic("brelse");

  releasesleep(&b->lock);

  int idx = b->blockno % NBUCKETS;
  struct spinlock *lk = &bcache.locks[idx];
  acquire(lk);
  
  b->refcnt--;
  if (b->refcnt == 0) {
    lru_add(b, &bcache.lrus[idx]);
    btable_remove(b);
  }
  release(lk);
}

void
bpin(struct buf *b) {
  acquire(&bcache.lock);
  b->refcnt++;
  release(&bcache.lock);
}

void
bunpin(struct buf *b) {
  acquire(&bcache.lock);
  b->refcnt--;
  release(&bcache.lock);
}
