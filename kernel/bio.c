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
  struct buf buf[NBUF];

  struct buf      list[NBUCKETS];
  struct spinlock locks[NBUCKETS];
} bcache;

void
binit(void)
{
  struct buf *b;

  initlock(&bcache.lock, "bcache");
  for (int i = 0; i < NBUCKETS; i++) {
    initlock(&bcache.locks[i], "bcache.bucket");
    bcache.list[i].next = &bcache.list[i];
    bcache.list[i].prev = &bcache.list[i];
  }

  for (int i = 0; i < NBUF; i++) {
    int bkt = i % NBUCKETS;
    b = bcache.buf + i;
    b->next = bcache.list[bkt].next;
    b->prev = &bcache.list[bkt];
    initsleeplock(&b->lock, "buffer");
    bcache.list[bkt].next->prev = b;
    bcache.list[bkt].next = b;
  }
}

// Look through buffer cache for block on device dev.
// If not found, allocate a buffer.
// In either case, return locked buffer.
static struct buf*
bget(uint dev, uint blockno)
{
  struct buf *b;
  int bkt = blockno % NBUCKETS;

  acquire(&bcache.locks[bkt]);
  // Is the block already cached?
  for (b = bcache.list[bkt].next; b != &bcache.list[bkt]; b = b->next) {
    if (b->dev == dev && b->blockno == blockno) {
      assert(b->refcnt >= 0);
      b->refcnt++;
      release(&bcache.locks[bkt]);
      acquiresleep(&b->lock);
      return b;
    }
  }

  // Not cached; recycle an unused buffer.
  for(b = bcache.list[bkt].prev; b != &bcache.list[bkt]; b = b->prev){
    if(b->refcnt == 0) {
      b->dev = dev;
      b->blockno = blockno;
      b->valid = 0;
      b->refcnt = 1;
      release(&bcache.locks[bkt]);
      acquiresleep(&b->lock);
      return b;
    }
  }
  release(&bcache.locks[bkt]);

  acquire(&bcache.lock);
  // Steal a available buf from other bucket
  for (int i = 0; i < NBUCKETS; i++) {
    if (i == bkt)
      continue;
    acquire(&bcache.locks[i]);
    for(b = bcache.list[i].prev; b != &bcache.list[i]; b = b->prev) {
      if(b->refcnt == 0) {
        // move buf from i to bkt
        b->next->prev = b->prev;
        b->prev->next = b->next;
        b->next = bcache.list[bkt].next;
        b->prev = &bcache.list[bkt];
        bcache.list[bkt].next->prev = b;
        bcache.list[bkt].next = b;

        b->dev = dev;
        b->blockno = blockno;
        b->valid = 0;
        b->refcnt = 1;
        release(&bcache.lock);
        release(&bcache.locks[i]);
        acquiresleep(&b->lock);
        return b;
      }
    }
    release(&bcache.locks[i]);
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

  int bkt = b->blockno % NBUCKETS;
  acquire(&bcache.locks[bkt]);
  b->refcnt--;
  if (b->refcnt == 0) {
    // no one is waiting for it.
    b->next->prev = b->prev;
    b->prev->next = b->next;
    b->next = bcache.list[bkt].next;
    b->prev = &bcache.list[bkt];
    bcache.list[bkt].next->prev = b;
    bcache.list[bkt].next = b;
  }
  release(&bcache.locks[bkt]);
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


