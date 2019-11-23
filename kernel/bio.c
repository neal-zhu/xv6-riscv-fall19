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

  // Linked list of all buffers, through prev/next.
  // head.next is most recently used.
  struct buf head;

  // hashtable to lookup cached buf
  struct buf      *list[NBUCKETS];
  struct spinlock locks[NBUCKETS];
  int nelems;
} bcache;

struct buf **find_ptr(uint dev, uint blockno) {
  uint i = blockno % NBUCKETS;
  struct buf **p = &bcache.list[i];
  while (*p != 0 &&
          ((*p)->blockno != blockno || (*p)->dev != dev))
    p = &(*p)->next_hash;
  return p;
}

void btable_insert(struct buf *buf) {
  struct buf **p = find_ptr(buf->dev, buf->blockno);
  *p = buf;
  buf->next_hash = 0;
  bcache.nelems++;
}

void btable_remove(struct buf *buf) {
  struct buf **p = find_ptr(buf->dev, buf->blockno);
  if (*p != 0) {
    *p = (*p)->next_hash;
    bcache.nelems--;
  }
}

struct buf *btable_lookup(uint dev, uint blockno) {
  struct buf **p = find_ptr(dev, blockno);
  return *p;
}

void
binit(void)
{
  struct buf *b;

  initlock(&bcache.lock, "bcache");
  memset(bcache.list, 0, sizeof(bcache.list));
  bcache.nelems = 0;
  for (int i = 0; i < NBUCKETS; i++)
    initlock(&bcache.locks[i], "bcache.bucket");

  // Create linked list of buffers
  bcache.head.prev = &bcache.head;
  bcache.head.next = &bcache.head;
  for(b = bcache.buf; b < bcache.buf+NBUF; b++){
    b->next = bcache.head.next;
    b->prev = &bcache.head;
    initsleeplock(&b->lock, "buffer");
    bcache.head.next->prev = b;
    bcache.head.next = b;
  }
}

// Look through buffer cache for block on device dev.
// If not found, allocate a buffer.
// In either case, return locked buffer.
static struct buf*
bget(uint dev, uint blockno)
{
  struct buf *b;

  acquire(&bcache.locks[blockno%NBUCKETS]);
  // Is the block already cached?
  if ((b = btable_lookup(dev, blockno)) != 0 && b->refcnt != 0) {
    b->refcnt++;
    release(&bcache.locks[blockno%NBUCKETS]);
    acquiresleep(&b->lock);
    return b;
  }

  acquire(&bcache.lock);
  // Not cached; recycle an unused buffer.
  for(b = bcache.head.prev; b != &bcache.head; b = b->prev){
    if(b->refcnt == 0) {
      b->dev = dev;
      b->blockno = blockno;
      b->valid = 0;
      b->refcnt = 1;
      btable_insert(b);
      release(&bcache.locks[blockno%NBUCKETS]);
      release(&bcache.lock);
      acquiresleep(&b->lock);
      return b;
    }
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

  acquire(&bcache.lock);
  b->refcnt--;
  if (b->refcnt == 0) {
    // no one is waiting for it.
    b->next->prev = b->prev;
    b->prev->next = b->next;
    b->next = bcache.head.next;
    b->prev = &bcache.head;
    bcache.head.next->prev = b;
    bcache.head.next = b;
    acquire(&bcache.locks[b->blockno%NBUCKETS]);
    btable_remove(b);
    release(&bcache.locks[b->blockno%NBUCKETS]);
  }
  release(&bcache.lock);
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


