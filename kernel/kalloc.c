// Physical memory allocator, for user processes,
// kernel stacks, page-table pages,
// and pipe buffers. Allocates whole 4096-byte pages.

#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "spinlock.h"
#include "riscv.h"
#include "defs.h"

#define IDX(a) (((char*)(a) - kmem.free_start)/PGSIZE)

void freerange(void *pa_start, void *pa_end);

extern char end[]; // first address after kernel.
                   // defined by kernel.ld.

struct run {
  struct run *next;
};

struct {
  struct spinlock lock;
  struct run *freelist;
  int *refs;
  char *free_start;
} kmem;

void
kinit()
{
  initlock(&kmem.lock, "kmem");
  freerange(end, (void*)PHYSTOP);
}

void
freerange(void *pa_start, void *pa_end)
{
  int pgsz = ((char*)pa_end - (char*)pa_start) / (PGSIZE + sizeof(int));
  kmem.refs = (int*)pa_start;
  memset(kmem.refs, 0, sizeof(int) * pgsz);
  for (int i = 0; i < pgsz; i++) {
    kmem.refs[i] = 1; // for kfree
  }
  char *p;
  p = (char*)PGROUNDUP((uint64)pa_start + sizeof(int)*pgsz);
  kmem.free_start = p;
  for(; p + PGSIZE <= (char*)pa_end; p += PGSIZE)
    kfree(p);
}

// Free the page of physical memory pointed at by v,
// which normally should have been returned by a
// call to kalloc().  (The exception is when
// initializing the allocator; see kinit above.)
void
kfree(void *pa)
{
  struct run *r;

  if(((uint64)pa % PGSIZE) != 0 || (char*)pa < kmem.free_start || (uint64)pa >= PHYSTOP)
    panic("kfree");

  acquire(&kmem.lock);
  if (--kmem.refs[IDX(pa)] > 0) {
    release(&kmem.lock);
    return;
  }

  if (kmem.refs[IDX(pa)] < 0) {
    printf("count %d\n", kmem.refs[IDX(pa)]);
    panic("kfree: refcount");
  }

  // Fill with junk to catch dangling refs.
  memset(pa, 1, PGSIZE);

  r = (struct run*)pa;

  r->next = kmem.freelist;
  kmem.freelist = r;
  release(&kmem.lock);
}

// Allocate one 4096-byte page of physical memory.
// Returns a pointer that the kernel can use.
// Returns 0 if the memory cannot be allocated.
void *
kalloc(void)
{
  struct run *r;

  acquire(&kmem.lock);
  r = kmem.freelist;
  if(r) {
    kmem.freelist = r->next;
    kmem.refs[IDX(r)]++;
  }
  release(&kmem.lock);

  if(r) {
    memset((char*)r, 5, PGSIZE); // fill with junk
  }
  return (void*)r;
}

void kref(void *m) {
  acquire(&kmem.lock);
  kmem.refs[IDX(m)]++;
  release(&kmem.lock);
}

