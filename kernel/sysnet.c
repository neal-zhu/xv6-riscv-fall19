//
// network system calls.
//

#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "riscv.h"
#include "spinlock.h"
#include "proc.h"
#include "defs.h"
#include "fs.h"
#include "sleeplock.h"
#include "file.h"
#include "net.h"

struct sock {
  struct sock *next; // the next socket in the list
  uint32 raddr;      // the remote IPv4 address
  uint16 lport;      // the local UDP port number
  uint16 rport;      // the remote UDP port number
  struct spinlock lock; // protects the rxq
  struct mbufq rxq;  // a queue of packets waiting to be received
};

static struct spinlock lock;
static struct sock *sockets;

void
sockinit(void)
{
  initlock(&lock, "socktbl");
}

int
sockalloc(struct file **f, uint32 raddr, uint16 lport, uint16 rport)
{
  struct sock *si, *pos;

  si = 0;
  *f = 0;
  if ((*f = filealloc()) == 0)
    goto bad;
  if ((si = (struct sock*)kalloc()) == 0)
    goto bad;

  // initialize objects
  si->raddr = raddr;
  si->lport = lport;
  si->rport = rport;
  initlock(&si->lock, "sock");
  mbufq_init(&si->rxq);
  (*f)->type = FD_SOCK;
  (*f)->readable = 1;
  (*f)->writable = 1;
  (*f)->sock = si;

  // add to list of sockets
  acquire(&lock);
  pos = sockets;
  while (pos) {
    if (pos->raddr == raddr &&
        pos->lport == lport &&
	pos->rport == rport) {
      release(&lock);
      goto bad;
    }
    pos = pos->next;
  }
  si->next = sockets;
  sockets = si;
  release(&lock);
  return 0;

bad:
  if (si)
    kfree((char*)si);
  if (*f)
    fileclose(*f);
  return -1;
}

int
sockread(struct sock *s, uint64 addr, int n) {
  struct proc *pr = myproc();
  acquire(&s->lock);
  while (mbufq_empty(&s->rxq)) {
    if (pr->killed) {
      release(&s->lock);
      return -1;
    }
    sleep(s, &s->lock);
  }
  
  struct mbuf *m;
  m = mbufq_pophead(&s->rxq);
  int j = m->len;
  if (j > n)
    panic("smallbuf");
  if (copyout(pr->pagetable, addr, m->head, j) == -1) {
    release(&s->lock);
    return -1;
  }
  release(&s->lock);
  mbuffree(m);
  return j;
}

int
sockwrite(struct sock *s, uint64 addr, int n) {
  struct mbuf *m;
  m = mbufalloc(MBUF_DEFAULT_HEADROOM);
  struct proc *proc = myproc();
  if (copyin(proc->pagetable, m->head, addr, n) == -1) {
    panic("sockwrite");
    return -1;
  }
  mbufput(m, n);
  net_tx_udp(m, s->raddr, s->lport, s->rport);
  return n;
}

void 
sockclose(struct sock *s) {
  acquire(&lock);
  struct sock **l = &sockets;
  while (*l) {
    if ((*l)->lport == s->lport && (*l)->rport == s->rport && (*l)->raddr == s->raddr) {
      *l = s->next;
      break;
    } else {
      l = &(*l)->next;
    }
  }
  release(&lock);
  acquire(&s->lock);
  while (!mbufq_empty(&s->rxq)) {
    mbuffree(mbufq_pophead(&s->rxq));
  }
  release(&s->lock);
  kfree((char*)s);
}

// called by protocol handler layer to deliver UDP packets
void
sockrecvudp(struct mbuf *m, uint32 raddr, uint16 lport, uint16 rport)
{
  acquire(&lock);
  struct sock *pos;
  pos = sockets;
  while (pos) {
    if (pos->raddr == raddr &&
        pos->lport == lport &&
	pos->rport == rport) {
      break;
    }
    pos = pos->next;
  }
  release(&lock);
  if (pos) {
    acquire(&pos->lock);
    mbufq_pushtail(&pos->rxq, m);
    wakeup(pos);
    release(&pos->lock);
  } else {
    mbuffree(m);
  }
}
