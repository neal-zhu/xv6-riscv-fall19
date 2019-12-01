#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "riscv.h"
#include "spinlock.h"
#include "proc.h"
#include "defs.h"
#include "e1000_dev.h"
#include "net.h"

#define TX_RING_SIZE 16
static struct tx_desc tx_ring[TX_RING_SIZE] __attribute__((aligned(16)));
static struct mbuf *tx_mbufs[TX_RING_SIZE];

#define RX_RING_SIZE 16
static struct rx_desc rx_ring[RX_RING_SIZE] __attribute__((aligned(16)));
static struct mbuf *rx_mbufs[RX_RING_SIZE];

// remember where the e1000's registers live.
static volatile uint32 *regs;

struct spinlock e1000_lock;

// called by pci_init().
// xregs is the memory address at which the
// e1000's registers are mapped.
void
e1000_init(uint32 *xregs)
{
  int i;

  initlock(&e1000_lock, "e1000");

  regs = xregs;

  // Reset the device
  regs[E1000_IMS] = 0; // disable interrupts
  regs[E1000_CTL] |= E1000_CTL_RST;
  regs[E1000_IMS] = 0; // redisable interrupts
  __sync_synchronize();

  // [E1000 14.5] Transmit initialization
  memset(tx_ring, 0, sizeof(tx_ring));
  for (i = 0; i < TX_RING_SIZE; i++) {
    tx_ring[i].status = E1000_TXD_STAT_DD;
    tx_mbufs[i] = 0;
  }
  regs[E1000_TDBAL] = (uint64) tx_ring;
  if(sizeof(tx_ring) % 128 != 0)
    panic("e1000");
  regs[E1000_TDLEN] = sizeof(tx_ring);
  regs[E1000_TDH] = regs[E1000_TDT] = 0;
  
  // [E1000 14.4] Receive initialization
  memset(rx_ring, 0, sizeof(rx_ring));
  for (i = 0; i < RX_RING_SIZE; i++) {
    rx_mbufs[i] = mbufalloc(0);
    if (!rx_mbufs[i])
      panic("e1000");
    rx_ring[i].addr = (uint64) rx_mbufs[i]->head;
  }
  regs[E1000_RDBAL] = (uint64) rx_ring;
  if(sizeof(rx_ring) % 128 != 0)
    panic("e1000");
  regs[E1000_RDH] = 0;
  regs[E1000_RDT] = RX_RING_SIZE - 1;
  regs[E1000_RDLEN] = sizeof(rx_ring);

  // filter by qemu's MAC address, 52:54:00:12:34:56
  regs[E1000_RA] = 0x12005452;
  regs[E1000_RA+1] = 0x5634 | (1<<31);
  // multicast table
  for (int i = 0; i < 4096/32; i++)
    regs[E1000_MTA + i] = 0;

  // transmitter control bits.
  regs[E1000_TCTL] = E1000_TCTL_EN |  // enable
    E1000_TCTL_PSP |                  // pad short packets
    (0x10 << E1000_TCTL_CT_SHIFT) |   // collision stuff
    (0x40 << E1000_TCTL_COLD_SHIFT);
  regs[E1000_TIPG] = 10 | (8<<10) | (6<<20); // inter-pkt gap

  // receiver control bits.
  regs[E1000_RCTL] = E1000_RCTL_EN | // enable receiver
    E1000_RCTL_BAM |                 // enable broadcast
    E1000_RCTL_SZ_2048 |             // 2048-byte rx buffers
    E1000_RCTL_SECRC;                // strip CRC
  
  // ask e1000 for receive interrupts.
  regs[E1000_RDTR] = 0; // interrupt after every received packet (no timer)
  regs[E1000_RADV] = 0; // interrupt after every packet (no timer)
  regs[E1000_IMS] = (1 << 7); // RXDW -- Receiver Descriptor Write Back
  __sync_synchronize();
}

static void
mprint(const char *s, struct mbuf *m) {
  return;
  struct eth *eh = mbufpullhdr(m, *eh);
  uint16 type = htons(eh->type);
  const char *stype;
  if (type == ETHTYPE_IP) {
    stype = "UDP";
  } else if (type == ETHTYPE_ARP) {
    stype = "ARP";
  } else {
    stype = "unknow";
  }
  printf("%s mbuf(%p) ethtype %s len %d",s, m, stype, m->len);
  memmove(mbufpush(m, sizeof(*eh)), eh, sizeof(*eh));
  if (type == ETHTYPE_IP) {
    printf(" upd(%s)\n", m->head+(sizeof(struct eth) + sizeof(struct ip) + sizeof(struct udp)));
  } else if (type == ETHTYPE_ARP) {
    struct arp *arp = (struct arp *)(m->head + sizeof(struct eth));
    printf(" arp(sip:%d.%d.%d.%d tip: %d.%d.%d.%d)\n", arp->sip&0xff,(arp->sip>>8)&0xff,(arp->sip>>16)&0xff,(arp->sip>>24)&0xff, 
            arp->tip&0xff,(arp->tip>>8)&0xff,(arp->tip>>16)&0xff,(arp->tip>>24)&0xff);
  } else {
    printf(" unknown(%p)\n", type);
  }
}

int
e1000_transmit(struct mbuf *m)
{
  acquire(&e1000_lock);
  uint32 tdt;
  struct tx_desc *td;
  tdt = regs[E1000_TDT];
  //mprint("e1000_transmit", m);
  td = &tx_ring[tdt];

  if ((td->status & E1000_TXD_STAT_DD) == 0) {
    panic("e1000_transmit");
    release(&e1000_lock);
    return 1;
  }

  if (td->addr) {
    mbuffree(tx_mbufs[tdt]);
  }

  td->addr = (uint64)m->head;
  td->length = m->len;
  td->cmd |= (E1000_TXD_CMD_EOP | E1000_TXD_CMD_RS);
  td->status &= ~E1000_TXD_STAT_DD;
  tx_mbufs[tdt] = m;
  regs[E1000_TDT] = ((tdt+1) % TX_RING_SIZE);
  release(&e1000_lock);
  return 0;
}

static void
e1000_recv(void)
{
  struct mbuf *bufs[RX_RING_SIZE];
  int x = 0;
  acquire(&e1000_lock);
  uint32 rdt, next;
  rdt = regs[E1000_RDT];
  next = (rdt + 1) % RX_RING_SIZE;
  //rdh = regs[E1000_RDH];
  //printf("e1000_recv rdh %d rdt %d next %d\n", rdh, rdt, next);
  for (int i = 0, j = next; i < RX_RING_SIZE; i++) {
    int idx = j + i;
    struct rx_desc *rx = &rx_ring[idx];
    struct mbuf *m = rx_mbufs[idx];
    if (rx->status & E1000_RXD_STAT_DD && rx->status & E1000_RXD_STAT_EOP) {
      //printf("e1000_recv %d rx status %d errors %d: ", idx, rx->status, rx->errors);
      mbufput(m, rx->length);
      mprint("e1000_recv", rx_mbufs[idx]);
      rx_mbufs[idx] = mbufalloc(0);
      if (!rx_mbufs[idx])
        panic("e1000_recv");
      rx->status = 0;
      rx->addr = (uint64)rx_mbufs[idx]->head;
      regs[E1000_RDT] = idx;
      bufs[x++] = m;
    } else {
      //printf("\n");
    }
  }
  release(&e1000_lock);
  for (int i = 0; i < x; i++) {
    net_rx(bufs[i]);
  }
}

void
e1000_intr(void)
{
  e1000_recv();
  // tell the e1000 we've seen this interrupt;
  // without this the e1000 won't raise any
  // further interrupts.
  regs[E1000_ICR];
}
