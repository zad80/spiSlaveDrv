#ifndef __SPI_SLAVE_H
#define __SPI_SLAVE_H
/* building define  */
#define DEBUG
/* Hardware resources */

/* ECSPI-1/2 */
#define XCSPI_SIZE 0x3FFF
#define ECSPI1_BASE 0x50010000
#define ECSPI1_END (ECSPI1_BASE + XCSPI_SIE)

#define ECSPI2_BASE 0x63FAC000
#define ECSPI2_END (ECSPI2_BASE + XCSPI_SIE)

#define ECSPIX_RXDATA(base) (base+0x0)
#define ECSPIX_TXDATA(base) (base+0x4)
#define ECSPIX_CONREG(base) (base+0x8)
#define ECSPIX_CONFIGREG(base) (base+0xc)
#define ECSPIX_INTREG(base) (base+0x10)
#define ECSPIX_DMAREG(base) (base+0x14)
#define ECSPIX_STATUS(base) (base+0x18)
#define ECSPIX_PERIODREG(base) (base+0x1c)
#define ECSPIX_TESTREG(base) (base+0x20)
#define ECSPIX_MSGSATA(base) (base+0x40)

/*  CSPI  */
#define CSPI_BASE 0x63FC0000
#define CSPI_END (CSPI_BASE + XCSPI_SIE)

#define CSPI_RXDATA(base) ECSPIX_RXDATA(base)
#define CSPI_TXDATA(base) ECSPIX_TXDATA(base)
#define CSPI_CONREG(base) ECSPIX_CONREG(base)
#define CSPI_INTREG(base) (base+0xc)
#define CSPI_DMAREG(base) (base+0x10)
#define CSPI_STATUS(base) (base+0x14)
#define CSPI_PERIODREG(base) (base+0x18)
#define CSPI_TESTREG(base) (base+0x1c)

enum spi_type{
  ECSPI,
  CSPI,
};
enum init_status{
  NONE=0,
  MEM_REQUESTED=(1<<1),
  IOREMAPPED=(1<<2),
  IRQ_ASSIGNED=(1<<3),
  PROC_CREATED=(1<<4),
};

/* Struct for handling resource */

typedef struct _spi_slave_device{
  /*  registers base address */
  unsigned int regs_addr;
  /*  iomemmapped address */
  void __iomem *base;
  /* irq number */
  int irq;
  /* enum spi type */
  enum spi_type type;
  /* device name */
  u8 *name;
  /* initialization status */
  u8 initStatus;
#ifdef DEBUG
  /* proc parent for registers interface */
  struct proc_dir_entry *proc_dir;
#endif
}spi_slave_device;
extern spi_slave_device spi_devices[];

/*  function devlarations */
int delete_spi_regs (spi_slave_device *dev ,char *names[]);
int create_spi_regs (spi_slave_device *dev, char *names[]);

struct proc_dir_entry * register_proc (const char* name,read_proc_t *read_proc,write_proc_t *write_proc,struct proc_dir_entry *parent,void *data );
void unregister_proc(const char* name,struct proc_dir_entry *parent);
void deinit_device (spi_slave_device *dev);
#endif
