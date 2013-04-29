#ifndef __SPI_SLAVE_H
#define __SPI_SLAVE_H
/* building define  */
#define DEBUG
/* Hardware resources */
#include <linux/clk.h>
#include <linux/spi/spi.h>
static unsigned int mx51_ecspi_clkdiv(unsigned int fin, unsigned int fspi);
/* REGISTER DEFINE */
#define MX51_ECSPI_CTRL         0x08
#define MX51_ECSPI_CTRL_ENABLE          (1 <<  0)
#define MX51_ECSPI_CTRL_XCH             (1 <<  2)
#define MX51_ECSPI_CTRL_MODE_MASK       (0xf << 4)
#define MX51_ECSPI_CTRL_POSTDIV_OFFSET  8
#define MX51_ECSPI_CTRL_PREDIV_OFFSET   12
#define MX51_ECSPI_CTRL_CS(cs)          ((cs) << 18)
/* BURST LENGTH OFFSET */
#define MX51_ECSPI_CTRL_BL_OFFSET       20

#define MX51_ECSPI_CONFIG       0x0c
#define MX51_ECSPI_CONFIG_DATA_IDLE(cs)    (1 << ((cs) +  16))    
#define MX51_ECSPI_CONFIG_SCLKPHA(cs)   (1 << ((cs) +  0))
#define MX51_ECSPI_CONFIG_SCLKPOL(cs)   (1 << ((cs) +  4))
/* SBBCTRL control when a spi burst is detected as 
 * ended and RXDATA FIFO advanced:
 * 1 in case this bit is set , the burst is determined
 * by the chip select toggle
 * 0 in case this bit is cleared the burst is detected by burst length*/
#define MX51_ECSPI_CONFIG_SBBCTRL(cs)   (1 << ((cs) +  8))
#define MX51_ECSPI_CONFIG_SSBPOL(cs)    (1 << ((cs) + 12))

#define MX51_ECSPI_INT          0x10
#define MX51_ECSPI_INT_TEEN             (1 <<  0)
#define MX51_ECSPI_INT_RREN             (1 <<  3)

#define MX51_ECSPI_STAT         0x18
#define MX51_ECSPI_STAT_RR              (1 <<  3)

/* ECSPI-1/2 */
#define XCSPI_SIZE 0x3FFF
#define ECSPI1_BASE 0x50010000
#define ECSPI1_END (ECSPI1_BASE + XCSPI_SIE)

#define ECSPI2_BASE 0x63FAC000
#define ECSPI2_END (ECSPI2_BASE + XCSPI_SIE)
enum ecspi_regs_n {
N_ECSPIX_RXDATA,
N_ECSPIX_TXDATA,
N_ECSPIX_CONREG,
N_ECSPIX_CONFIGREG,
N_ECSPIX_INTREG,
N_ECSPIX_DMAREG,
N_ECSPIX_STATUS,
N_ECSPIX_PERIODREG,
N_ECSPIX_TESTREG,
N_ECSPIX_MSGDATA,
N_ECSPIX_MAX,
};

#define ECSPIX_RXDATA(base) ((unsigned int *)(((char *)base)+0x0))
#define ECSPIX_TXDATA(base) ((unsigned int *)(((char *)base)+0x4))
#define ECSPIX_CONREG(base) ((unsigned int *)(((char *)base)+0x8))
#define ECSPIX_CONFIGREG(base) ((unsigned int *)(((char *)base)+0xc))
#define ECSPIX_INTREG(base) ((unsigned int *)(((char *)base)+0x10))
#define ECSPIX_DMAREG(base) ((unsigned int *)(((char *)base)+0x14))
#define ECSPIX_STATUS(base) ((unsigned int *)(((char *)base)+0x18))
#define ECSPIX_PERIODREG(base) ((unsigned int *)(((char *)base)+0x1c))
#define ECSPIX_TESTREG(base) ((unsigned int *)(((char *)base)+0x20))
#define ECSPIX_MSGDATA(base) ((unsigned int *)(((char *)base)+0x40))


/*  CSPI  */
#define CSPI_BASE 0x63FC0000
#define CSPI_END (CSPI_BASE + XCSPI_SIE)

enum cspi_regs_n{
N_CSPI_RXDATA,
N_CSPI_TXDATA,
N_CSPI_CONREG,
N_CSPI_INTREG,
N_CSPI_DMAREG,
N_CSPI_STATUS,
N_CSPI_PERIODREG,
N_CSPI_TESTREG,
N_CSPI_MAX,
};
#define CSPI_RXDATA(base) ECSPIX_RXDATA(base)
#define CSPI_TXDATA(base) ECSPIX_TXDATA(base)
#define CSPI_CONREG(base) ECSPIX_CONREG(base)
#define CSPI_INTREG(base) ((unsigned int *)(((char *)base)+0xc))
#define CSPI_DMAREG(base) ((unsigned int *)(((char *)base)+0x10))
#define CSPI_STATUS(base) ((unsigned int *)(((char *)base)+0x14))
#define CSPI_PERIODREG(base) ((unsigned int *)(((char *)base)+0x18))
#define CSPI_TESTREG(base) ((unsigned int *)(((char *)base)+0x1c))

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
  /*  clk structure */
  struct clk *clk;
  /* clock speed in HZ which arrive to the spiBlock */
  unsigned long spi_clk;
  /* requested port speed in Hz */
  unsigned int speed_hz;
  /*  bitmask modes
   *  valid modes:
   *
   *  */
  unsigned int mode;
  /* set to 1 for master settings */
  uint8_t master;
  uint8_t bits_per_word;
  uint8_t detect_onCS;
#ifdef DEBUG
  /* proc parent for registers interface */
  struct proc_dir_entry *proc_dir;
#endif
}spi_slave_device;
extern spi_slave_device spi_devices[];
extern char *ecspi_regs_name[];
extern char *cspi_regs_name[];
/*  function devlarations */
int delete_spi_regs (spi_slave_device *dev ,char *names[]);
int create_spi_regs (spi_slave_device *dev, char *names[]);

struct proc_dir_entry * register_proc (const char* name,read_proc_t *read_proc,write_proc_t *write_proc,struct proc_dir_entry *parent,void *data );
void unregister_proc(const char* name,struct proc_dir_entry *parent);
void deinit_device (spi_slave_device *dev);
#endif
