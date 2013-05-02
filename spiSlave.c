/*
 * =====================================================================================
 *
 *       Filename:  spiSlave.c
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  03/18/2013 01:54:59 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  zad 
 *   Organization:  
 *
 * =====================================================================================
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/gpio.h>

#include <mach/hardware.h>
#include <asm/mach/map.h>
#include <mach/iomux-v3.h>
#include <mach/common.h>
#include <mach/iomux-mx53.h>
#include <mach/gpio.h>
#include <mach/spi.h>
#include <linux/em_baseboard.h>

#include <linux/uaccess.h>
#include <linux/cdev.h> // self explainig
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>



#include "spiSlave.h"
#include "spiSlaveUsr.h"
#include "ringBuffer.h"
#include "bits.h"
#include "drv_dbg.h"
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Zad");

//if raw is defined, resources aren't requested to access iomem but
//instead phisical addresses are used.
//#define RAW

#ifdef RAW
#undef writeb 
#undef writew
#undef writel
#undef readb
#undef readw
#undef readl
#define writeb __raw_writeb
#define writew __raw_writew
#define writel __raw_writel
#define readb __raw_readb
#define readw __raw_readw
#define readl __raw_readl
#endif
#define gpio_nr(bank, nr)       ((bank - 1) * 32 + nr)

static struct pad_desc spi1_pads[] = {
        MX53_PAD_SD2_CLK__CSPI2_SCLK,
        MX53_PAD_SD2_CMD__CSPI2_MOSI,
        MX53_PAD_SD2_DATA0__CSPI2_MISO,
        MX53_PAD_SD2_DATA3__GPIO_1_12,
        /* add further gpios as chipselects */
};

static struct pad_desc spi2_pads[] = {
        MX53_PAD_CSI0_D8_CSPI2_SCLK,
        MX53_PAD_CSI0_D9_CSPI2_MOSI,
        MX53_PAD_CSI0_D10_CSPI2_MISO,
        MX53_PAD_CSI0_D11__GPIO_5_29,
        /* add further gpios as chipselects */
};



/* Constants & MACROES */
#define DRIVER "spiSlave"
#define PROC_NAME "spi"
#define RING_SIZE (10)
/* Global variables used by the DD */
ring_buffer_t* readWrite_buffer=NULL;

spi_slave_device spi_devices[]={
#if 0
  {
    .regs_addr=ECSPI1_BASE,
    .base=NULL,
    .irq=36,
    .type=ECSPI,
    .name="ECSPI-1",
    .initStatus=0x0,
    .clk=NULL,
    .speed_hz=6000000,
    .master=0,
    .mode=SPI_CPOL,
    .bits_per_word = 16,
    .detect_onCS=0,
    .gpio_cs=gpio_nr(1, 12),
  },
#endif
  {
    .regs_addr=ECSPI2_BASE,
    .base=NULL,
    .irq=37,
    .type=ECSPI,
    .name="ECSPI-2",
    .initStatus=0x0,
    .clk=NULL,
    .speed_hz=2000000,
    .master=0,
    .mode=SPI_CPOL,
    .bits_per_word = 16,
    .detect_onCS=0,
    .gpio_cs=gpio_nr(5, 29),
  },
#if 0
  {
    .regs_addr=CSPI_BASE,
     .base=NULL,
    .irq=38,
    .type=CSPI,
    .name="CSPI",
    .initStatus=0x0,
    .clk=NULL,
    .speed_hz=2000000,
    .master=0,
    .mode=SPI_CPOL,
    .bits_per_word = 16,
    .detect_onCS=0,
    .gpio_cs=-1,
  },
#endif
};

/*  char device global variables */
struct cdev *spiCdev;
dev_t spiDevt = MKDEV(0,0);
struct class *spiClass;
bool opened=false;
/* mmapping global variable */
// pointer to the vmalloc'd area - alway page aligned
static char *vmalloc_area;
unsigned int *readerIndex=NULL;
unsigned int *writerIndex=NULL;
struct fasync_struct *async_queue;
bool run_timer=false;
unsigned char fake_count=0;
struct timer_list fake_timer;
#define TIMERDELAY 3
/* function declaration */
void add_fake_data(unsigned long arg);
int addToBuffer(char newValue);
int spi_open(struct inode *inode,struct file *filp);
int spi_release(struct inode *inode, struct file *filp);
loff_t spi_llseek(struct file *,loff_t,int);
ssize_t spi_read_char(struct file*, char __user*, size_t,loff_t *);
ssize_t spi_write_char(struct file*, const char __user*,size_t,loff_t *);
static int spi_mmap(struct file *filp, struct vm_area_struct *vma);
int spi_ioctl(struct inode *inode,struct file *filp,unsigned int cmd,unsigned long arg);
static int spi_p_fasync(int fd, struct file *filp, int mode);
/* init file_operation using c tag style*/
struct file_operations spiChar_fops={
	.owner = THIS_MODULE,
	.open = spi_open,
	.release = spi_release,
        .mmap = spi_mmap,
	.llseek = spi_llseek,
	.read = spi_read_char,
	.write = spi_write_char,
        .ioctl = spi_ioctl,
        .fasync = spi_p_fasync,
};



static int __init init_dd(void);
static void __exit exit_dd(void);
int write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data);


void set_chipSelect(spi_slave_device *dev){
  if(dev==NULL)
    return;
  if(dev->master)
  {
    gpio_direction_output(dev->gpio_cs, dev->mode & SPI_CS_HIGH ? 0 : 1);
    gpio_set_value(dev->gpio_cs, !(dev->mode & SPI_CS_HIGH));
    debugPrintFL(50,DRIVER,"direction: %x,value: %x\n",dev->mode & SPI_CS_HIGH ? 0 : 1,!(dev->mode & SPI_CS_HIGH));
  }else{
    gpio_direction_input(dev->gpio_cs);
    debugPrintFL(50,DRIVER,"direction slave:INPUT" );

  }
}


/* Function implementation for char device */
static int spi_p_fasync(int fd, struct file *filp, int mode)
{
  int ret=0;
  debugPrintFL(50,DRIVER, "spi_p_fasync called\n");
  return fasync_helper(fd, filp, mode, &async_queue);

  if ((ret = fasync_helper(fd, filp, mode, &async_queue)) < 0)
    return ret;

  if (mode) 
    ret = __f_setown(filp, task_pid(current), PIDTYPE_PID, 0);
  return ret;
}
ssize_t spi_write_char(struct file *filp,const char __user *buff, size_t size,loff_t *offset)
{
	debugPrintFL(50,DRIVER, "spi_write_char called\n");
	return size;
}
ssize_t spi_read_char(struct file *filp,char __user *buff, size_t size, loff_t *offset)
{
	debugPrintFL(50,DRIVER, "spi_read_char called\n");
	return 0;
}
int spi_open(struct inode *inode, struct file *filp)
{
	debugPrintFL(50,DRIVER, "spi_open called\n");
       return 0;
}
int spi_release(struct inode *inode, struct file *filp)
{
	debugPrintFL(50,DRIVER, "spi_release called\n");
        /* remove this filp from the asynchronously notified filp's */
        spi_p_fasync(-1, filp, 0);
	return 0;
}
loff_t spi_llseek(struct file* filp, loff_t offset, int type)
{
	debugPrintFL(50,DRIVER, "spi_llseek called\n");
	return offset;
}

void spiSlave_vma_open(struct vm_area_struct *vma){
	debugPrintFL(50,DRIVER, "spiSlave_vma_open called\n");
        if(opened==false)
          opened=true;
}
void spiSlave_vma_close(struct vm_area_struct *vma){
	debugPrintFL(50,DRIVER, "spiSlave_vma_close called\n");
        opened=false;
}
int spi_ioctl(struct inode *inode,struct file *filp,unsigned int cmd,unsigned long arg){
  // TODO: serialize the ioctl with a semaphore
  int err=0;
  debugPrintFL(50,DRIVER, "CMD = %i\n", cmd);
  switch (cmd)
  {
    case C_SPISLAVE_IOSETACQ:
      {
        YSPISLAVE_SETACQ  data;
        debugPrintF(DRIVER,"doing C_SPISLAVE_IOSETACQ IOCTL.\n");
        if ((err = copy_from_user(&data, (YSPISLAVE_SETACQ *)arg, sizeof(YSPISLAVE_SETACQ))) < 0) {
          debugPrintF(DRIVER,"error in copy_from_user case C_SPISLAVE_IOSETACQ: at\n");
          goto TEST_IOCTL_FAIL;
        }
        /* do the stuff and set data.res */
        data.res=0;
        if(data.acq_ena==0 ){
          //stop acquisition
          debugPrintF(DRIVER,"stop reading from spi\n");
          run_timer=false;
          del_timer_sync(&fake_timer);
          fake_count=0;
        }else if(data.acq_ena==1){
          //start acquisition
          debugPrintF(DRIVER,"start reading from spi\n");
          if(run_timer==false){
            /* set run_time to true */
            run_timer=true;
            /* start the timer */
            fake_timer.data=(unsigned long)&fake_count;
            fake_timer.function=add_fake_data;
            /* 1 Second in the future */
            fake_timer.expires=jiffies + TIMERDELAY;
            debugPrintF(DRIVER,"call add_timer\n");
            add_timer(&fake_timer);
            debugPrintF(DRIVER,"add_timer called\n");
          }
        }else{
          debugPrintF(DRIVER,"Invalid acq_value %d\n",data.acq_ena);
          data.res=EIOCTL_ACQVAL;
        }
        if ((err = copy_to_user((YSPISLAVE_SETACQ *)arg, &data, sizeof(YSPISLAVE_SETACQ))) < 0) {
          debugPrintF(DRIVER,"error in copy_to_user at\n");
          goto TEST_IOCTL_FAIL;
        }
      }
      break;
    case C_SPISLAVE_IOSETREADEP:
      {
        YSPISLAVE_SETREADERP  data;
        debugPrintF(DRIVER,"doing C_SPISLAVE_IOSETREADEP IOCTL.\n");
        if ((err = copy_from_user(&data, (YSPISLAVE_SETREADERP *)arg, sizeof(YSPISLAVE_SETREADERP))) < 0) {
          debugPrintF(DRIVER,"error in copy_from_user case C_SPISLAVE_IOSETREADEP: at\n");
          goto TEST_IOCTL_FAIL;
        }
        data.res=0;
        /* do the stuff and set data.res */
        if(data.readerPtr<4 || data.readerPtr>VMEMSIZE){
          debugPrintF(DRIVER,"Invalid readerPtr %d value\n",data.readerPtr); 
          data.res=EIOCTL_OUTSIDEB;
        }else if(*readerIndex<=*(writerIndex) && data.readerPtr >(*writerIndex)){
          debugPrintF(DRIVER,"Invalid readerPtr %d cannot go ahead writer %d\n",
              data.readerPtr,*writerIndex); 
          data.res=EIOCTL_OUTSIDEB;
        }else{
          *readerIndex=data.readerPtr;
          debugPrintF(DRIVER,"New readerIndex value %d\n",*readerIndex);
        }
        if ((err = copy_to_user((YSPISLAVE_SETREADERP *)arg, &data, sizeof(YSPISLAVE_SETREADERP))) < 0) {
          debugPrintF(DRIVER,"error in copy_to_user at\n");
          goto TEST_IOCTL_FAIL;
        }
      }
      break;
    default:
        err = -EINVAL;
      goto TEST_IOCTL_FAIL;
  }
  return 0;

TEST_IOCTL_FAIL:
  return err;
}
static struct vm_operations_struct spiSlave_vm_ops = {
  .open = spiSlave_vma_open,
  .close = spiSlave_vma_close,
};


int alloc_vmem(void){
  int ret=0,i=0;
  if ((vmalloc_area = (char *)vmalloc(VMEMSIZE)) == NULL) {
    ret = -ENOMEM;
    vfree(vmalloc_area);
    vmalloc_area=NULL;
  }else{
    /* mark the pages reserved */
    for (i = 0; i < VMEMSIZE; i+= PAGE_SIZE) {
      SetPageReserved(vmalloc_to_page((void *)(((unsigned long)vmalloc_area) + i)));
    }
  }
  writerIndex=(unsigned int*)&vmalloc_area[WRITE_IDX_START];
  readerIndex=(unsigned int*)&vmalloc_area[READ_IDX_START];
  *readerIndex=DATA_IDX_START;
  *writerIndex=DATA_IDX_START;
  return ret;
}
void free_vmem(void){
  int i=0;
  if(vmalloc_area){
    /* unreserve the pages */
    for (i = 0; i < VMEMSIZE; i+= PAGE_SIZE) {
      ClearPageReserved(vmalloc_to_page((void *)(((unsigned long)vmalloc_area) + i)));
    }
    /* free the memory areas */
    vfree(vmalloc_area);
    vmalloc_area=NULL;
    writerIndex=NULL;
    readerIndex=NULL;
  }
}
// helper function, mmap's the vmalloc'd area which is not physically contiguous
int mmap_vmem(struct file *filp, struct vm_area_struct *vma)
{
  int ret;
  long length = vma->vm_end - vma->vm_start;
  unsigned long start = vma->vm_start;
  char *vmalloc_area_ptr = (char *)vmalloc_area;
  unsigned long pfn;

  /* check length - do not allow larger mappings than the number of
     pages allocated */
  if (length > VMEMSIZE){
    printk(KERN_ERR"asking to much memory %ld only available %d\n",length,VMEMSIZE);
    return -ENOMEM;
  }
  vma->vm_flags&=~(VM_WRITE|VM_EXEC);
  /* loop over all pages, map it page individually */
  while (length > 0) {
    pfn = vmalloc_to_pfn(vmalloc_area_ptr);
    if ((ret = remap_pfn_range(vma, start, pfn, PAGE_SIZE,PAGE_SHARED)) < 0) {
      return ret;
    }    
    start += PAGE_SIZE;
    vmalloc_area_ptr += PAGE_SIZE;
    length -= PAGE_SIZE;
  }
  vma->vm_ops=&spiSlave_vm_ops;
  spiSlave_vma_open(vma);
  return 0;
}

/* character device mmap method */
static int spi_mmap(struct file *filp, struct vm_area_struct *vma)
{
  debugPrintFL(50,DRIVER,"mmap is invoked\n");
  /* at offset 0 we map the vmalloc'd area */
  if (vma->vm_pgoff == 0 && opened==false) {
    return mmap_vmem(filp, vma);
  }
  /* at any other offset we return an error */
  return -EIO;
}


/* init_char function tasks:
 * allocate a major
 * register a minor range
 * allocate a cdev and register a character device
 * ask for device file insertion inside /dev by udev
 * */
int numberOfMinor=1;
int char_init(void)
{
	int iterator=0;
	int result=0;
        if((result=alloc_vmem())){
		printk(KERN_WARNING "spiChar: unable to allocate Memory\n");
                return result;
        }
	/* ask for a free major with 2 minor , starting from 0
	 * for a device named "spiDev"
	 * */
	if((result=alloc_chrdev_region(&spiDevt,0,numberOfMinor,"spiDev"))<0)
	{
		/* error detected , unable to allocate a major */
		printk(KERN_WARNING "spiChar: unable to allocate a major\n");
		return result;
	}
	spiCdev = cdev_alloc();
	cdev_init(spiCdev,&spiChar_fops);
	if((result=cdev_add(spiCdev,spiDevt,numberOfMinor))<0)
	{
		/* gracefully fail */
		printk(KERN_WARNING "spiChar: failed to add cdev\n");
		cdev_del(spiCdev);
		unregister_chrdev_region(spiDevt,numberOfMinor);
		return result;
	}
	spiClass= class_create(THIS_MODULE,"spiChar");
	
	for(iterator=0;iterator<numberOfMinor;iterator++)
	{
          /*  device_create in this case creates an entry inside 
           *  /sys/device/virtual/spiChar/spiChar%d 
           *  it will be possible to add attrubute using the pointer returned by 
           *  device_create
           *  */
		if( (NULL==device_create(spiClass,NULL,MKDEV(MAJOR(spiDevt),iterator),NULL,"spiChar%d",iterator))<0)
		{
			printk(KERN_WARNING "spiChar: failed to create device %d\n",iterator);
			for(iterator--;iterator>=0;iterator--)
			{
				device_destroy(spiClass,MKDEV(MAJOR(spiDevt),iterator));
			}
			class_destroy(spiClass);
			cdev_del(spiCdev);
			unregister_chrdev_region(spiDevt,numberOfMinor);
			return -1;
		}else{
			printk(KERN_WARNING "spiChar: created device %d\n",iterator);
		}

	}
	return result;
}


/* char_exit function tasks:
 * unregister device file from /dev
 * unregister and free cdev
 * unregister minors
 * */

void  char_exit(void){
	int iterator=0;
	for(iterator=0;iterator<numberOfMinor;iterator++)
	{
		device_destroy(spiClass,MKDEV(MAJOR(spiDevt),iterator));
	}
	class_destroy(spiClass);
	cdev_del(spiCdev);
	unregister_chrdev_region(spiDevt,numberOfMinor);
}

/* create a timer for debugging purpose*/
/* timer function for debugging purpose*/
unsigned long int lastFasync=0;
void add_fake_data(unsigned long arg){
  unsigned char *n = (unsigned char*)arg;
  if(addToBuffer(*n)){
   printk(KERN_ERR"No space left\n");
  }else{
    if (async_queue && ((jiffies - lastFasync)> 10*HZ)){
      debugPrintF(DRIVER,"writerIndex=%d\n",*writerIndex);
      kill_fasync(&async_queue, SIGIO, POLL_IN);
      lastFasync=jiffies;
    }
    if(run_timer){
      *n=*n+1;
      /*  rescedule the time */
      fake_timer.data=(unsigned long)&fake_count;
      fake_timer.function=add_fake_data;
      /* 1 Second in the future */
      //fake_timer.expires=jiffies + HZ;
      fake_timer.expires=jiffies + TIMERDELAY;
      add_timer(&fake_timer);
    }
  }
}

static irqreturn_t spi_slave_isr(int irq, void *dev_id)
{
#if 0
  struct spi_slave_device *spi_dev = dev_id;
  while (spi_imx->devtype_data->rx_available(spi_imx)) {
    spi_imx->rx(spi_imx);
    spi_imx->txfifo--;
  }

  if (spi_imx->count) {
    spi_imx_push(spi_imx);
    return IRQ_HANDLED;
  }

  if (spi_imx->txfifo) {
    /* No data left to push, but stilremove_proc_entryl waiting for rx data,
     * enable receive data available interrupt.
     */
    spi_imx->devtype_data->intctrl(
        spi_imx, MXC_INT_RR);
    return IRQ_HANDLED;
  }

  spi_imx->devtype_data->intctrl(spi_imx, 0);
  complete(&spi_imx->xfer_done);
/* and signal asynchronous readers*/
         if (async_queue)
                kill_fasync(async_queue, SIGIO, POLL_IN);
#endif
  return IRQ_HANDLED;
}
struct proc_dir_entry *spi_parent =NULL;
/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  get_ecspi_addr
 *  Description:  
 * =====================================================================================
 */
unsigned int* get_ecspi_addr( char *regName, unsigned int *base)
{
  unsigned int *addr=NULL;
  if(regName==NULL){
    printkE("","regNum is null\n");
    return NULL;
  }
  if(strcmp(regName,ecspi_regs_name[N_ECSPIX_RXDATA])==0){
    addr=ECSPIX_RXDATA(base);
  }else if(strcmp(regName,ecspi_regs_name[N_ECSPIX_TXDATA])==0){
    addr=ECSPIX_TXDATA(base);
  }else if(strcmp(regName,ecspi_regs_name[N_ECSPIX_CONREG])==0){
    addr=ECSPIX_CONREG(base);
  }else if(strcmp(regName,ecspi_regs_name[N_ECSPIX_CONFIGREG])==0){
    addr=ECSPIX_CONFIGREG(base);
  }else if(strcmp(regName,ecspi_regs_name[N_ECSPIX_INTREG])==0){
    addr=ECSPIX_INTREG(base);
  }else if(strcmp(regName,ecspi_regs_name[N_ECSPIX_DMAREG])==0){
    addr=ECSPIX_DMAREG(base);
  }else if(strcmp(regName,ecspi_regs_name[N_ECSPIX_STATUS])==0){
    addr=ECSPIX_STATUS(base);
  }else if(strcmp(regName,ecspi_regs_name[N_ECSPIX_PERIODREG])==0){
    addr=ECSPIX_PERIODREG(base);
  }else if(strcmp(regName,ecspi_regs_name[N_ECSPIX_TESTREG])==0){
    addr=ECSPIX_TESTREG(base);
   }else if(strcmp(regName,ecspi_regs_name[N_ECSPIX_MSGDATA])==0){
    addr=ECSPIX_MSGDATA(base);
   } 
  return addr;
}		/* -----  end of function get_ecspi_addr  ----- */


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  get_cspi_addr
 *  Description:  
 * =====================================================================================
 */
unsigned int* get_cspi_addr( char *regName, unsigned int *base)
{
  unsigned int *addr=NULL;
  if(regName==NULL){
    printkE("","regNum is null\n");
    return NULL;
  }
  if(strcmp(regName,cspi_regs_name[N_CSPI_RXDATA])==0){
    addr=CSPI_RXDATA(base);
  }else if(strcmp(regName,cspi_regs_name[N_CSPI_TXDATA])==0){
    addr=CSPI_TXDATA(base);
  }else if(strcmp(regName,cspi_regs_name[N_CSPI_CONREG])==0){
    addr=CSPI_CONREG(base);
  }else if(strcmp(regName,cspi_regs_name[N_CSPI_INTREG])==0){
    addr=CSPI_INTREG(base);
  }else if(strcmp(regName,cspi_regs_name[N_CSPI_DMAREG])==0){
    addr=CSPI_DMAREG(base);
  }else if(strcmp(regName,cspi_regs_name[N_CSPI_STATUS])==0){
    addr=CSPI_STATUS(base);
  }else if(strcmp(regName,cspi_regs_name[N_CSPI_PERIODREG])==0){
    addr=CSPI_PERIODREG(base);
  }else if(strcmp(regName,cspi_regs_name[N_ECSPIX_TESTREG])==0){
    addr=CSPI_TESTREG(base);
  }
  return addr;
}		/* -----  end of function get_cspi_addr  ----- */

spi_slave_device * getDevice(char *name){
  int i=0;
  int l= sizeof(spi_devices)/sizeof(spi_devices[0]);
  if(name==NULL)
  {
    printkE(DRIVER,"Error null pointer passed as name\n");
  }
  for(i=0;i<l;i++){
    if(strcasecmp(name,spi_devices[i].name)==0){
      return &spi_devices[i];
    }
  }
  return NULL;
}

unsigned int * getDeviceBase(char *name){
  unsigned int *base=NULL;
  spi_slave_device *dev = NULL;
  if(name==NULL)
  {
    printkE(DRIVER,"Error null pointer passed as name\n");
  }
  dev = getDevice(name);
    if(dev){
#ifdef RAW
      base= (unsigned int *)dev->regs_addr;
#else
      base= dev->base;
#endif
    }
  return base;
}

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  write_proc_ecspi1
 *  Description:  
 * =====================================================================================
 */
int write_proc_ecspi1(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
 char *end=NULL;
  unsigned int *addr=NULL;
  unsigned int val=simple_strtol(buffer,&end,0);
  unsigned int *base=getDeviceBase("ECSPI-1");
  if(base==NULL){
    printkE(DRIVER,"Failed to get base address\n");
    return -EINVAL;
  }

  if(data==NULL){
    printkE("","data is null\n");
    return -EINVAL;
  }
  if(buffer==end){
    printkE("","Invalid number passed\n");
  }
  if((addr=get_ecspi_addr(data,base))!=NULL){
    debugPrintFL(50,DRIVER,"addr=%p,val=0x%x\n",addr,val);
    if(strcmp(data,ecspi_regs_name[N_ECSPIX_CONFIGREG])==0){
      spi_slave_device *dev=getDevice("ECSPI-1");
      if(dev){
        /*  Polarity of the CS signal not used  */
        if(GET_BITS(val,12,12)){
          //set
          dev->mode |= SPI_CS_HIGH;
        }else{
          //clear
          dev->mode &= ~SPI_CS_HIGH;
        }
      }
    }

    writel(val,addr);
  }else{
    printkE(__FUNCTION__,"register not Found\n");
  }
  return count;
}

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  write_proc_ecspi2
 *  Description:  
 * =====================================================================================
 */
int write_proc_ecspi2(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
 char *end=NULL;
  unsigned int *addr=NULL;
  unsigned int val=simple_strtol(buffer,&end,0);
  unsigned int *base=getDeviceBase("ECSPI-2");
  if(base==NULL){
    printkE(DRIVER,"Failed to get base address\n");
    return -EINVAL;
  }

  if(data==NULL){
    printkE("","data is null\n");
    return -EINVAL;
  }
  if(buffer==end){
    printkE("","Invalid number passed\n");
  }
  if((addr=get_ecspi_addr(data,base))){
    debugPrintFL(50,DRIVER,"addr=%p,val=0x%x\n",addr,val);
    if(strcmp(data,ecspi_regs_name[N_ECSPIX_CONFIGREG])==0){
      spi_slave_device *dev=getDevice("ECSPI-2");
      if(dev){
        /*  Polarity of the CS signal not used  */
        if(GET_BITS(val,12,12)){
          //set
          dev->mode |= SPI_CS_HIGH;
        }else{
          //clear
          dev->mode &= ~SPI_CS_HIGH;
        }
      }
    }
    if(strcmp(data,ecspi_regs_name[N_ECSPIX_CONREG])==0){
      spi_slave_device *dev=getDevice("ECSPI-2");
      if(dev){
          //set master flag
          dev->master=GET_BITS(val,4,4);
      }
    }
    if(strcmp(data,ecspi_regs_name[N_ECSPIX_TXDATA])==0){
      spi_slave_device *dev=getDevice("ECSPI-2");
      if(dev){
        set_chipSelect(dev);
      }
    }
    writel(val,addr);
    if(strcmp(data,ecspi_regs_name[N_ECSPIX_TXDATA])==0){
      spi_slave_device *dev=getDevice("ECSPI-2");
      if(dev){
        if(dev->master)
        {
          gpio_direction_output(dev->gpio_cs, dev->mode & SPI_CS_HIGH ? 0 : 1);
          gpio_set_value(dev->gpio_cs,0);
        }else{
          gpio_direction_input(dev->gpio_cs);
          debugPrintFL(50,DRIVER,"direction slave:INPUT" );
        }
      }
    }
  }else{
    printkE(__FUNCTION__,"register not Found\n");
  }
  return count; 
}

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  write_proc_cspi
 *  Description:  
 * =====================================================================================
 */
int write_proc_cspi(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
  char *end=NULL;
  unsigned int *addr=NULL;
  unsigned int val=simple_strtol(buffer,&end,0);
  unsigned int *base=getDeviceBase("CSPI");
  if(base==NULL){
    printkE(DRIVER,"Failed to get base address\n");
    return -EINVAL;
  }


  if(data==NULL){
    printkE("","data is null\n");
    return -EINVAL;
  }
  if(buffer==end){
    printkE("","Invalid number passed\n");
  }
  if((addr=get_cspi_addr(data,base))){
    debugPrintFL(50,DRIVER,"addr=%p,val=0x%x\n",addr,val);
    writel(val,addr);
  }else{
    printkE(__FUNCTION__,"register not Found\n");
  }
  return count; 
}		/* -----  end of function write_proc  ----- */


int write_spi_reg(uint8_t n,uint32_t val,spi_slave_device *dev)
{

  unsigned int *addr=NULL;
  if(dev==NULL){
    debugPrintFL(50,DRIVER,"invalid device passed\n");
    return -EINVAL;
  }
  if(n<0 || n>=((dev->type==ECSPI)?N_ECSPIX_MAX:N_CSPI_MAX)){
    debugPrintFL(50,DRIVER,"invalid reg number %d passed\n",n);
    return -EINVAL;
  }
#ifndef RAW
  if(dev->base==NULL){
    debugPrintFL(50,DRIVER,"invalid base address passed\n");
    return -EINVAL;
  }
#endif
  if((addr=(get_ecspi_addr(ecspi_regs_name[n],getDeviceBase(dev->name))))){
    writel(val,addr);
    debugPrintFL(50,DRIVER,"addr=%p,val=0x%x\n",addr,val);
    return 0;
  }else{
    debugPrintFL(50,DRIVER,"address not found\n");
    return -EINVAL;
  }

}		/* -----  end of function read_spi_reg  ----- */

int32_t read_spi_reg(uint8_t n,spi_slave_device *dev)
{

  unsigned int val=0x0;
  unsigned int *addr=NULL;
  if(dev==NULL){
    debugPrintFL(50,DRIVER,"invalid device passed\n");
    return -EINVAL;
  }
  if(n<0 || n>=((dev->type==ECSPI)?N_ECSPIX_MAX:N_CSPI_MAX)){
    debugPrintFL(50,DRIVER,"invalid reg number %d passed\n",n);
    return -EINVAL;
  }
#ifndef RAW
  if(dev->base==NULL){
    debugPrintFL(50,DRIVER,"invalid base address passed\n");
    return -EINVAL;
  }
#endif
  if((addr=(get_ecspi_addr(ecspi_regs_name[n],getDeviceBase(dev->name))))){
    val=readl(addr);
    debugPrintFL(50,DRIVER,"addr=%p,val=0x%x\n",addr,val);
    return val;
  }else{
    debugPrintFL(50,DRIVER,"address not found\n");
    return -EINVAL;
  }

}		/* -----  end of function read_spi_reg  ----- */


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  read_proc_cspi
 *  Description:  
 * =====================================================================================
 */
int read_proc_cspi(char *page, char **start, off_t offset,int count, int *eof, void *data)
{

  char *regName = data;
  int read=0;
  unsigned int *addr=NULL;
  unsigned int val=0;
  if(data==NULL){
    printkE("","data is null\n");
    return -EINVAL;
  }
  if(count>PAGE_SIZE){
    count=PAGE_SIZE;
  }
  if((addr=(get_cspi_addr(regName,getDeviceBase("CSPI"))))){
    val=readl(addr);
    debugPrintFL(50,DRIVER,"addr=%p,val=0x%x\n",addr,val);
    read+=sprintf(page+read,"0x%08x\n",val);
  }else{
    printkE(__FUNCTION__,"register not Found\n");
  }
  *eof=1;
  return read;
}		/* -----  end of function read_proc  ----- */



int parse_ecspi_reg(char *regName, unsigned int val,char * page){
  int read=0;
  if(regName==NULL || page==NULL){
    printkE(DRIVER,"null arguments\n");
    return read;
  }
  if(strcmp(regName,ecspi_regs_name[N_ECSPIX_CONREG])==0){
    read+=sprintf(page+read,"burstl=0x%x ",GET_BITS(val,20,31));
    read+=sprintf(page+read,"chnnelSelect=0x%x ",GET_BITS(val,18,19));
    read+=sprintf(page+read,"spiDataRdyCtl=0x%x ",GET_BITS(val,16,17));
    read+=sprintf(page+read,"preDivider=0x%x ",GET_BITS(val,12,15));
    read+=sprintf(page+read,"postDivider=0x%x \n",GET_BITS(val,8,11));
    read+=sprintf(page+read,"channelMode 0x%x ",GET_BITS(val,4,7));
    read+=sprintf(page+read,"startModeControl 0x%x ",GET_BITS(val,3,3));
    read+=sprintf(page+read,"initExchange 0x%x ",GET_BITS(val,2,2));
    read+=sprintf(page+read,"HwTriggerEnabled 0x%x ",GET_BITS(val,1,1));
    read+=sprintf(page+read,"spiPortEnabled 0x%x \n",GET_BITS(val,0,0));
  }else if(strcmp(regName,ecspi_regs_name[N_ECSPIX_CONFIGREG])==0){
    read+=sprintf(page+read,"lenghtInHTmode=0x%x ",GET_BITS(val,24,28));
    read+=sprintf(page+read,"inactiveClkState=0x%x ",GET_BITS(val,20,23));
    read+=sprintf(page+read,"inactiveDataState=0x%x ",GET_BITS(val,16,19));
    read+=sprintf(page+read,"chipSelectPolarity=0x%x\n",GET_BITS(val,12,15));
    read+=sprintf(page+read,"chipSelMarkBurst=0x%x ",GET_BITS(val,8,11));
    read+=sprintf(page+read,"clkPolarity 0x%x ",GET_BITS(val,4,7));
    read+=sprintf(page+read,"clkPhase 0x%x ",GET_BITS(val,0,3));
  }else if(strcmp(regName,ecspi_regs_name[N_ECSPIX_INTREG])==0){
    read+=sprintf(page+read,"transferCompleteIrqEn=0x%x ",GET_BITS(val,7,7));
    read+=sprintf(page+read,"RXFifoOverflowIrqEn=0x%x ",GET_BITS(val,6,6));
    read+=sprintf(page+read,"RxFifoFullIrqEn=0x%x ",GET_BITS(val,5,5));
    read+=sprintf(page+read,"RxDataRequestIrqEn=0x%x\n",GET_BITS(val,4,4));
    read+=sprintf(page+read,"RxReadyIrqEn=0x%x ",GET_BITS(val,3,3));
    read+=sprintf(page+read,"TxFullIrqEna 0x%x ",GET_BITS(val,2,2));
    read+=sprintf(page+read,"TxFifoDataReqIrqEna 0x%x ",GET_BITS(val,1,1));
    read+=sprintf(page+read,"TxFifoEmptyIrqEna 0x%x ",GET_BITS(val,0,0));
  }else if(strcmp(regName,ecspi_regs_name[N_ECSPIX_DMAREG])==0){
  }else if(strcmp(regName,ecspi_regs_name[N_ECSPIX_STATUS])==0){
    read+=sprintf(page+read,"transferComplete=0x%x ",GET_BITS(val,7,7));
    read+=sprintf(page+read,"RXFifoOverflow=0x%x ",GET_BITS(val,6,6));
    read+=sprintf(page+read,"RxFifoFull=0x%x ",GET_BITS(val,5,5));
    read+=sprintf(page+read,"RxDataRequest=0x%x\n",GET_BITS(val,4,4));
    read+=sprintf(page+read,"RxReady=0x%x ",GET_BITS(val,3,3));
    read+=sprintf(page+read,"TxFull 0x%x ",GET_BITS(val,2,2));
    read+=sprintf(page+read,"TxFifoDataReq 0x%x ",GET_BITS(val,1,1));
    read+=sprintf(page+read,"TxFifoEmpty 0x%x ",GET_BITS(val,0,0));
   }else if(strcmp(regName,ecspi_regs_name[N_ECSPIX_PERIODREG])==0){
  }else if(strcmp(regName,ecspi_regs_name[N_ECSPIX_TESTREG])==0){
  }else if(strcmp(regName,ecspi_regs_name[N_ECSPIX_MSGDATA])==0){
  } 
  return read;

}

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  read_proc_ecspi
 *  Description:  
 * =====================================================================================
 */
int read_proc_ecspi(char *page, char **start, off_t offset,int count, int *eof, void *data,unsigned int *base)
{
  char *regName = data;
  int read=0;
  unsigned int *addr=NULL;
  unsigned int val=0;
  if(data==NULL){
    printkE("","data is null\n");
    return -EINVAL;
  }
  if(count>PAGE_SIZE){
    count=PAGE_SIZE;
  }

  if((addr=get_ecspi_addr(regName,base))){
    val=readl(addr);
    debugPrintFL(50,DRIVER,"addr=%p,val=0x%x\n",addr,val);
    read+=sprintf(page+read,"0x%08x\n",val);
    read+=parse_ecspi_reg(regName,val,page+read);
  }else{
    printkE(__FUNCTION__,"register not Found\n");
  }
  *eof=1;
  return read;
}		/* -----  end of function read_proc  ----- */

int read_proc_ecspi1(char *page, char **start, off_t offset,int count, int *eof, void *data)
{
  if(data==NULL)
    return -EINVAL;
  if(count>PAGE_SIZE){
    count=PAGE_SIZE;
  }
 debugPrintFL(50,DRIVER,"%s: to Read %s\n",__FUNCTION__,(char *)data);
 return read_proc_ecspi(page,start,offset,count,eof,data,getDeviceBase("ECSPI-1"));
}		/* -----  end of function read_proc  ----- */

int read_proc_ecspi2(char *page, char **start, off_t offset,int count, int *eof, void *data)
{
  if(data==NULL)
    return -EINVAL;
  if(count>PAGE_SIZE){
    count=PAGE_SIZE;
  }
 debugPrintFL(50,DRIVER,"%s: to Read %s\n",__FUNCTION__,(char *)data);
 return read_proc_ecspi(page,start,offset,count,eof,data,getDeviceBase("ECSPI-2"));
}		/* -----  end of function read_proc  ----- */


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  spi_delete_proc_entry
 *  Description:  this function create : the directory of the device and all
 *  the proc entry to the register access
 * =================remove_proc_entry====================================================================
 */
int spi_delete_proc_entry (spi_slave_device *dev,struct proc_dir_entry *parent )
{
  int res=0;
  if (dev ==NULL || dev->proc_dir==NULL){
    res=EINVAL;
    printk( KERN_WARNING "%s:NULL spi_device passed\n",__FUNCTION__);
    return res;
  }
 
  if(dev->type == ECSPI)
  {
    res = delete_spi_regs(dev,ecspi_regs_name);
  }
  else if(dev->type == CSPI)
  {
    res = delete_spi_regs(dev,cspi_regs_name);
  }
  unregister_proc(dev->name,spi_parent);
  return res;
}		/* -----  end of function spi_create_proc_entry  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  spi_create_proc_entry
 *  Description:  this function create : the directory of the device and all
 *  the proc entry to the registeremove_proc_entryr access
 * =====================================================================================
 */
int spi_create_proc_entry (spi_slave_device *dev,struct proc_dir_entry *parent )
{
  int res=0;
  if (dev ==NULL){
    res=EINVAL;
    printk( KERN_WARNING "%s:NULL spi_device passed\n",__FUNCTION__);
    return res;
  }
  /* 1) First create the new directory for the device */
  dev->proc_dir = proc_mkdir(dev->name,parent);
  if(dev->proc_dir==NULL){
    printk(KERN_WARNING "%s failed to create proc dir %s\n",__FUNCTION__,dev->name);
    res=ECANCELED;
    return res;
  }
  if(dev->type == ECSPI)
  {
    res = create_spi_regs(dev,ecspi_regs_name);
  }
  else if(dev->type == CSPI)
  {
    res = create_spi_regs(dev,cspi_regs_name);

  }
  return res;
}		/* -----  end of function spi_create_proc_entry  ----- */

int create_spi_regs (spi_slave_device *dev, char *names[] )
{
  int res=0,i=0;
  read_proc_t *read_proc=NULL;
  write_proc_t* write_proc=NULL;
  if (dev ==NULL || dev->proc_dir==NULL || names==NULL){
    res=EINVAL;
    printk( KERN_WARNING "%s:NULL spi_device passed\n",__FUNCTION__);
    return res;
  }
  if(strcmp(dev->name,"ECSPI-1")==0){
    read_proc=&read_proc_ecspi1;
    write_proc=&write_proc_ecspi1;
  }else if(strcmp(dev->name,"ECSPI-2")==0){
    read_proc=&read_proc_ecspi2;
    write_proc=&write_proc_ecspi2;
  }else{
    read_proc=&read_proc_cspi;
    write_proc=&write_proc_cspi;
  }
  while(names[i]!=NULL){
    if(register_proc(names[i],read_proc,write_proc,dev->proc_dir,names[i])){
      debugPrintFL(50 ,DRIVER,"registerd %s for %s\n",names[i],dev->name);
    }
    i++;
  }
  return res;
}
int delete_spi_regs (spi_slave_device *dev ,char *names[])
{
  int res=0,i=0;
  if (dev ==NULL || dev->proc_dir==NULL || names==NULL){
    res=EINVAL;
    printk( KERN_WARNING "%s:NULL spi_device passed\n",__FUNCTION__);
    return res;
  }
  while(names[i]!=NULL){
    unregister_proc(names[i],dev->proc_dir);
    i++;
  }
  return res;
}
char * cspi_cloks_names[]={
  "spi_imx.0",
  "spi_imx.1",
  "spi_imx.2",
  NULL
};
static unsigned int mx51_ecspi_clkdiv(unsigned int fin, unsigned int fspi)
{
        /*
         * there are two 4-bit dividers, the pre-divider divides by
         * $pre, the post-divider by 2^$post
         */
        unsigned int pre, post;

        if (unlikely(fspi > fin))
                return 0;

        post = fls(fin) - fls(fspi);
        if (fin > fspi << post)
                post++;

        /* now we have: (fin <= fspi << post) with post being minimal */

        post = max(4U, post) - 4;
        if (unlikely(post > 0xf)) {
                printkE(DRIVER," cannot set clock freq: %u (base freq: %u)\n",
                                 fspi, fin);
                return 0xff;
        }

        pre = DIV_ROUND_UP(fin, fspi << post) - 1;

        debugPrintFL(50,DRIVER,"with clock: %u,configured spi speed: %u, post: %u, pre: %u\n",
                        fin, fspi, post, pre);
        return (pre << MX51_ECSPI_CTRL_PREDIV_OFFSET) |
                (post << MX51_ECSPI_CTRL_POSTDIV_OFFSET);
}
static int  mx51_ecspi_config(spi_slave_device *dev)
{
        u32 ctrl = MX51_ECSPI_CTRL_ENABLE, cfg = 0;

        /*
         * The hardware seems to have a race condition when changing modes. The
         * current assumption is that the selection of the channel arrives
         * earlier in the hardware than the mode bits when they are written at
         * the same time.
         * 
         */
        if(dev->master){
        ctrl |= MX51_ECSPI_CTRL_MODE_MASK;
        }
        /* set clock speed */
        ctrl |= mx51_ecspi_clkdiv(dev->spi_clk, dev->speed_hz);

        /* set chip select to use */
        // Emanuele : Chip select not used
        // ctrl |= MX51_ECSPI_CTRL_CS(dev->cs);


        //configure the burst length in this context the single burst is
        //consideret as a "word" and with this setting the number of byte
        //are considered
        ctrl |= (dev->bits_per_word -1) << MX51_ECSPI_CTRL_BL_OFFSET;

        //control when the spi burst is detected and RXBUFFER advanced
        if(dev->detect_onCS){
        cfg |= MX51_ECSPI_CONFIG_SBBCTRL(0);
        }
        /* SBBCTRL control when a spi burst is detected as 
         * ended and RXDATA FIFO advanced:
         * 1 in case this bit is set , the burst is determined
         * by the chip select toggle
         * 0 in case this bit is cleared the burst is detected by burst length*/

        if (dev->mode & SPI_CPHA){
                cfg |= MX51_ECSPI_CONFIG_SCLKPHA(0);
                //cfg |= MX51_ECSPI_CONFIG_SCLKPHA(1);
                //cfg |= MX51_ECSPI_CONFIG_SCLKPHA(2);
                //cfg |= MX51_ECSPI_CONFIG_SCLKPHA(3);
        }
        if (dev->mode & SPI_CPOL){
                cfg |= MX51_ECSPI_CONFIG_SCLKPOL(0);
                //cfg |= MX51_ECSPI_CONFIG_SCLKPOL(1);
                //cfg |= MX51_ECSPI_CONFIG_SCLKPOL(2);
                //cfg |= MX51_ECSPI_CONFIG_SCLKPOL(3);
        }
        /*  Polarity of the CS signal not used  */
        if (dev->mode & SPI_CS_HIGH)
                cfg |= MX51_ECSPI_CONFIG_SSBPOL(0);

        /* configure if the data line stay high=0 or low=1 when in idle */
        if (0){
                cfg |= MX51_ECSPI_CONFIG_DATA_IDLE(0);
                //cfg |= MX51_ECSPI_CONFIG_DATA_IDLE(1);
                //cfg |= MX51_ECSPI_CONFIG_DATA_IDLE(2);
                //cfg |= MX51_ECSPI_CONFIG_DATA_IDLE(3);
        }
        write_spi_reg(N_ECSPIX_CONREG,ctrl,dev);
        write_spi_reg(N_ECSPIX_CONFIGREG,cfg,dev);
        //writel(ctrl, spi_imx->base + _ECSPI_CTRL);
        //writel(cfg, spi_imx->base + MX51_ECSPI_CONFIG);

        return 0;
}
 
/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  init_dd
 *  Description:  
 * =====================================================================================
 */
static int __init init_dd ( void )
{
  int nSpi = sizeof(spi_devices)/sizeof(spi_devices[0]);
  int i=0,ret;
  bool clock_tied=false;
  debugPrintFL(50 ,DRIVER,"has %d spi ports\n",nSpi);
  init_timer(&fake_timer);
  spi_parent = proc_mkdir(PROC_NAME,NULL);
  if(spi_parent==NULL){
    printk(KERN_WARNING "%s failed to create proc dir\n",__FUNCTION__);
  }
  /* configure the IOMUXC for both ecspi2 and ecspi1 */
  ret = mxc_iomux_v3_setup_multiple_pads(spi2_pads,
      ARRAY_SIZE(spi2_pads));
  if (ret){
    printkE(DRIVER,"Failed to initialize IOMUX for ecspi2\n");
    goto fail;
  }
  ret = mxc_iomux_v3_setup_multiple_pads(spi1_pads,
      ARRAY_SIZE(spi1_pads));
  if (ret){
    printkE(DRIVER,"Failed to initialize IOMUX for ecspi1\n");
    goto fail;
  }
/* request resources */
  for(i=0;i<nSpi;i++){
    ret=0;
    gpio_request(spi_devices[i].gpio_cs, DRIVER);

#ifndef RAW
    if (!request_mem_region(spi_devices[i].regs_addr, XCSPI_SIZE, spi_devices[i].name)) {
      printk(KERN_WARNING "request_mem_region 0x%x for %s failed\n",spi_devices[i].regs_addr,spi_devices[i].name);
      ret = -EBUSY;
      goto fail;
    }
    debugPrintFL(50 ,DRIVER,"0x%x obtained\n",
        spi_devices[i].regs_addr);
    spi_devices[i].initStatus|=MEM_REQUESTED;
    spi_devices[i].base = ioremap(spi_devices[i].regs_addr, XCSPI_SIZE);
    if (!spi_devices[i].base) {
      printk(KERN_WARNING"ioremap 0x%x for %s failed\n",
          spi_devices[i].regs_addr,spi_devices[i].name);
      ret = -EINVAL;
      goto fail;
    }
    debugPrintFL(50 ,DRIVER,"0x%x mapped to 0x%p\n",
        spi_devices[i].regs_addr,
        spi_devices[i].base);
    spi_devices[i].initStatus|=IOREMAPPED;
#endif
    ret = request_irq(spi_devices[i].irq, spi_slave_isr, 0, DRIVER, &spi_devices[i]);
    if (ret) {
      printk(KERN_WARNING "can't get irq%d: %d\n", spi_devices[i].irq, ret);
      goto fail;
    }else{
      debugPrintFL(50 ,DRIVER,"got irq %d\n",
          spi_devices[i].irq); 
      spi_devices[i].initStatus|=IRQ_ASSIGNED;
    }
    if(spi_create_proc_entry(&spi_devices[i],spi_parent)){
      printk(KERN_WARNING"failed to create proc entries\n");
        goto fail;
    }
    spi_devices[i].initStatus|=PROC_CREATED;
    if(cspi_cloks_names[i]){
      spi_devices[i].clk = clk_get_sys(cspi_cloks_names[i], NULL);
      if (IS_ERR(spi_devices[i].clk)) {
        printkE(DRIVER, "unable to get clock for %s\n",spi_devices[i].name);
      }else{
        /*  Emanuele add a dev_printk */
        clk_enable(spi_devices[i].clk);
        debugPrintFL(50,DRIVER,"Enabled clock for %s\n",spi_devices[i].name);
retryClock:
        spi_devices[i].spi_clk = clk_get_rate(spi_devices[i].clk);
        debugPrintFL(50,DRIVER,"default clock rate for %s is %ld\n",spi_devices[i].name, spi_devices[i].spi_clk);
        if(spi_devices[i].spi_clk%spi_devices[i].speed_hz){
          if(clock_tied==false){
            debugPrintFL(50,DRIVER,"The clock can not be used for selected speed \nTrying reconfiguring");
            if(clk_set_rate(spi_devices[i].clk,spi_devices[i].speed_hz)){
              printkE(DRIVER, "unable to set clock at %d\n",spi_devices[i].speed_hz);
              if(clk_set_rate(spi_devices[i].clk,spi_devices[i].spi_clk*3)){
                printkE(DRIVER, "unable to set clock at %ld\n",spi_devices[i].spi_clk*3);
              }
            }
            clock_tied=true;
            goto retryClock;
          }
        }
      }
    }
    if(spi_devices[i].type==ECSPI){
      mx51_ecspi_config(&spi_devices[i]);
      spi_devices[i].spi_clk = clk_get_rate(spi_devices[i].clk);
      debugPrintFL(50,DRIVER,"default clock rate for %s is %ld\n",spi_devices[i].name, spi_devices[i].spi_clk);
    }
    set_chipSelect(&spi_devices[i]);
  }

  /* register chardev */
  if(char_init()){
    printkE(DRIVER,"Failed to register chardev for spi\n");
    goto fail;
  }
  return ret;
fail:
  while(i>=0){
    deinit_device(&spi_devices[i--]);
  }
  if(spi_parent){
  unregister_proc(PROC_NAME,NULL);
  }
  return ret;
}		/* -----  end of function init_dd  ----- */


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  deinit_device
 *  Description:  
 * =====================================================================================
 */
void deinit_device (spi_slave_device *dev)
{
  if(dev==NULL){
    printk(KERN_WARNING "%s: NULL pointer to dev passed\n",__FUNCTION__);
    return ;
  }
  debugPrintFL(50 ,DRIVER,"%s initStatus = 0x%x\n",dev->name,dev->initStatus);
  if(dev->initStatus&PROC_CREATED){
    debugPrintFL(50 ,DRIVER,"%s free procRegisters\n",dev->name);
    if(spi_delete_proc_entry(dev,spi_parent)){
      printk(KERN_WARNING"failed to remove proc entries\n");
    }

    dev->initStatus&=(~PROC_CREATED);
  }
  if(dev->initStatus&IRQ_ASSIGNED){
    /* TODO: disable irq */
    debugPrintFL(50 ,DRIVER,"%s released irq %d\n",dev->name,dev->irq);
    free_irq(dev->irq, dev);
    dev->initStatus&=(~IRQ_ASSIGNED);
  }
  if(dev->initStatus&IOREMAPPED){
    if(dev->base){
      iounmap(dev->base);
      debugPrintFL(50 ,DRIVER,"%s umapped mem\n",dev->name);
    }else{
      printk(KERN_WARNING "%s: error tring to unmap NULL\n",__FUNCTION__);
    }
    dev->base=0;
    dev->initStatus&=(~IOREMAPPED);
  }
  if(dev->initStatus&MEM_REQUESTED){
    release_mem_region(dev->regs_addr,XCSPI_SIZE);
    debugPrintFL(50 ,DRIVER,"%s released mem region\n",dev->name);
    dev->initStatus&=(~MEM_REQUESTED);
  }
  gpio_free(dev->gpio_cs);
  debugPrintFL(50 ,DRIVER,"%s initStatus = 0x%x\n",dev->name,dev->initStatus);
}		/* -----  end of function deinit_device  ----- */
char *ecspi_regs_name[]={
"ECSPIX_RXDATA",
"ECSPIX_TXDATA",
"ECSPIX_CONREG",
"ECSPIX_CONFIGREG",
"ECSPIX_INTREG",
"ECSPIX_DMAREG",
"ECSPIX_STATUS",
"ECSPIX_PERIODREG",
"ECSPIX_TESTREG",
"ECSPIX_MSGSDATA",
NULL,
};
char *cspi_regs_name[]={
"CSPI_RXDATA",
"CSPI_TXDATA",
"CSPI_CONREG",
"CSPI_INTREG",
"CSPI_DMAREG",
"CSPI_STATUS",
"CSPI_PERIODREG",
"CSPI_TESTREG",
NULL,
};
/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  unregister_proc
 *  Description:  
 * =====================================================================================
 */
void unregister_proc(const char* name,struct proc_dir_entry *parent)
{
    debugPrintFL(50 ,DRIVER,"Removing proc %s\n",name);
    remove_proc_entry(name,parent);

}		/* -----  end of function unregister_proc  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  register_proc
 *  Description:  
 * =====================================================================================
 */
struct proc_dir_entry * register_proc (const char* name,read_proc_t *read_proc,write_proc_t *write_proc,struct proc_dir_entry *parent,void *data )
{
  /*  allocate a new proc_dire_entry and check the success */
  struct proc_dir_entry * readWrite_proc_dir = create_proc_entry(name,
      (read_proc==NULL)?S_IRUGO:0x0|(write_proc==NULL)?S_IWUGO:0x0,parent);
  if(readWrite_proc_dir==NULL)
  {
    printk (KERN_WARNING "readWriteProc: error creating proc_dir\n");
    return NULL;
  }
  /* initialize the structure with the correct fnc */
  readWrite_proc_dir->read_proc=read_proc;
  readWrite_proc_dir->write_proc=write_proc;
  readWrite_proc_dir->data=data;
  return 0;
}		/* -----  end of function register_proc  ----- */
module_init(init_dd);

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  exit_dd
 *  Description:  
 * =====================================================================================
 */
static void __exit exit_dd ( void )
{
  int nSpi = sizeof(spi_devices)/sizeof(spi_devices[0]);
  int i=0;
  for(i=0;i<nSpi;i++){
    deinit_device(&spi_devices[i]);
  }
  if(spi_parent){
    unregister_proc(PROC_NAME,NULL);
  }
  debugPrintF(DRIVER,"stop reading from spi\n");
  run_timer=false;
  del_timer_sync(&fake_timer);
  fake_count=0;

  char_exit();
  free_vmem();
  debugPrintFL(50 ,DRIVER,"Exiting\n");
}		/* -----  end of function exit_dd  ----- */
module_exit(exit_dd);


#if 0
/*![
\n***************************************************************************
\n Name: uart_read_ring                                          Type: PUBLIC
\n
\n Abstract:
\n   read a character stream from serial input buffer
\n
\n Warning:
\n  NOT THREAD/INTERRUPT SAFE.
\n
\n Interfaces:
\n
\n return:  TRUE    character available
\n          FALSE   no char in buffer
\n***************************************************************************
]*/
static
BOOL uart_read_ring(RING_BUF_T* p, BYTE* pData)
{
        if((NULL== p)||(NULL==pData)) return(FALSE);

    if (p->hd.r==p->hd.w ){return(FALSE);}

        *pData = p->buf[p->hd.r];       /* Get data */

        p->hd.r++;
    if (p->hd.r >= p->hd.size)
        {
                p->hd.r=0;
        }
    return(TRUE);
}

/*![
\n***************************************************************************
\n Name: uart_write_ring                                                         Type:   LOCAL
\n
\n Abstract:
\n   write a character to serial ring buffer
\n
\n Warning:
\n  NOT THREAD/INTERRUPT SAFE.
\n
\n Interfaces:
\n
\n return:  TRUE    character written
\n          FALSE   Ring buffer is FULL: OVERRUN
\n***************************************************************************
]*/
static
BOOL uart_write_ring(RING_BUF_T* p, BYTE data)
{
    WORD        idx;

        if(NULL==p) return(FALSE);

        idx= p->hd.w+1;
    if (idx >= p->hd.size) idx=0;
    if ( idx != p->hd.r)
    {
                p->buf[p->hd.w]= data;
                p->hd.w=idx;
                return(TRUE);
    }
        return(FALSE);
}


#endif 

//producer
int addToBuffer(char newValue)
{
  unsigned int w= (*writerIndex)+1;
  //debugPrintFL(50 ,DRIVER,"index at %d value 0x%x\n",*writerIndex,newValue);
  if(w>=VMEMSIZE){
    w=WRITE_IDX_START;
  }
  if(w != *readerIndex){
    vmalloc_area[*writerIndex]=newValue;
    *writerIndex=w;
    return 0;
  }
  return -ENOSPC;
}    
//consumer
int readBuffer(char* pData)
{
  if(NULL==pData){
    debugPrintFL(50,DRIVER,"Invalid pointer pData\n");
    return -EINVAL;
  }
  if (*readerIndex == *writerIndex ){
    debugPrintFL(50,DRIVER,"no data available\n");
    return -ENOSPC;
  }

  *pData = vmalloc_area[*readerIndex];       /* Get data */
  *readerIndex=(*readerIndex)+1;
  if (*readerIndex >=VMEMSIZE )
  {
    *readerIndex=DATA_IDX_START;
  }
  return 0;
}
