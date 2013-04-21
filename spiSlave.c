/*
 * =====================================================================================
 *
 *       Filename:  procReadWrite.c
 *
 *    Description:  implementation of a device driver which create a file under
 *    /proc virtual file system and let the user write and read on that file.
 *    Thing written inside the file are then read back from the same file.
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

#include <linux/uaccess.h>
#include "spiSlave.h"
#include "ringBuffer.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Zad");



/* Constants & MACROES */
#define DRIVER "spiSlave"
#define PROC_NAME "spi"
#define RING_SIZE (10)
/* Global variables used by the DD */
ring_buffer_t* readWrite_buffer=NULL;
struct proc_dir_entry *readWrite_proc_dir=NULL;

spi_slave_device spi_devices[]={
  {
    .regs_addr=ECSPI1_BASE,
    .base=NULL,
    .irq=36,
    .type=ECSPI,
    .name="ECSPI-1",
    .initStatus=0x0,
  },
  {
    .regs_addr=ECSPI2_BASE,
    .base=NULL,
    .irq=37,
    .type=ECSPI,
    .name="ECSPI-2",
    .initStatus=0x0,
  },
  {
    .regs_addr=CSPI_BASE,
    .base=NULL,
    .irq=38,
    .type=CSPI,
    .name="CSPI",
    .initStatus=0x0,
  },
};

/* Function declarations */

static int __init init_dd(void);
static void __exit exit_dd(void);
int read_proc(char *page, char **start, off_t offset,int count, int *eof, void *data);
int write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data);
int register_proc(void);
void unregister_proc(void);
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
                /* No data left to push, but still waiting for rx data,
                 * enable receive data available interrupt.
                 */
                spi_imx->devtype_data->intctrl(
                                spi_imx, MXC_INT_RR);
                return IRQ_HANDLED;
        }

        spi_imx->devtype_data->intctrl(spi_imx, 0);
        complete(&spi_imx->xfer_done);
#endif
        return IRQ_HANDLED;
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
  debugPrintFL(50 ,DRIVER,"has %d spi ports\n",nSpi);

  /* request resources */
  for(i=0;i<nSpi;i++){
    ret=0;
    if (!request_mem_region(spi_devices[i].regs_addr, XCSPI_SIZE, spi_devices[i].name)) {
      printk(KERN_WARNING "request_mem_region 0x%x for %s failed\n",spi_devices[i].regs_addr,spi_devices[i].name);
      ret = -EBUSY;
      goto fail;
    }
    debugPrintFL(50 ,DRIVER,"0x%p obtained\n",
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

    ret = request_irq(spi_devices[i].irq, spi_slave_isr, 0, DRIVER, &spi_devices[i]);
    if (ret) {
      printk(KERN_WARNING "can't get irq%d: %d\n", spi_devices[i].irq, ret);
      goto fail;
    }else{
      debugPrintFL(50 ,DRIVER,"got irq %d\n",
          spi_devices[i].irq); 
      spi_devices[i].initStatus|=IRQ_ASSIGNED;
    }
    /*  TODO: add the proc entry
        if(spi_create_proc_entry()){
        printk(KERN_WARNING"failed to create proc entries\n")
        }
        spi_devices[i].initStatus|=PROC_CREATED;
        */
  }
  return ret;
  fail:
  while(i>=0){
    deinit_device(&spi_devices[i--]);
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
    /* TODO: add the removeProc */
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
     debugPrintFL(50 ,DRIVER,"%s initStatus = 0x%x\n",dev->name,dev->initStatus);
}		/* -----  end of function deinit_device  ----- */
/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  remove_proc_entry
 *  Description:  
 *  return 0 in case of success 
 *  return <0 in case of error
 * =====================================================================================
 */
int spi_remove_proc_entry (void)
{
  unregister_proc();
  free_ring_buffer(readWrite_buffer);
  return 0;
}		/* -----  end of function remove_proc_entry  ----- */
/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  create_proc_entry
 *  Description: 
 *  return 0 in case of success 
 *  return <0 in case of error
 *
 * =====================================================================================
 */
int spi_create_proc_entry (void)
{
  if((readWrite_buffer=alloc_ring_buffer(RING_SIZE))==NULL)
  {
    return -1;
  }
  if(register_proc()){
    free_ring_buffer(readWrite_buffer);
    return -1;
  }


  return 0;
}		/* -----  end of function create_procs_entry  ----- */
/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  write_proc
 *  Description:  
 * =====================================================================================
 */
int write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{

  int res=0;
  void *kernel_buffer = kzalloc(count,GFP_KERNEL);
  if(count>PAGE_SIZE){
    count=PAGE_SIZE;
  }
  if(kernel_buffer==NULL){
    debugPrintFL(50,DRIVER,"failed to allocate temporary buffer\n");
    res = -EFAULT;
    goto error;
  }
  if(copy_from_user(kernel_buffer,buffer,count)){
    debugPrintFL(50,DRIVER,"failed to copy %luB memory from users space\n",count);
    res=-EFAULT;
    goto error;
  }
  res= write_ring_buffer(readWrite_buffer,kernel_buffer,count);
error:
  if(kernel_buffer!=NULL)
    kfree(kernel_buffer);
  return res;
}		/* -----  end of function write_proc  ----- */
/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  read_proc
 *  Description:  
 * =====================================================================================
 */
int read_proc(char *page, char **start, off_t offset,int count, int *eof, void *data)
{
  if(count>PAGE_SIZE){
    count=PAGE_SIZE;
  }
  return read_ring_buffer(readWrite_buffer,page,count);
}		/* -----  end of function read_proc  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  unregister_proc
 *  Description:  
 * =====================================================================================
 */
void unregister_proc ( void)
{
  if(readWrite_proc_dir)
  {
    remove_proc_entry(PROC_NAME,NULL);
  }
  
}		/* -----  end of function unregister_proc  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  register_proc
 *  Description:  
 * =====================================================================================
 */
int register_proc ( void )
{
  /*  allocate a new proc_dire_entry and check the success */
  readWrite_proc_dir = create_proc_entry(PROC_NAME,S_IRUGO|S_IWUGO,NULL);
  if(readWrite_proc_dir==NULL)
  {
    printk (KERN_WARNING "readWriteProc: error creating proc_dir\n");
    return -1;
  }
  /* initialize the structure with the correct fnc */
  readWrite_proc_dir->read_proc=read_proc;
  readWrite_proc_dir->write_proc=write_proc;

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
 debugPrintFL(50 ,DRIVER,"Exiting\n");
}		/* -----  end of function exit_dd  ----- */
module_exit(exit_dd);
