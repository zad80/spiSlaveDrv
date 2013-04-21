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
struct proc_dir_entry *spi_parent =NULL;

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
"ECSPIX_MSGSATA",
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
 *         Name:  spi_delete_proc_entry
 *  Description:  this function create : the directory of the device and all
 *  the proc entry to the register access
 * =====================================================================================
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
 *  the proc entry to the register access
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
  if (dev ==NULL || dev->proc_dir==NULL || names==NULL){
    res=EINVAL;
    printk( KERN_WARNING "%s:NULL spi_device passed\n",__FUNCTION__);
    return res;
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
  spi_parent = proc_mkdir(PROC_NAME,NULL);
  if(spi_parent==NULL){
    printk(KERN_WARNING "%s failed to create proc dir\n",__FUNCTION__);
  }
  /* request resources */
  for(i=0;i<nSpi;i++){
    ret=0;
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
  debugPrintFL(50 ,DRIVER,"%s initStatus = 0x%x\n",dev->name,dev->initStatus);
}		/* -----  end of function deinit_device  ----- */
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
void unregister_proc(const char* name,struct proc_dir_entry *parent)
{
  if(parent)
  {
    debugPrintFL(50 ,DRIVER,"Removing proc %s\n",name);
    remove_proc_entry(name,parent);
  }

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
  debugPrintFL(50 ,DRIVER,"Exiting\n");
}		/* -----  end of function exit_dd  ----- */
module_exit(exit_dd);
