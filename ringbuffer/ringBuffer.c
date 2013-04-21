/*
 * =====================================================================================
 *
 *       Filename:  ringBuffer.c
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  03/18/2013 04:55:50 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include "ringBuffer.h"
void free_ring_buffer(ring_buffer_t * rb)
{
  if(rb==NULL)
    return;
  if(rb->data != NULL)
  {
    kfree(rb->data);
    rb->data=NULL;
  }
  
  kfree(rb);
  rb=NULL;
  return;
}
#define isEmpty(r,w)  ((r==w)?true:false)
#define isFull(r,w,s,e) ((r==w-1 || (w==e && r==s))?true:false)
#define REAL_WRITE 1
int write_ring_buffer(ring_buffer_t *rb,char *buffer,int count)
{
  int available=0;
  int written=0;
  /* First save the current read poiter
   * in order to avoid update on it while doing other checks
   * */
  void *read=rb->read;
  /*  ERROR CHECK */
redo_check:
  debugPrintF("","tmp read is %p write %p count %lu \n",read,rb->write,count);
  if(count<=0||buffer==NULL || rb==NULL || rb->data==NULL)
  {	
    return -EFAULT;
  }
  /*  check for fullyness */
  if(isFull(read,rb->write,rb->data,rb->end)){
    return -ENOMEM;
  }
  if(rb->write>=read){
    available=(rb->end)-rb->write;
    if(available==0){
      /*  we are at the end of the buffer */
      if(REAL_WRITE)
        memcpy(rb->write,&buffer[written],1);
      debugPrintF("","memcoping 1 and then wrap from buffer %p (%s) to write %p (%s)\n",buffer,buffer,rb->write,rb->write);
      rb->write=rb->data;
      count--;
      written++;
      if(count<=0)
        return written;
      goto redo_check;
    }
  }else{
    available=read-rb->write-1;
  }
  debugPrintF("","available %i\n",available);
  if(count<=available){
    available=count;
  }
  debugPrintF("","new available %i\n",available);
  if(REAL_WRITE)
    memcpy(rb->write,&buffer[written],available);
  debugPrintF("","memcoping from buffer %p (%s) to write %p (%s)\n",buffer,buffer,rb->write,rb->write);
  written+=available;
  count-=available;
  /*update the write pointer  */
  rb->write+=(available);
 debugPrintF("","new write %p\n",rb->write);
  if(count)
    goto redo_check;
  return written;
}
int read_ring_buffer(ring_buffer_t *rb,char *buffer,int count)
{
	int available=0;
        int readed=0;
	/* First save the current write poiter
	 * in order to avoid update on it while doing other checks
	 * */
	void *write=rb->write;
        debugPrintF("","tmp write is %p\n",write);
redo_check:
	/*  ERROR CHECK */
	if(count<=0 ||buffer==NULL || rb==NULL || rb->data==NULL)
	{	
          debugPrintF("","invalid param \n");
		return -EFAULT;
	}
	/*  CHECK */
	if(isEmpty(rb->read,write))
        {
          debugPrintF("","empty buffer \n");
          return readed;
        }
        if(write > rb->read)
        {
          available=write-rb->read ;	
        }
        else
        {
          /*  writer wrap around */
          available=(rb->end)-rb->read;	
        }
        debugPrintF("","available %i\n",available);
        if(available==0 && rb->read==rb->end)
        {
          debugPrintF("","read wrapped\n");
          memcpy(&buffer[readed],rb->read,1);
          rb->read=rb->data;
          /*  read one and wrap */
          count--;
          if(count==0)
            return 1;
          readed++;
          goto redo_check;
        }
	/*  check if the reader wan't less than available */
	if(available>count)
	{
		available=count;
	}
        debugPrintF("","memcpy read %p (%s)to buffer  %p\n",rb->read,rb->read,buffer);
	memcpy(&buffer[readed],rb->read,available);
        debugPrintF("","buffer contains (%s) \n",buffer);
        count -=available;
        readed+=available;
	/*update the read pointer  */
	rb->read+=(available);
        debugPrintF("","new read %p \n",rb->read);
        if(count)
          goto redo_check;
	return readed;
}
ring_buffer_t* alloc_ring_buffer(unsigned int size)
{
  ring_buffer_t *new_rb =NULL;
        debugPrintF("","ring_buffer_t called\n");
  if(size>1){
    new_rb = kzalloc(sizeof(ring_buffer_t),GFP_KERNEL);
    if(new_rb!=NULL){
      new_rb->data= kzalloc(size,GFP_KERNEL);
      if(new_rb->data==NULL){
        kfree(new_rb);
        new_rb=NULL;
      }else{
	new_rb->read=new_rb->write=new_rb->data;
	new_rb->end=new_rb->data+(size-1);
        debugPrintF("","ring_buffer_t allocated data=%p end=%p write=%p read=%p\n",
            new_rb->data , new_rb->end, new_rb->write,new_rb->read);
      }
    }else{
      debugPrintF("","failed to alloc ring_buffer_t\n");
    }
  }
  return new_rb;
}


