/*  avoid double include */
#ifndef __RING_BUFFER_H
#define __RING_BUFFER_H

#include "drv_dbg.h"


typedef struct __ring_buffer{
  void *data;
  void *read;
  void *write;
  void *end;
  unsigned int size;
}ring_buffer_t;

/* function declarations */
ring_buffer_t* alloc_ring_buffer(unsigned int size);
void free_ring_buffer(ring_buffer_t * rb);
int write_ring_buffer(ring_buffer_t * rb,char *b,int count);
int read_ring_buffer(ring_buffer_t * rb,char *b,int count);

#endif //ifndef __RING_BUFFER_H
