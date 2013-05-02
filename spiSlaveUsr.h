#ifndef __SPISLAVE_USR__
#define __SPISLAVE_USR__
#include <linux/ioctl.h>
// length of the two memory areas
//#define VMEMSIZE (1024*1024*510)
#define VMEMSIZE (4096)
#define WRITE_IDX_START 0
#define READ_IDX_START 5
#define DATA_IDX_START 9
// ioctl part 
#define SPISLAVE_MAGIC 'z'

typedef struct _YSPISLAVE_SETACQ{
unsigned char acq_ena;
int res;
}YSPISLAVE_SETACQ;

typedef struct _YSPISLAVE_SETREADERP{
unsigned int readerPtr;
int res;
}YSPISLAVE_SETREADERP;

//Error Code
// EIOCTL_ACQVAL only 0 or 1 can be specified in acq_ena
#define EIOCTL_ACQVAL 1

// EIOCTL_WAHEAD actuan reader index <= writer index , trying to set
// reader index ahead writer.
#define EIOCTL_WAHEAD 2
// EIOCTL_OUTSIDEB trying to set reader outside array boundary limits
#define EIOCTL_OUTSIDEB 3
// EIOCTL_RBACKWARD trying to set reader bacward and before write
#define EIOCTL_RBACWARD 4
//* set the acquisition status */
#define C_SPISLAVE_IOSETACQ _IOWR(SPISLAVE_MAGIC,1,YSPISLAVE_SETACQ)
/* set the readerPtr */
#define C_SPISLAVE_IOSETREADEP _IOWR(SPISLAVE_MAGIC,2,YSPISLAVE_SETREADERP)

#endif //__SPISLAVE_USR__
