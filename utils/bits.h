#ifndef __UTILS_BITS_H__
#define __UTILS_BITS_H__ 

/*
 *----------------------------------------------------------------------------
 * Miscellaneous defines
 *----------------------------------------------------------------------------
 */
#define BIT_MSK(b)	(1 << (b))
#define BIT0		(BIT_MSK(0))
#define BIT1		(BIT_MSK(1))
#define BIT2		(BIT_MSK(2))
#define BIT3		(BIT_MSK(3))
#define BIT4		(BIT_MSK(4))
#define BIT5		(BIT_MSK(5))
#define BIT6		(BIT_MSK(6))
#define BIT7		(BIT_MSK(7))
#define BIT8		(BIT_MSK(8))
#define BIT9		(BIT_MSK(9))
#define BIT10		(BIT_MSK(10))
#define BIT11		(BIT_MSK(11))
#define BIT12		(BIT_MSK(12))
#define BIT13		(BIT_MSK(13))
#define BIT14		(BIT_MSK(14))
#define BIT15		(BIT_MSK(15))
#define BIT16		(BIT_MSK(16))
#define BIT17		(BIT_MSK(17))
#define BIT18		(BIT_MSK(18))
#define BIT19		(BIT_MSK(19))
#define BIT20		(BIT_MSK(20))
#define BIT21		(BIT_MSK(21))
#define BIT22		(BIT_MSK(22))
#define BIT23		(BIT_MSK(23))
#define BIT24		(BIT_MSK(24))
#define BIT25		(BIT_MSK(25))
#define BIT26		(BIT_MSK(26))
#define BIT27		(BIT_MSK(27))
#define BIT28		(BIT_MSK(28))
#define BIT29		(BIT_MSK(29))
#define BIT30		(BIT_MSK(30))
#define BIT31		(BIT_MSK(31))


/* bit map related macros */
#ifndef setbit
#ifndef NBIT 
#define NBIT    32       /* 32 bits per byte */
#endif /* #ifndef NBIT */
#define setbit(reg, pos)    (((u32 *)reg)[(pos) / NBIT] |= 1 << ((pos) % NBIT))
#define clrbit(reg, pos)    (((u32 *)reg)[(pos) / NBIT] &= ~(1 << ((pos) % NBIT)))
#define isset(reg, pos)     (((const u32 *)reg)[(pos) / NBIT] & (1 << ((pos) % NBIT)))
#define isclr(reg, pos)     ((((const u32 *)reg)[(pos) / NBIT] & (1 << ((pos) % NBIT))) == 0)

#define MAKE_MASK(pos,len) (((0x1 << len) -1) <<pos) //MAKE_MASK(4,2) 0x0000110000
#define SET_BIT2VAL(dst,val,start,end) ( ((val<<start)& MAKE_MASK(start,(end-start+1))) | (dst & ~MAKE_MASK(start,(end-start+1)) ) )
#define GET_BITS(dst,start,end)  ((dst & (MAKE_MASK(start,(end-start+1)))) >>start )

#endif /* setbit */
#define isbitset(a, i)  (((a) & (1 << (i))) != 0)

#endif
