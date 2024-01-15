/*
 * MEM_Pool.h
 *
 *  Created on: Feb 12, 2017
 *  Author: Torsten Jaekel
 */

#ifndef MEM_POOL_H_
#define MEM_POOL_H_

#include <stdint.h>

#define MEM_POOL_SEG_SIZE		(4096)		//byte chunk size greater 4K*16bit! - aligned for Cache Line Size!

#define MEM_POOL_SEGMENTS		4
extern uint8_t sMemPoolRAM[MEM_POOL_SEG_SIZE * MEM_POOL_SEGMENTS];
#define SRAM_FREE_START			((unsigned long)&sMemPoolRAM[0])

#define ALLOC_USED		1		//start of allocated entries
#define ALLOC_SUBSEQ	2		//it belongs to the start with ALLOC_USED
#define ALLOC_FREE		0		//free entry

typedef struct MEM_POOL {
	unsigned long	startAddr;
	int				alloc;
} TMem_Pool;

extern void  MEM_PoolInit(void);
extern void *MEM_PoolAlloc(unsigned int n);
extern void  MEM_PoolFree(void *ptr);
unsigned int MEM_PoolWatermark(void);
unsigned int MEM_PoolAvailable(void);
unsigned int MEM_PoolMax(void);

#endif /* MEM_POOL_H_ */
