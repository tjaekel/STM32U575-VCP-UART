/*
 * SYS_config.h
 *
 *  Created on: Jul 2, 2018
 *      Author: tj925438
 */

#ifndef SYS_CONFIG_H_
#define SYS_CONFIG_H_

#include "cmd_dec.h"

#define CFG_KEY_VALID			0xC5ACCE55u

/* debug flags */
#define DBG_VERBOSE				(1 << 0)		//more debug messages - VERBOSE

/* ATT: the size of the struct has to be 256 bytes = 64x long words - we read/write one page on I2C flash */
typedef struct {						// setcfg <idx>
	unsigned long	key;				// 0: valid or not key: CFG_KEY_VALID - DON'T TOUCH
	// QSPI configuration
	unsigned long	QSPIdiv;			// 1: QSPI divider
	unsigned long	QSPImode;			// 2: QSPI peripheral: PHA and POL, SPI mode 0..3
	unsigned long	QSPIaddr;			// 3: QSPI peripheral: number ADDR bytes
	unsigned long	QSPIalt;			// 4: QSPI peripheral: number ALL bytes
	unsigned long	QSPIturn;			// 5: QSPI peripheral: clocks for turnaround

	/* debug and other sys config */
	unsigned long	Debug;				// 6: debug flags
	unsigned long	CfgFlags;			// 7: config flags:
										//

} tCFGparams;

/* TODO: define all the bits, e.g. CfgFlags */

extern tCFGparams gCFGparams;

void CFG_Read(void);
void CFG_Default(void);
void CFG_Set(unsigned long idx, unsigned long val);
void CFG_Print(EResultOut out);
void CFG_Print_hex(EResultOut out);

void SYSINFO_print(EResultOut out);

#endif /* SYS_CONFIG_H_ */
