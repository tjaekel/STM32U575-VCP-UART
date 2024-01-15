/*
 * SYS_config.c
 *
 *  Created on: Jul 2, 2018
 *      Author: tj925438
 */

#include "SYS_config.h"
#include <string.h>

const tCFGparams defaultCFGparams = {
		.key 			= CFG_KEY_VALID,

		/* QSPI defaults */
		.QSPIdiv		= 32,			// 1: QSPI divider
		.QSPImode		= 0,			// 2: QSPI peripheral: PHA and POL, SPI mode 0..3
		.QSPIaddr		= 4,			// 3: QSPI peripheral: number ADDR bytes
		.QSPIalt		= 3,			// 4: QSPI peripheral: number ALL bytes
		.QSPIturn		= 2,			// 5: QSPI peripheral: clocks for turnaround

			/* debug and other sys config */
		.Debug			= 0,			// 6: debug flags
		.CfgFlags		= 0				// 7: config flags
};

tCFGparams gCFGparams;

void CFG_Read(void)
{
	memcpy(&gCFGparams, &defaultCFGparams, sizeof(tCFGparams));
}

void CFG_Default(void)
{
	memcpy(&gCFGparams, &defaultCFGparams, sizeof(tCFGparams));
}

/*
 * write value to a specific index in the structure
 */
void CFG_Set(unsigned long idx, unsigned long val)
{
	unsigned long *sPtr;

	if (idx >= (sizeof(tCFGparams) / 4))
		return;			/* just 64 unsigned long elements */

	/* handle structure like an array */
	sPtr = (unsigned long *)&gCFGparams;

	sPtr[idx] = val;
}

void CFG_Print(EResultOut out)
{
	/* print SPI config */
	Generic_Send("Sys Config:\r\n", 13, out);
	print_log(out, "[ 1] QSPI div        : %ld\r\n", gCFGparams.QSPIdiv);
	print_log(out, "[ 2] QSPI mode       : %ld\r\n", gCFGparams.QSPImode);
	print_log(out, "[ 3] QSPI addr       : %ld\r\n", gCFGparams.QSPIaddr);
	print_log(out, "[ 4] QSPI alt        : %ld\r\n", gCFGparams.QSPIalt);
	print_log(out, "[ 5] QSPI turn       : %ld\r\n", gCFGparams.QSPIturn);
	print_log(out, "[ 6] Debug           : %lx\r\n", gCFGparams.Debug);
	print_log(out, "[ 7] CfgFlags        : %lx\r\n", gCFGparams.CfgFlags);
}

void CFG_Print_hex(EResultOut out)
{
	int i;
	unsigned long *ulPtr = (unsigned long *)&gCFGparams;

	for (i = 0; i < (sizeof(tCFGparams) / 4); i++)
	{
		print_log(out, (const char *)"%2d : 0x%08lx\r\n", i, *ulPtr++);
	}
}

void SYSINFO_print(EResultOut out)
{
	print_log(out, "%s", VERSION_STRING);
}
