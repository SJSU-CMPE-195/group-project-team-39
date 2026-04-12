/*
 * Clock configuration — Apalis iMX8QM Cortex-M4F Core 0
 *
 * The clock tree is owned by the SCU firmware.  The M4 subsystem
 * clock is configured before the M4 firmware starts.
 */

#include "fsl_common.h"
#include "clock_config.h"

extern uint32_t SystemCoreClock;

void BOARD_BootClockRUN(void)
{
    SystemCoreClockUpdate();
}

void BOARD_InitBootClocks(void)
{
    BOARD_BootClockRUN();
}
