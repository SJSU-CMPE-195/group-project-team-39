/*
 * Clock configuration — Apalis iMX8QM Cortex-M4F Core 0
 */

#ifndef _CLOCK_CONFIG_H_
#define _CLOCK_CONFIG_H_

#if defined(__cplusplus)
extern "C" {
#endif

void BOARD_BootClockRUN(void);
void BOARD_InitBootClocks(void);

#if defined(__cplusplus)
}
#endif

#endif /* _CLOCK_CONFIG_H_ */
