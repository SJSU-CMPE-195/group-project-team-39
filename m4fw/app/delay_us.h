#ifndef DELAY_US_H
#define DELAY_US_H

#include "fsl_common.h"
#include "FreeRTOSConfig.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ARM DWT (Data Watchpoint and Trace) cycle-counter based microsecond delay.
 *
 * At 264 MHz (configCPU_CLOCK_HZ), one cycle = ~3.79 ns, so 264 cycles = 1 us.
 * This gives sub-microsecond precision for motor pulse timing.
 *
 * NOTE: This is a busy-wait — it does NOT yield the CPU to FreeRTOS.
 * Use only for short delays (< ~1 ms). For longer waits, use vTaskDelay().
 */

#define DWT_CONTROL  (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT   (*(volatile uint32_t *)0xE0001004)
#define DWT_LAR      (*(volatile uint32_t *)0xE0001FB0)
#define SCB_DEMCR    (*(volatile uint32_t *)0xE000EDFC)

static inline void delay_us_init(void)
{
    SCB_DEMCR  |= (1UL << 24);   /* TRCENA: enable DWT */
    DWT_LAR     = 0xC5ACCE55;    /* unlock DWT (required on some cores) */
    DWT_CYCCNT  = 0;
    DWT_CONTROL |= 1UL;          /* enable CYCCNT */
}

static inline void delay_us(uint32_t us)
{
    const uint32_t cycles = us * (configCPU_CLOCK_HZ / 1000000UL);
    const uint32_t start  = DWT_CYCCNT;
    while ((DWT_CYCCNT - start) < cycles) {}
}

#ifdef __cplusplus
}
#endif

#endif /* DELAY_US_H */
