/*
 * Board configuration for Apalis iMX8QM — Cortex-M4F Core 0
 * Adapted from the MCUXpresso SDK mekmimx8qm board files.
 */

#ifndef _BOARD_H_
#define _BOARD_H_

#include "clock_config.h"
#include "fsl_gpio.h"

/* SCFW includes */
#include "main/rpc.h"
#include "svc/pm/pm_api.h"
#include "svc/irq/irq_api.h"
#include "svc/misc/misc_api.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BOARD_NAME "Apalis-iMX8QM"

/* M4 core 0 debug UART */
#define BOARD_DEBUG_UART_TYPE     kSerialPort_Uart
#define BOARD_DEBUG_UART_BAUDRATE 115200U
#define BOARD_DEBUG_UART_BASEADDR (uint32_t) CM4_0__LPUART
#define BOARD_DEBUG_UART_INSTANCE 0U
#define BOARD_DEBUG_UART_SC_RSRC  SC_R_M4_0_UART
#define BOARD_DEBUG_UART_CLKSRC   kCLOCK_M4_0_Lpuart
#define BOARD_UART_IRQ            M4_0_LPUART_IRQn
#define BOARD_M4_CPU_RSRC         SC_R_M4_0_PID0

#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
 * API
 ******************************************************************************/
sc_ipc_t BOARD_InitRpc(void);
sc_ipc_t BOARD_GetRpcHandle(void);
void BOARD_InitDebugConsole(void);
void BOARD_InitMemory(void);
void BOARD_InitBootPins(void);
void BOARD_InitBootClocks(void);

#if defined(__cplusplus)
}
#endif

#endif /* _BOARD_H_ */
