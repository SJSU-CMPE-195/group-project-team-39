/*
 * Board initialization for Apalis iMX8QM — Cortex-M4F Core 0
 * Adapted from the MCUXpresso SDK mekmimx8qm hello_world example.
 */

#include "fsl_common.h"
#include "board.h"
#include "fsl_gpio.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
static sc_ipc_t ipcHandle;

/*******************************************************************************
 * Code
 ******************************************************************************/
sc_ipc_t BOARD_InitRpc(void)
{
    SystemInitScfwIpc();
    ipcHandle = SystemGetScfwIpcHandle();

    if (ipcHandle)
    {
        CLOCK_Init(ipcHandle);
    }

    if (sc_misc_boot_done(ipcHandle, BOARD_M4_CPU_RSRC) != SC_ERR_NONE)
    {
        /* Non-fatal on some boot paths; continue anyway */
    }

    return ipcHandle;
}

sc_ipc_t BOARD_GetRpcHandle(void)
{
    return ipcHandle;
}

void BOARD_InitDebugConsole(void)
{
    /*
     * Debug UART init requires fsl_debug_console and fsl_lpuart drivers.
     * For the GPIO toggle demo these are not needed — stub it out.
     * To enable, extract the LPUART driver from the SDK and uncomment:
     *
     * uint32_t freq = SC_24MHZ;
     * sc_pm_set_resource_power_mode(ipcHandle, BOARD_DEBUG_UART_SC_RSRC, SC_PM_PW_MODE_ON);
     * CLOCK_EnableClockExt(BOARD_DEBUG_UART_CLKSRC, 0);
     * freq = CLOCK_SetIpFreq(BOARD_DEBUG_UART_CLKSRC, freq);
     * DbgConsole_Init(BOARD_DEBUG_UART_INSTANCE, BOARD_DEBUG_UART_BAUDRATE, BOARD_DEBUG_UART_TYPE, freq);
     */
}

void BOARD_InitMemory(void)
{
    /* TCM execution — no MPU reconfiguration needed for this demo */
}
