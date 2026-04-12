/*
 * Pin mux configuration — Apalis iMX8QM M4 Core 0 Gantry Firmware
 *
 * Linux enumerates gpiochip2 = LSIO_GPIO0 (base 0x5D080000).
 * Configures four M4-subsystem pads to LSIO_GPIO0 (ALT3) via SCU IPC:
 *
 *   SoC pad              Alt func  GPIO              Ixora X27  Function
 *   ──────────────────   ────────  ────────────────  ─────────  ────────
 *   SC_P_M40_GPIO0_00    ALT3      LSIO_GPIO0[8]    pin 13     Motor 1 DIR
 *   SC_P_M40_GPIO0_01    ALT3      LSIO_GPIO0[9]    pin 14     Motor 1 PUL
 *   SC_P_M41_GPIO0_00    ALT3      LSIO_GPIO0[12]   pin 15     Motor 2 DIR
 *   SC_P_M41_GPIO0_01    ALT3      LSIO_GPIO0[13]   pin 16     Motor 2 PUL
 */

#include "pin_mux.h"
#include "fsl_common.h"
#include "main/imx8qm_pads.h"
#include "svc/pad/pad_api.h"

void BOARD_InitBootPins(void)
{
}

void BOARD_InitGpioPins(sc_ipc_t ipc)
{
    sc_err_t err;

    /* Power on LSIO_GPIO0 */
    err = sc_pm_set_resource_power_mode(ipc, SC_R_GPIO_0, SC_PM_PW_MODE_ON);
    if (err != SC_ERR_NONE)
    {
        /* GPIO0 may already be powered — non-fatal */
    }

    /* Motor 1 DIR — SC_P_M40_GPIO0_00 → LSIO_GPIO0_IO08 (ALT3) */
    err = sc_pad_set_mux(ipc, SC_P_M40_GPIO0_00, 3U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF);
    (void)err;

    /* Motor 1 PUL — SC_P_M40_GPIO0_01 → LSIO_GPIO0_IO09 (ALT3) */
    err = sc_pad_set_mux(ipc, SC_P_M40_GPIO0_01, 3U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF);
    (void)err;

    /* Motor 2 DIR — SC_P_M41_GPIO0_00 → LSIO_GPIO0_IO12 (ALT3) */
    err = sc_pad_set_mux(ipc, SC_P_M41_GPIO0_00, 3U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF);
    (void)err;

    /* Motor 2 PUL — SC_P_M41_GPIO0_01 → LSIO_GPIO0_IO13 (ALT3) */
    err = sc_pad_set_mux(ipc, SC_P_M41_GPIO0_01, 3U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF);
    (void)err;
}
