/*
 * Pin mux configuration — Apalis iMX8QM M4 Core 0 Gantry Firmware
 *
 * Configures LSIO_GPIO0 pins 8, 9, 12, 13 as GPIO via SCU pad API.
 * (Linux gpiochip2 = LSIO_GPIO0, base 0x5D080000)
 *   LSIO_GPIO0[8]   → Ixora X27 pin 13  (Motor 1 DIR)
 *   LSIO_GPIO0[9]   → Ixora X27 pin 14  (Motor 1 PUL)
 *   LSIO_GPIO0[12]  → Ixora X27 pin 15  (Motor 2 DIR)
 *   LSIO_GPIO0[13]  → Ixora X27 pin 16  (Motor 2 PUL)
 */

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

#include "board.h"
#include "main/imx8qm_pads.h"
#include "svc/pad/pad_api.h"

#if defined(__cplusplus)
extern "C" {
#endif

void BOARD_InitBootPins(void);
void BOARD_InitGpioPins(sc_ipc_t ipc);

#if defined(__cplusplus)
}
#endif

#endif /* _PIN_MUX_H_ */
