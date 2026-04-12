#pragma once

extern "C" {
#include <stdint.h>
#include "fsl_gpio.h"
#include "FreeRTOS.h"
}

/**
 * @brief Driver for limit switches on bare-metal FreeRTOS (Cortex-M4F).
 *
 * Ported from the Linux/libgpiod version. GPIO access uses the NXP fsl_gpio
 * driver (GPIO_PinRead).
 */
class limitSwitch {
public:
  /**
   * @param p_base      GPIO peripheral base (e.g. LSIO__GPIO2).
   * @param p_gpio_pin  Pin number for the switch input.
   */
  limitSwitch(GPIO_Type *p_base, uint32_t p_gpio_pin);

  ~limitSwitch() = default;

  /**
   * @brief Read the current switch state.
   * @return 0 or 1.
   */
  uint8_t read();

private:
  GPIO_Type *m_base;
  uint32_t   m_pin;
};
