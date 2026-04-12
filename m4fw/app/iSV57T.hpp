#pragma once

extern "C" {
#include <stdint.h>
#include "fsl_device_registers.h"
#include "FreeRTOS.h"
}

#include "delay_us.h"

/**
 * @brief Driver for the iSV57T BLDC motor on bare-metal FreeRTOS (Cortex-M4F).
 *
 * Uses the M4's own RGPIO peripheral (CM4_0__RGPIO / CM4_1__RGPIO) directly.
 * Microsecond timing uses the ARM DWT cycle counter (delay_us).
 */
class iSV57T {
public:
  enum motor_direction : uint8_t { CCW = 0, CW = 1 };

  /**
   * @param p_base   RGPIO peripheral base (e.g. CM4_0__RGPIO).
   * @param p_dir_pin  Pin number for the BLDC direction signal.
   * @param p_pul_pin  Pin number for the BLDC pulse signal.
   * @param p_pulse_per_rev  Pulses per revolution configured on the BLDC.
   *        Valid: 1600, 2000, 3200, 4000, 5000, 6400, 8000.
   */
  iSV57T(RGPIO_Type *p_base, uint32_t p_dir_pin, uint32_t p_pul_pin,
         uint16_t p_pulse_per_rev);

  ~iSV57T() = default;

  /**
   * @brief Rotate the motor.
   * @param p_direction  CW or CCW.
   * @param p_degree     Degrees to rotate.
   */
  void rotate(uint8_t p_direction, float p_degree);

  /**
   * @brief Set target RPM (100–3000).
   */
  void set_target_rpm(float p_target_rpm);

  /**
   * @brief Set pulse-high duration in microseconds (2.5–5.0 us).
   */
  void set_pulse_high_us(float p_pulse_high_us);

private:
  RGPIO_Type *m_base;
  uint32_t    m_dir_pin;
  uint32_t    m_pul_pin;

  uint16_t m_pulse_per_rev;

  float target_rpm     = 300.0f;
  float pulse_high_us  = 3.0f;
};
