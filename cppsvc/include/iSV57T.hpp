#pragma once

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <gpiod.h>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>

/**
 * @brief The driver for the iSV57T BLDC to function on the current setup
 */
class iSV57T {
public:
  enum motor_direction : uint8_t { CCW = 0, CW = 1 };

  /**
   * @brief Constructs a new iSV57T object.
   *
   * @param p_chip refers to the GPIO controller that handlers all the
   GPIO pins associated with it. Ensure that the motor connection uses pins on
   the same scope.
   * @param p_dir_line refers to the line offset for the GPIO pin connected to
   the BLDC's direction pin.
   * @param p_pul_line refers to the line offset for the GPIO pin connected to
   the BLDC's pulse pin.
   * @param p_pulse_per_rev refers to the pulse/rev set on the BLDC. The ranges
   for the current BLDC are from 1600, 2000, 3200, 4000, 5000, 6400, 8000. Note,
   the higher you go, the finer the movements are but at the cost of rotation
   speed. The lower you go, the faster the motor can rotate but at the cost of a
   coarse movement.
   */
  iSV57T(gpiod_chip *p_chip, unsigned p_dir_line, unsigned p_pul_line,
         uint16_t p_pulse_per_rev);

  /**
   * @brief The destructor to close the GPIO lines
   */
  ~iSV57T();

  /**
   * @brief Function to control the motor's rotation.
   *
   * @param p_direction sets the direction the motor will rotate; CCW = 0 and CW
   * = 1.
   * @param p_degree refers to the total degrees the motor should rotate.
   */
  void rotate(uint8_t p_direction, float p_degree);

  /**
   * @brief Function to set the target RPM to a desired value.
   *
   * @param p_target_rpm refers to the new target RPM that the user wants to
   * set. There is a limit up to 3000 RPM.
   */
  void set_target_rpm(float p_target_rpm);

  /**
   * @brief Function to set the target RPM to a desired value.
   *
   * @param p_pulse_high_us referes to the new pulse_high_us valuye. According
   * to the datasheet, the function will limit to only a range of 2.5 us to 5
   * us.
   */
  void set_pulse_high_us(float p_pulse_high_us);

private:
  gpiod_line *m_dir_line;
  gpiod_line *m_pul_line;

  uint16_t m_pulse_per_rev; // Pulse per Rev for the set on the BLDC

  float target_rpm =
      300.0; // Maximum RPM is up to 3000. Play around with smaller settings.
  float pulse_high_us =
      3; // Datasheet mentions pulse width is 2.5 us
         // and direction signal must be valid at
         // least 5 us before the pulse signal. Range is 2.5 us to 5 us.
};