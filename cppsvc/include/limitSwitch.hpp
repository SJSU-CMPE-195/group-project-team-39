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
 * @brief The driver for the switches to function on the current setup
 */
class limitSwitch {
public:
  /**
   * @brief Constructs a new limit switch object.
   *
   * @param p_chip refers to the GPIO controller that handlers all the
   GPIO pins associated with it. Ensure that the motor connection uses pins on
   the same scope.
   * @param p_gpio_line refers to the line offest for the GPIO pin connected to
   the switch.
   */
  limitSwitch(gpiod_chip *p_chip, unsigned p_gpio_line);

  /**
   * @brief The destructor to free the GPIO lines.
   */
  ~limitSwitch();

  /**
   * @brief Function to read the state of the switch.
   */
  uint8_t read();

private:
  gpiod_line *m_line;
};