#include "limitSwitch.hpp"

limitSwitch::limitSwitch(gpiod_chip *p_chip, unsigned p_gpio_line) {
  // Checks if the scope passed in is NULL
  if (!p_chip)
    throw std::runtime_error(
        std::string("Null gpiod_chip passed to limitSwitch"));

  // Initialize the GPIO pins for the DIR and PUL pins
  m_line = gpiod_chip_get_line(p_chip, p_gpio_line);
  if (!m_line) {
    throw std::runtime_error(std::string("Failed to get line ") +
                             std::to_string(p_gpio_line) + " " +
                             strerror(errno));
  }

  // Setting the GPIO as an INPUT
  if (gpiod_line_request_input(m_line, "my-consumer") < 0)
    throw std::runtime_error(std::string("Failed to request GPIO input during "
                                         "limit switch initialization. ") +
                             strerror(errno));
}

limitSwitch::~limitSwitch() {
  if (!m_line)
    throw std::runtime_error(
        std::string("GPIO lines are NULL. Failed to close lines. ") +
        strerror(errno));
  gpiod_line_release(m_line);
}

uint8_t limitSwitch::read() {
  int val = gpiod_line_get_value(m_line);
  if (val < 0)
    throw std::runtime_error(std::string("Failed to read GPIO line: ") +
                             strerror(errno));
  return static_cast<uint8_t>(val);
}