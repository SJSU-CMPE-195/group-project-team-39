#include "iSV57T.hpp"

iSV57T::iSV57T(const char *p_gpio_scope, unsigned p_dir_line,
               unsigned p_pul_line, uint16_t p_pulse_per_rev)
    : m_pulse_per_rev(p_pulse_per_rev) {

  // Initialize the GPIO scope
  m_chip = gpiod_chip_open(p_gpio_scope);
  if (!m_chip)
    throw std::runtime_error(std::string("Failed to open ") + p_gpio_scope);

  // Initialize the GPIO pins for the DIR and PUL pins
  m_dir_line = gpiod_chip_get_line(m_chip, p_dir_line);
  if (!m_dir_line) {
    gpiod_chip_close(m_chip);
    throw std::runtime_error("Failed to get line " +
                             std::to_string(p_dir_line));
  }

  m_pul_line = gpiod_chip_get_line(m_chip, p_pul_line);
  if (!m_pul_line) {
    gpiod_chip_close(m_chip);
    throw std::runtime_error("Failed to get line " +
                             std::to_string(p_pul_line));
  }

  // Set the GPIO pin values to default LOW
  if (gpiod_line_request_output(m_pul_line, "motor-pul", 0) < 0)
    throw std::runtime_error(
        "Failed to request PUL output during motor initialization.");
  if (gpiod_line_request_output(m_dir_line, "motor-dir", 0) < 0)
    throw std::runtime_error(
        "Failed to request DIR output during motor initialization.");

  if (gpiod_line_set_value(m_pul_line, 0) < 0)
    throw std::runtime_error(
        "Failed to set PUL pin to LOW during motor initializtion.");

  if (gpiod_line_set_value(m_dir_line, 0) < 0)
    throw std::runtime_error(
        "Failed to set PUL pin to LOW during motor initializtion.");
}

void iSV57T::rotate(uint8_t p_direction, float p_degree) {
  if (gpiod_line_set_value(m_dir_line, p_direction) < 0)
    throw std::runtime_error(
        "Failed to set PUL pin to LOW during motor rotation.");

  const long pulses =
      static_cast<long>(std::llround((p_degree / 360.0) * m_pulse_per_rev));

  const double steps_per_sec =
      (target_rpm * static_cast<double>(m_pulse_per_rev)) / 60.0;
  if (steps_per_sec <= 0.0)
    throw std::runtime_error("Invalid steps_per_sec computed.");

  uint32_t period_us = static_cast<uint32_t>(1'000'000.0 / steps_per_sec);

  if (period_us <= pulse_high_us + 1) {
    period_us = pulse_high_us + 2;
  }

  const uint32_t low_us = period_us - pulse_high_us;

  for (long i = 0; i < pulses; ++i) {
    if (gpiod_line_set_value(m_pul_line, 1) < 0)
      throw std::runtime_error("Failed to set PUL pin HIGH.");

    std::this_thread::sleep_for(std::chrono::microseconds(
        static_cast<long long>(std::llround(pulse_high_us)))); // Delaying

    if (gpiod_line_set_value(m_pul_line, 0) < 0)
      throw std::runtime_error("Failed to set PUL pin LOW.");
    std::this_thread::sleep_for(std::chrono::microseconds(
        static_cast<long long>(std::llround(low_us)))); // Delaying
  }
}

void iSV57T::set_pulse_high_us(float p_pulse_high_us) {
  if (p_pulse_high_us >= 2.5 && p_pulse_high_us <= 5.0)
    pulse_high_us = p_pulse_high_us;
  else
    throw std::runtime_error("Failed to set pulse high value due to being out "
                             "of the range of 2.5 to 5.0 us. "
                             "Current pulse_high_us value is " +
                             std::to_string(pulse_high_us) + " us.");
}

void iSV57T::set_target_rpm(float p_target_rpm) {
  if (p_target_rpm >= 100 && p_target_rpm <= 3000)
    target_rpm = p_target_rpm;
  else
    throw std::runtime_error("Failed to set new target RPM due to being out "
                             "of the range of 100 to 3000 RPM. "
                             "Current target_rpm value is " +
                             std::to_string(target_rpm) + " RPM.");
}