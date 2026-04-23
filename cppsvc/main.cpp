#include <arpa/inet.h>

#include <csignal>
#include <cstdint>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include "gantry.hpp"
#include "iSV57T.hpp"
#include "include/gantry.hpp"
#include "include/iSV57T.hpp"
#include "limitSwitch.hpp"

// gpio-641 (MXM3_11/GPIO5       ) // Gotta figure out what line those are but
// they belong to chip3 gpio-642 (MXM3_13/GPIO6

// Physical Pin 17 is South North limit switch (y)
// Physical Pin 18 is West East limit switch x)

int main() {
  const char *gpio_chip4_char = "/dev/gpiochip4"; // For the limit switch GPIO
  const char *gpio_chip0_char = "/dev/gpiochip0"; // For the motors GPIO
  // m1
  const unsigned m1_dir_line = 8; // Physical Pin 13
  const unsigned m1_pul_line = 9; // Physical Pin 14

  // m2
  const unsigned m2_dir_line = 12; // Physical Pin 15
  const unsigned m2_pul_line = 13; // Physical Pin 16

  // limit switch
  const unsigned sw1_line = 1; // Physical Pin 17
  const unsigned sw2_line = 2; // Physical Pin 18

  const uint16_t pulse_per_rev = 2000;

  gpiod_chip *chip0 = gpiod_chip_open(gpio_chip0_char);
  if (!chip) {
    fprintf(stderr, "Failed to open %s for motor pins: %s\n", gpio_chip_char,
            strerror(errno));
    return 0;
  }

  gpiod_chip *chip4 = gpiod_chip_open(gpio_chip4_char);
  if (!chip2) {
    fprintf(stderr, "Failed to open %s for limit switches: %s\n",
            gpio_chip_char, strerror(errno));
    gpiod_chip_close(chip);
    return 0;
  }

  std::cout << "Initializing limit switch objects...\n";

  limitSwitch sw_y(chip4, sw1_line);
  limitSwitch sw_x(chip4, sw2_line);

  std::cout << "Finished limit switch object initialization!\n";

  // Testing the limit switch for a reading
  // std::thread sw1_thread([&]() {
  //   while (sw1.read() != 1) {
  //   }
  //   std::cout << "Limit switch 1 (Y-axis) triggered!\n";
  // });

  // std::thread sw2_thread([&]() {
  //   while (sw2.read() != 1) {
  //   }
  //   std::cout << "Limit switch 2 (X-axis) triggered!\n";
  // });

  std::cout << "Initializing motor object...\n";

  iSV57T m_lower = iSV57T(chip, m1_dir_line, m1_pul_line, pulse_per_rev);
  iSV57T m_upper = iSV57T(chip, m2_dir_line, m2_pul_line, pulse_per_rev);

  // Setting RPM Test
  // m1.set_target_rpm(700);
  // m1.set_target_rpm(700);
  // m2.set_target_rpm(1000);
  // m2.set_target_rpm(1000);
  // m1.set_target_rpm(500);
  // m2.set_target_rpm(500);
  // m1.set_target_rpm(300);
  // m2.set_target_rpm(300);
  // m1.set_target_rpm(900);

  std::cout << "Finished motor object initialization!\n";

  // Gantry Object Initialization
  std::cout << "Initializing gantry object...\n";

  gantry g = gantry(m_lower, m_upper, sw_x, sw_y);

  std::cout << "Finished gantry object initialization!\n";

  // std::cout << "Starting motor rotations...\n";

  // Make the rotate_motor function public when testing.
  // Moving North
  // g.rotate_motors(100, iSV57T::CCW, iSV57T::CW, gantry::MotorSelect::BOTH);

  // Moving South
  // g.rotate_motors(100, iSV57T::CW, iSV57T::CCW, gantry::MotorSelect::BOTH);

  // Moving East
  // g.rotate_motors(100, iSV57T::CCW, iSV57T::CCW, gantry::MotorSelect::BOTH);

  // Moving West
  // g.rotate_motors(100, iSV57T::CW, iSV57T::CW, gantry::MotorSelect::BOTH);

  // Delay line
  // std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // std::cout << "Ending motor rotations...\n";

  // sw1_thread.join();
  // sw2_thread.join();

  gpiod_chip_close(chip4);
  gpiod_chip_close(chip0);

  return 0;
}

// 63.7919 left to right mmm = 7900 degrees

// 7800 degrees top to bottom

// 125 mm diagonal == 43 rotation of 360 = 15480 degrees

// Golden Ratio: 123.84 degrees per 1 mm