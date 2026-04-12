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

#include "iSV57T.hpp"
#include "include/iSV57T.hpp"

/** Valid GPIO Usuable Pieces
 * GPIO1 (gpiochip2 MXM3_1 Line 8) Pin 13
 * GPIO2 (gpiochip2 MXM3_3 Line 9) Pin 14
 * GPIO3 (gpiochip2 MXM3_5 Line 12) Pin 15
 * GPIO4 (gpiochip2 MXM3_7 Line 13) Pin 16
 * GPIO5 (gpiochip6 MXM3_11 Line 1) Pin 17
 * GPIO6 (gpiochip6 MXM3_13 Line 2) Pin 18
 */

int main() {
  const char *gpio_chip_char = "/dev/gpiochip2";
  const unsigned m1_dir_line = 8;
  const unsigned m1_pul_line = 9;
  const unsigned m2_dir_line = 12;
  const unsigned m2_pul_line = 13;
  const uint16_t pulse_per_rev = 2000;

  gpiod_chip *chip = gpiod_chip_open(gpio_chip_char);
  if (!chip) {
    fprintf(stderr, "Failed to open %s: %s\n", gpio_chip_char, strerror(errno));
    std::cerr << "Something bad happened\n";
    return 0;
  }

  std::cout << "Initializing motor object...\n";

  iSV57T m1 = iSV57T(chip, m1_dir_line, m1_pul_line, pulse_per_rev);
  iSV57T m2 = iSV57T(chip, m2_dir_line, m2_pul_line, pulse_per_rev);

  auto rotate_both = [&](uint8_t dir) {
    std::thread t1([&]() { m1.rotate(dir, 360); });
    std::thread t2([&]() { m2.rotate(dir, 360); });
    t1.join();
    t2.join();
  };

  std::cout << "Finished motor object initialization!\n";

  std::cout << "Starting motor rotations...\n";
  rotate_both(iSV57T::CW);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  rotate_both(iSV57T::CCW);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  std::cout << "Ending motor rotations...\n";

  gpiod_chip_close(chip);

  return 0;
}