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

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>

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

#pragma pack(push, 1)
struct shm_xy {
  uint32_t seq;
  uint32_t ready;
  uint32_t request_next;
  uint32_t kind;
  double x_mm;
  double y_mm;
};
#pragma pack(pop)

enum coord_kind : uint32_t {
  KIND_NONE = 0,
  KIND_STILL = 1,
  KIND_INTERCEPT = 2,
  KIND_BOUNCE = 3
};

const char *kind_to_string(uint32_t kind) {
  switch (kind) {
  case KIND_STILL:
    return "STILL";
  case KIND_INTERCEPT:
    return "INTERCEPT";
  case KIND_BOUNCE:
    return "BOUNCE";
  default:
    return "NONE";
  }
}

bool read_stable_message(const shm_xy *p, uint32_t &last_seq, uint32_t &kind,
                         double &x_mm, double &y_mm) {
  uint32_t s1 = p->seq;

  if (s1 == 0 || (s1 & 1))
    return false;

  if (!p->ready)
    return false;

  uint32_t k = p->kind;
  double x = p->x_mm;
  double y = p->y_mm;

  uint32_t s2 = p->seq;

  if (s1 != s2 || (s2 & 1))
    return false;

  if (s2 == last_seq)
    return false;

  last_seq = s2;
  kind = k;
  x_mm = x;
  y_mm = y;
  return true;
}

int main() {
  const char *gpio_chip_char = "/dev/gpiochip2";
  const unsigned m1_dir_line = 8;
  const unsigned m1_pul_line = 9;
  const unsigned m2_dir_line = 12;
  const unsigned m2_pul_line = 13;
  const uint16_t pulse_per_rev = 2000;

  const char *shm_name = "/puck_xy_mm";
  const size_t shm_size = sizeof(shm_xy);

  // gpiod_chip *chip = gpiod_chip_open(gpio_chip_char);
  // if (!chip) {
  //   fprintf(stderr, "Failed to open %s: %s\n", gpio_chip_char, strerror(errno));
  //   std::cerr << "Something bad happened\n";
  //   return 0;
  // }

  // std::cout << "Initializing motor object...\n";

  // iSV57T m1 = iSV57T(chip, m1_dir_line, m1_pul_line, pulse_per_rev);
  // iSV57T m2 = iSV57T(chip, m2_dir_line, m2_pul_line, pulse_per_rev);

  // auto rotate_both = [&](uint8_t dir) {
  //   std::thread t1([&]() { m1.rotate(dir, 360); });
  //   std::thread t2([&]() { m2.rotate(dir, 360); });
  //   t1.join();
  //   t2.join();
  // };

  std::cout << "Finished motor object initialization!\n";

  // int shm_fd = shm_open(shm_name, O_RDWR, 0666);
  // if (shm_fd < 0) {
  //   std::cerr << "Failed to open shared memory " << shm_name << ": "
  //             << strerror(errno) << "\n";
  //   gpiod_chip_close(chip);
  //   return 1;
  // }

  int shm_fd = -1;
  while (shm_fd < 0) {
  shm_fd = shm_open(shm_name, O_RDWR, 0666);
  if (shm_fd < 0) {
    std::cerr << "Waiting for shared memory " << shm_name << ": "
              << strerror(errno) << "\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

  void *addr = mmap(nullptr, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED,
                    shm_fd, 0);
  if (addr == MAP_FAILED) {
    std::cerr << "Failed to mmap shared memory: " << strerror(errno) << "\n";
    close(shm_fd);
    //gpiod_chip_close(chip);
    return 1;
  }

  shm_xy *p_shm = static_cast<shm_xy *>(addr);

  std::cout << "Waiting for coordinates from Python...\n";

  uint32_t last_seq = 0;

  while (true) {
    // Ask Python for the next coordinate
    p_shm->request_next = 1;

    uint32_t kind = KIND_NONE;
    double x_mm = 0.0;
    double y_mm = 0.0;

    while (true) {
      if (read_stable_message(p_shm, last_seq, kind, x_mm, y_mm)) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    std::cout << "[C++] accepted kind=" << kind_to_string(kind)
              << " x_mm=" << x_mm
              << " y_mm=" << y_mm << std::endl;

    // std::cout << "[C++] coordinate kind=" << kind_to_string(kind)
    //           << ". Starting motor rotations...\n";
    // rotate_both(iSV57T::CW);
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // rotate_both(iSV57T::CCW);
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // std::cout << "[C++] Ending motor rotations...\n";

    // Tell Python we consumed this coordinate and want a fresh one next
    p_shm->ready = 0;
    p_shm->request_next = 1;
  }

  munmap(addr, shm_size);
  close(shm_fd);
  //gpiod_chip_close(chip);

  return 0;
}

// #include <arpa/inet.h>

// #include <csignal>
// #include <cstdint>
// #include <netinet/in.h>
// #include <stdio.h>
// #include <sys/socket.h>
// #include <unistd.h>

// #include <chrono>
// #include <cstring>
// #include <iostream>
// #include <string>
// #include <thread>

// #include "iSV57T.hpp"
// #include "include/iSV57T.hpp"

// /** Valid GPIO Usuable Pieces
//  * GPIO1 (gpiochip2 MXM3_1 Line 8) Pin 13
//  * GPIO2 (gpiochip2 MXM3_3 Line 9) Pin 14
//  * GPIO3 (gpiochip2 MXM3_5 Line 12) Pin 15
//  * GPIO4 (gpiochip2 MXM3_7 Line 13) Pin 16
//  * GPIO5 (gpiochip6 MXM3_11 Line 1) Pin 17
//  * GPIO6 (gpiochip6 MXM3_13 Line 2) Pin 18
//  */

// int main() {
//   const char *gpio_chip_char = "/dev/gpiochip2";
//   const unsigned m1_dir_line = 8;
//   const unsigned m1_pul_line = 9;
//   const unsigned m2_dir_line = 12;
//   const unsigned m2_pul_line = 13;
//   const uint16_t pulse_per_rev = 2000;

//   gpiod_chip *chip = gpiod_chip_open(gpio_chip_char);
//   if (!chip) {
//     fprintf(stderr, "Failed to open %s: %s\n", gpio_chip_char, strerror(errno));
//     std::cerr << "Something bad happened\n";
//     return 0;
//   }

//   std::cout << "Initializing motor object...\n";

//   iSV57T m1 = iSV57T(chip, m1_dir_line, m1_pul_line, pulse_per_rev);
//   iSV57T m2 = iSV57T(chip, m2_dir_line, m2_pul_line, pulse_per_rev);

//   auto rotate_both = [&](uint8_t dir) {
//     std::thread t1([&]() { m1.rotate(dir, 360); });
//     std::thread t2([&]() { m2.rotate(dir, 360); });
//     t1.join();
//     t2.join();
//   };

//   std::cout << "Finished motor object initialization!\n";

//   std::cout << "Starting motor rotations...\n";
//   rotate_both(iSV57T::CW);
//   std::this_thread::sleep_for(std::chrono::milliseconds(500));
//   rotate_both(iSV57T::CCW);
//   std::this_thread::sleep_for(std::chrono::milliseconds(500));
//   std::cout << "Ending motor rotations...\n";

//   gpiod_chip_close(chip);

//   return 0;
// }