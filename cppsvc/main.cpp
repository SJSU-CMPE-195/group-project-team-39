#include <arpa/inet.h>

#include <atomic>
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
#include <mutex>

std::mutex print_mtx;
std::atomic<bool> keep_running(true);

void handle_signal(int) {
  keep_running = false;
}

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
  KIND_BOUNCE = 3,
  KIND_RIGHT_SIDE = 4,
  KIND_OUT_OF_RANGE = 5
};

const char *kind_to_string(uint32_t kind) {
  switch (kind) {
    case KIND_STILL:
      return "STILL";
    case KIND_INTERCEPT:
      return "INTERCEPT";
    case KIND_BOUNCE:
      return "BOUNCE";
    case KIND_RIGHT_SIDE:
      return "RIGHT_SIDE";
    case KIND_OUT_OF_RANGE:
      return "OUT_OF_RANGE";
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
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  const char *gpio_chip_char = "/dev/gpiochip0";
  const unsigned m1_dir_line = 8;
  const unsigned m1_pul_line = 9;
  const unsigned m2_dir_line = 12;
  const unsigned m2_pul_line = 13;
  const uint16_t pulse_per_rev = 2000;

  const char *shm_name = "/puck_xy_mm";
  const size_t shm_size = sizeof(shm_xy);

  {
    std::lock_guard<std::mutex> lock(print_mtx);
    std::cout << "[C++] main started" << std::endl;
    std::cout << "Finished motor object initialization!" << std::endl;
  }

  int shm_fd = -1;
  while (keep_running && shm_fd < 0) {
    shm_fd = shm_open(shm_name, O_RDWR, 0666);
    if (shm_fd < 0) {
      std::lock_guard<std::mutex> lock(print_mtx);
      std::cerr << "Waiting for shared memory " << shm_name << ": "
                << strerror(errno) << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }

  if (!keep_running) {
    std::lock_guard<std::mutex> lock(print_mtx);
    std::cout << "[C++] shutdown requested before shm_open completed" << std::endl;
    return 0;
  }

  {
    std::lock_guard<std::mutex> lock(print_mtx);
    std::cout << "[C++] shm_open succeeded" << std::endl;
  }

  void *addr = mmap(nullptr, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED,
                    shm_fd, 0);
  if (addr == MAP_FAILED) {
    std::lock_guard<std::mutex> lock(print_mtx);
    std::cerr << "Failed to mmap shared memory: " << strerror(errno) << std::endl;
    close(shm_fd);
    //gpiod_chip_close(chip);
    return 1;
  }

  shm_xy *p_shm = static_cast<shm_xy *>(addr);

  {
    std::lock_guard<std::mutex> lock(print_mtx);
    std::cout << "Waiting for coordinates from Python..." << std::endl;
  }

  uint32_t last_seq = 0;

  while (keep_running) {
    // Ask Python for the next coordinate
    p_shm->request_next = 1;
    {
      std::lock_guard<std::mutex> lock(print_mtx);
      std::cout << "[C++] set request_next=1" << std::endl;
    }

    uint32_t kind = KIND_NONE;
    double x_mm = 0.0;
    double y_mm = 0.0;

    while (keep_running) {
      if (read_stable_message(p_shm, last_seq, kind, x_mm, y_mm)) {
        std::lock_guard<std::mutex> lock(print_mtx);
        std::cout << "[C++] got stable message" << std::endl;
        break;
      }

      {
        std::lock_guard<std::mutex> lock(print_mtx);
        std::cout << "[C++] waiting: "
                  << "seq=" << p_shm->seq
                  << " kind=" << p_shm->kind
                  << " x_mm=" << p_shm->x_mm
                  << " y_mm=" << p_shm->y_mm
                  << std::endl;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    if (!keep_running) {
      break;
    }

    {
      std::lock_guard<std::mutex> lock(print_mtx);
      std::cout << "[C++] accepted kind=" << kind_to_string(kind)
                << " x_mm=" << x_mm
                << " y_mm=" << y_mm << std::endl;
    }

    if (kind == KIND_RIGHT_SIDE) {
      std::lock_guard<std::mutex> lock(print_mtx);
      std::cout << "[C++] puck is on opponent side, not moving motors." << std::endl;
    } else if (kind == KIND_OUT_OF_RANGE) {
      std::lock_guard<std::mutex> lock(print_mtx);
      std::cout << "[C++] target y_mm outside robot allowed range, not moving motors." << std::endl;
    } else {
      // std::cout << "[C++] coordinate kind=" << kind_to_string(kind)
      //           << ". Starting motor rotations...\n";
      // rotate_both(iSV57T::CW);
      // std::this_thread::sleep_for(std::chrono::milliseconds(500));
      // rotate_both(iSV57T::CCW);
      // std::this_thread::sleep_for(std::chrono::milliseconds(500));
      // std::cout << "[C++] Ending motor rotations...\n";
    }

    // Tell Python we consumed this coordinate and want a fresh one next
    p_shm->ready = 0;
    p_shm->request_next = 1;
    {
      std::lock_guard<std::mutex> lock(print_mtx);
      std::cout << "[C++] consumed message, set ready=0 request_next=1" << std::endl;
    }
  }

  {
    std::lock_guard<std::mutex> lock(print_mtx);
    std::cout << "[C++] shutting down cleanly" << std::endl;
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

// #include <fcntl.h>
// #include <sys/mman.h>
// #include <sys/stat.h>

// #include "iSV57T.hpp"
// #include "include/iSV57T.hpp"
// #include <mutex>

// std::mutex print_mtx;

// /** Valid GPIO Usuable Pieces
//  * GPIO1 (gpiochip2 MXM3_1 Line 8) Pin 13
//  * GPIO2 (gpiochip2 MXM3_3 Line 9) Pin 14
//  * GPIO3 (gpiochip2 MXM3_5 Line 12) Pin 15
//  * GPIO4 (gpiochip2 MXM3_7 Line 13) Pin 16
//  * GPIO5 (gpiochip6 MXM3_11 Line 1) Pin 17
//  * GPIO6 (gpiochip6 MXM3_13 Line 2) Pin 18
//  */

// #pragma pack(push, 1)
// struct shm_xy {
//   uint32_t seq;
//   uint32_t ready;
//   uint32_t request_next;
//   uint32_t kind;
//   double x_mm;
//   double y_mm;
// };
// #pragma pack(pop)

// enum coord_kind : uint32_t {
//   KIND_NONE = 0,
//   KIND_STILL = 1,
//   KIND_INTERCEPT = 2,
//   KIND_BOUNCE = 3,
//   KIND_RIGHT_SIDE = 4,
//   KIND_OUT_OF_RANGE = 5
// };

// const char *kind_to_string(uint32_t kind) {
//   switch (kind) {
//     case KIND_STILL:
//       return "STILL";
//     case KIND_INTERCEPT:
//       return "INTERCEPT";
//     case KIND_BOUNCE:
//       return "BOUNCE";
//     case KIND_RIGHT_SIDE:
//       return "RIGHT_SIDE";
//     case KIND_OUT_OF_RANGE:
//       return "OUT_OF_RANGE";
//     default:
//       return "NONE";
//   }
// }

// bool read_stable_message(const shm_xy *p, uint32_t &last_seq, uint32_t &kind,
//                          double &x_mm, double &y_mm) {
//   uint32_t s1 = p->seq;

//   if (s1 == 0 || (s1 & 1))
//     return false;

//   if (!p->ready)
//     return false;

//   uint32_t k = p->kind;
//   double x = p->x_mm;
//   double y = p->y_mm;

//   uint32_t s2 = p->seq;

//   if (s1 != s2 || (s2 & 1))
//     return false;

//   if (s2 == last_seq)
//     return false;

//   last_seq = s2;
//   kind = k;
//   x_mm = x;
//   y_mm = y;
//   return true;
// }

// int main() {
//   const char *gpio_chip_char = "/dev/gpiochip0";
//   const unsigned m1_dir_line = 8;
//   const unsigned m1_pul_line = 9;
//   const unsigned m2_dir_line = 12;
//   const unsigned m2_pul_line = 13;
//   const uint16_t pulse_per_rev = 2000;

//   const char *shm_name = "/puck_xy_mm";
//   const size_t shm_size = sizeof(shm_xy);

//   std::cout << "[C++] main started" << std::endl;
//   std::cout << "Finished motor object initialization!" << std::endl;

//   int shm_fd = -1;
//   while (shm_fd < 0) {
//     shm_fd = shm_open(shm_name, O_RDWR, 0666);
//     if (shm_fd < 0) {
//       std::cerr << "Waiting for shared memory " << shm_name << ": "
//                 << strerror(errno) << std::endl;
//       std::this_thread::sleep_for(std::chrono::milliseconds(500));
//     }
//   }

//   std::cout << "[C++] shm_open succeeded" << std::endl;

//   void *addr = mmap(nullptr, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED,
//                     shm_fd, 0);
//   if (addr == MAP_FAILED) {
//     std::cerr << "Failed to mmap shared memory: " << strerror(errno) << std::endl;
//     close(shm_fd);
//     //gpiod_chip_close(chip);
//     return 1;
//   }

//   shm_xy *p_shm = static_cast<shm_xy *>(addr);

//   std::cout << "Waiting for coordinates from Python..." << std::endl;

//   uint32_t last_seq = 0;

//   while (true) {
//     // Ask Python for the next coordinate
//     p_shm->request_next = 1;
//     std::cout << "[C++] set request_next=1" << std::endl;

//     uint32_t kind = KIND_NONE;
//     double x_mm = 0.0;
//     double y_mm = 0.0;

//     while (true) {
//       if (read_stable_message(p_shm, last_seq, kind, x_mm, y_mm)) {
//         std::cout << "[C++] got stable message" << std::endl;
//         break;
//       }

//       std::cout << "[C++] waiting: "
//                 << "seq=" << p_shm->seq
//                 << " kind=" << p_shm->kind
//                 << " x_mm=" << p_shm->x_mm
//                 << " y_mm=" << p_shm->y_mm
//                 << std::endl;

//       std::this_thread::sleep_for(std::chrono::milliseconds(200));
//     }

//     std::cout << "[C++] accepted kind=" << kind_to_string(kind)
//               << " x_mm=" << x_mm
//               << " y_mm=" << y_mm << std::endl;

//     if (kind == KIND_RIGHT_SIDE) {
//       std::cout << "[C++] puck is on opponent side, not moving motors." << std::endl;
//     } else if (kind == KIND_OUT_OF_RANGE) {
//       std::cout << "[C++] target y_mm outside robot allowed range, not moving motors." << std::endl;
//     } else {
//       // std::cout << "[C++] coordinate kind=" << kind_to_string(kind)
//       //           << ". Starting motor rotations...\n";
//       // rotate_both(iSV57T::CW);
//       // std::this_thread::sleep_for(std::chrono::milliseconds(500));
//       // rotate_both(iSV57T::CCW);
//       // std::this_thread::sleep_for(std::chrono::milliseconds(500));
//       // std::cout << "[C++] Ending motor rotations...\n";
//     }

//     // Tell Python we consumed this coordinate and want a fresh one next
//     p_shm->ready = 0;
//     p_shm->request_next = 1;
//     std::cout << "[C++] consumed message, set ready=0 request_next=1" << std::endl;
//   }

//   munmap(addr, shm_size);
//   close(shm_fd);
//   //gpiod_chip_close(chip);

//   return 0;
// }


// // #include <arpa/inet.h>

// // #include <csignal>
// // #include <cstdint>
// // #include <netinet/in.h>
// // #include <stdio.h>
// // #include <sys/socket.h>
// // #include <unistd.h>

// // #include <chrono>
// // #include <cstring>
// // #include <iostream>
// // #include <string>
// // #include <thread>

// // #include <fcntl.h>
// // #include <sys/mman.h>
// // #include <sys/stat.h>

// // #include "iSV57T.hpp"
// // #include "include/iSV57T.hpp"

// // /** Valid GPIO Usuable Pieces
// //  * GPIO1 (gpiochip2 MXM3_1 Line 8) Pin 13
// //  * GPIO2 (gpiochip2 MXM3_3 Line 9) Pin 14
// //  * GPIO3 (gpiochip2 MXM3_5 Line 12) Pin 15
// //  * GPIO4 (gpiochip2 MXM3_7 Line 13) Pin 16
// //  * GPIO5 (gpiochip6 MXM3_11 Line 1) Pin 17
// //  * GPIO6 (gpiochip6 MXM3_13 Line 2) Pin 18
// //  */

// // #pragma pack(push, 1)
// // struct shm_xy {
// //   uint32_t seq;
// //   uint32_t ready;
// //   uint32_t request_next;
// //   uint32_t kind;
// //   double x_mm;
// //   double y_mm;
// // };
// // #pragma pack(pop)

// // enum coord_kind : uint32_t {
// //   KIND_NONE = 0,
// //   KIND_STILL = 1,
// //   KIND_INTERCEPT = 2,
// //   KIND_BOUNCE = 3,
// //   KIND_RIGHT_SIDE = 4,
// //   KIND_OUT_OF_RANGE = 5
// // };

// // const char *kind_to_string(uint32_t kind) {
// // switch (kind) {
// //   case KIND_STILL:
// //     return "STILL";
// //   case KIND_INTERCEPT:
// //     return "INTERCEPT";
// //   case KIND_BOUNCE:
// //     return "BOUNCE";
// //   case KIND_RIGHT_SIDE:
// //     return "RIGHT_SIDE";
// //   case KIND_OUT_OF_RANGE:
// //     return "OUT_OF_RANGE";
// //   default:
// //     return "NONE";
// //   }
// // }

// // bool read_stable_message(const shm_xy *p, uint32_t &last_seq, uint32_t &kind, double &x_mm, double &y_mm) {
// //   uint32_t s1 = p->seq;

// //   if (s1 == 0 || (s1 & 1))
// //   return false;

// //   if (!p->ready)
// //   return false;

// //   uint32_t k = p->kind;
// //   double x = p->x_mm;
// //   double y = p->y_mm;

// //   uint32_t s2 = p->seq;

// //   if (s1 != s2 || (s2 & 1))
// //     return false;

// //   if (s2 == last_seq)
// //     return false;

// //   last_seq = s2;
// //   kind = k;
// //   x_mm = x;
// //   y_mm = y;
// //   return true;
// // }

// // int main() {
// //   const char *gpio_chip_char = "/dev/gpiochip0";
// //   const unsigned m1_dir_line = 8;
// //   const unsigned m1_pul_line = 9;
// //   const unsigned m2_dir_line = 12;
// //   const unsigned m2_pul_line = 13;
// //   const uint16_t pulse_per_rev = 2000;

// //   const char *shm_name = "/puck_xy_mm";
// //   const size_t shm_size = sizeof(shm_xy);

// //   std::cout << "Finished motor object initialization!\n";

// //   int shm_fd = -1;
// //   while (shm_fd < 0) {
// //     shm_fd = shm_open(shm_name, O_RDWR, 0666);
// //     if (shm_fd < 0) {
// //       std::cerr << "Waiting for shared memory " << shm_name << ": "
// //                 << strerror(errno) << "\n";
// //       std::this_thread::sleep_for(std::chrono::milliseconds(500));
// //     }
// //   }

// //   void *addr = mmap(nullptr, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED,
// //                   shm_fd, 0);
// //   if (addr == MAP_FAILED) {
// //     std::cerr << "Failed to mmap shared memory: " << strerror(errno) << "\n";
// //     close(shm_fd);
// //     //gpiod_chip_close(chip);
// //     return 1;
// //   }

// //   shm_xy *p_shm = static_cast<shm_xy *>(addr);

// //   std::cout << "Waiting for coordinates from Python...\n";

// //   uint32_t last_seq = 0;

// //   while (true) {
// //   // Ask Python for the next coordinate
// //   p_shm->request_next = 1;

// //   uint32_t kind = KIND_NONE;
// //   double x_mm = 0.0;
// //   double y_mm = 0.0;

// //   while (true) {
// //     if (read_stable_message(p_shm, last_seq, kind, x_mm, y_mm)) {
// //       break;
// //     }
// //     std::this_thread::sleep_for(std::chrono::milliseconds(20));
// //   }

// //   std::cout << "[C++] accepted kind=" << kind_to_string(kind)
// //             << " x_mm=" << x_mm
// //             << " y_mm=" << y_mm << "\n";

// //   if (kind == KIND_RIGHT_SIDE) {
// //     std::cout << "[C++] puck is on opponent side, not moving motors.\n";
// //   } else if (kind == KIND_OUT_OF_RANGE) {
// //     std::cout << "[C++] target y_mm outside robot allowed range, not moving motors.\n";
// //   } else {
// //     // std::cout << "[C++] coordinate kind=" << kind_to_string(kind)
// //     //           << ". Starting motor rotations...\n";
// //     // rotate_both(iSV57T::CW);
// //     // std::this_thread::sleep_for(std::chrono::milliseconds(500));
// //     // rotate_both(iSV57T::CCW);
// //     // std::this_thread::sleep_for(std::chrono::milliseconds(500));
// //     // std::cout << "[C++] Ending motor rotations...\n";
// //   }

// //   // Tell Python we consumed this coordinate and want a fresh one next
// //   p_shm->ready = 0;
// //   p_shm->request_next = 1;
// //   }

// //   munmap(addr, shm_size);
// //   close(shm_fd);
// //   //gpiod_chip_close(chip);

// //   return 0;
// // }


// // // #include <arpa/inet.h>

// // // #include <csignal>
// // // #include <cstdint>
// // // #include <netinet/in.h>
// // // #include <stdio.h>
// // // #include <sys/socket.h>
// // // #include <unistd.h>
// // // //#include "include/gantry.hpp"
// // // #include <chrono>
// // // #include <cmath>
// // // #include <cstring>
// // // #include <iostream>
// // // #include <string>
// // // #include <thread>

// // // #include <fcntl.h>
// // // #include <sys/mman.h>
// // // #include <sys/stat.h>

// // // #include "iSV57T.hpp"
// // // #include "include/iSV57T.hpp"

// // // /** Valid GPIO Usuable Pieces
// // //  * GPIO1 (gpiochip2 MXM3_1 Line 8) Pin 13
// // //  * GPIO2 (gpiochip2 MXM3_3 Line 9) Pin 14
// // //  * GPIO3 (gpiochip2 MXM3_5 Line 12) Pin 15
// // //  * GPIO4 (gpiochip2 MXM3_7 Line 13) Pin 16
// // //  * GPIO5 (gpiochip6 MXM3_11 Line 1) Pin 17
// // //  * GPIO6 (gpiochip6 MXM3_13 Line 2) Pin 18
// // //  */

// // // #pragma pack(push, 1)
// // // struct shm_xy {
// // //   uint32_t seq;
// // //   uint32_t ready;
// // //   uint32_t request_next;
// // //   uint32_t kind;
// // //   double x_mm;
// // //   double y_mm;
// // // };
// // // #pragma pack(pop)

// // // enum coord_kind : uint32_t {
// // //   KIND_NONE = 0,
// // //   KIND_STILL = 1,
// // //   KIND_INTERCEPT = 2,
// // //   KIND_BOUNCE = 3,
// // //   KIND_RIGHT_SIDE = 4,
// // //   KIND_OUT_OF_RANGE = 5
// // // };

// // // const char *kind_to_string(uint32_t kind) {
// // // switch (kind) {
// // //   case KIND_STILL:
// // //     return "STILL";
// // //   case KIND_INTERCEPT:
// // //     return "INTERCEPT";
// // //   case KIND_BOUNCE:
// // //     return "BOUNCE";
// // //   case KIND_RIGHT_SIDE:
// // //     return "RIGHT_SIDE";
// // //   case KIND_OUT_OF_RANGE:
// // //     return "OUT_OF_RANGE";
// // //   default:
// // //     return "NONE";
// // //   }
// // // }

// // // bool read_stable_message(const shm_xy *p, uint32_t &last_seq, uint32_t &kind, double &x_mm, double &y_mm) {
// // //   uint32_t s1 = p->seq;

// // //   if (s1 == 0 || (s1 & 1))
// // //     return false;

// // //   if (!p->ready)
// // //     return false;

// // //   uint32_t k = p->kind;
// // //   double x = p->x_mm;
// // //   double y = p->y_mm;

// // //   uint32_t s2 = p->seq;

// // //   if (s1 != s2 || (s2 & 1))
// // //     return false;

// // //   if (s2 == last_seq)
// // //     return false;

// // //   last_seq = s2;
// // //   kind = k;
// // //   x_mm = x;
// // //   y_mm = y;
// // //   return true;
// // // }

// // // int main() {
// // //   const char *gpio_chip_char = "/dev/gpiochip0";
// // //   const unsigned m1_dir_line = 8;
// // //   const unsigned m1_pul_line = 9;
// // //   const unsigned m2_dir_line = 12;
// // //   const unsigned m2_pul_line = 13;
// // //   const uint16_t pulse_per_rev = 2000;

// // //   const char *shm_name = "/puck_xy_mm";
// // //   const size_t shm_size = sizeof(shm_xy);

// // //   gpiod_chip *chip = gpiod_chip_open(gpio_chip_char);
// // //   if (!chip) {
// // //     fprintf(stderr, "Failed to open %s: %s\n", gpio_chip_char, strerror(errno));
// // //     std::cerr << "Something bad happened\n";
// // //     return 0;
// // //   }

// // //   std::cout << "Initializing motor object...\n";

// // //   // iSV57T m1 = iSV57T(chip, m1_dir_line, m1_pul_line, pulse_per_rev);
// // //   // iSV57T m2 = iSV57T(chip, m2_dir_line, m2_pul_line, pulse_per_rev);

// // //   iSV57T m1 = iSV57T(chip, m1_dir_line, m1_pul_line, pulse_per_rev);
// // //   iSV57T m2 = iSV57T(chip, m2_dir_line, m2_pul_line, pulse_per_rev);

// // //   std::cout << "Finished motor object initialization!\n";

// // //   // Tunable motion settings
// // //   //constexpr double DEG_PER_MM = 123.84;
// // //   constexpr double DEG_PER_MM = 8.84;
// // //   constexpr double MIN_MOVE_MM = 1.0;

// // //   try {
// // //     m1.set_target_rpm(500);
// // //     m2.set_target_rpm(500);
// // //   } catch (const std::exception &e) {
// // //     std::cerr << "Failed to set target RPM: " << e.what() << "\n";
// // //     gpiod_chip_close(chip);
// // //     return 1;
// // //   }

// // //   // Current gantry position in mm, based on commanded motion only.
// // //   double curr_x_mm = 0.0;
// // //   double curr_y_mm = 0.0;

// // //   auto mm_to_deg = [&](double mm) -> double {
// // //     return std::abs(mm) * DEG_PER_MM;
// // //   };

// // //   // Y motion: both motors same direction.
// // //   // If physical motion is reversed, swap CW <-> CCW here.
// // //   auto move_y_mm = [&](double dy_mm) {
// // //     if (std::abs(dy_mm) < MIN_MOVE_MM)
// // //       return;

// // //     const double deg = mm_to_deg(dy_mm);
// // //     const uint8_t dir = (dy_mm > 0.0) ? iSV57T::CW : iSV57T::CCW;

// // //     std::thread t1([&]() { m1.rotate(dir, deg); });
// // //     std::thread t2([&]() { m2.rotate(dir, deg); });
// // //     t1.join();
// // //     t2.join();

// // //     curr_y_mm += dy_mm;
// // //   };

// // //   // X motion: motors opposite direction.
// // //   // If physical motion is reversed, swap the two cases here.
// // //   auto move_x_mm = [&](double dx_mm) {
// // //     if (std::abs(dx_mm) < MIN_MOVE_MM)
// // //       return;

// // //     const double deg = mm_to_deg(dx_mm);

// // //     if (dx_mm > 0.0) {
// // //       // Positive X
// // //       std::thread t1([&]() { m1.rotate(iSV57T::CCW, deg); });
// // //       std::thread t2([&]() { m2.rotate(iSV57T::CW, deg); });
// // //       t1.join();
// // //       t2.join();
// // //     } else {
// // //       // Negative X
// // //       std::thread t1([&]() { m1.rotate(iSV57T::CW, deg); });
// // //       std::thread t2([&]() { m2.rotate(iSV57T::CCW, deg); });
// // //       t1.join();
// // //       t2.join();
// // //     }

// // //     curr_x_mm += dx_mm;
// // //   };

// // //   auto move_to_xy_mm = [&](double target_x_mm, double target_y_mm) {
// // //     if((target_x_mm >= 800) || (target_y_mm >= 800)){
// // //       std::cout << "Out of boundary (800)\n";
// // //       return;
// // //     }
// // //     const double dx_mm = target_x_mm - curr_x_mm;
// // //     const double dy_mm = target_y_mm - curr_y_mm;

// // //     std::cout << "[C++] move request: target=("
// // //               << target_x_mm << ", " << target_y_mm
// // //               << ") curr=(" << curr_x_mm << ", " << curr_y_mm
// // //               << ") delta=(" << dx_mm << ", " << dy_mm << ")\n";

// // //     // Simple sequential decomposition.
// // //     move_x_mm(dx_mm);
// // //     move_y_mm(dy_mm);

// // //     std::cout << "[C++] new estimated position=("
// // //               << curr_x_mm << ", " << curr_y_mm << ")\n";
// // //   };

// // //   // gpiod_chip *chip = gpiod_chip_open(gpio_chip_char);
// // //   // if (!chip) {
// // //   //   fprintf(stderr, "Failed to open %s: %s\n", gpio_chip_char, strerror(errno));
// // //   //   std::cerr << "Something bad happened\n";
// // //   //   return 0;
// // //   // }

// // //   // std::cout << "Initializing motor object...\n";

// // //   // iSV57T m1 = iSV57T(chip, m1_dir_line, m1_pul_line, pulse_per_rev);
// // //   // iSV57T m2 = iSV57T(chip, m2_dir_line, m2_pul_line, pulse_per_rev);

// // //   // auto rotate_both = [&](uint8_t dir) {
// // //   //   std::thread t1([&]() { m1.rotate(dir, 360); });
// // //   //   std::thread t2([&]() { m2.rotate(dir, 360); });
// // //   //   t1.join();
// // //   //   t2.join();
// // //   // };

// // //   std::cout << "Finished motor object initialization!\n";

// // //   // int shm_fd = shm_open(shm_name, O_RDWR, 0666);
// // //   // if (shm_fd < 0) {
// // //   //   std::cerr << "Failed to open shared memory " << shm_name << ": "
// // //   //             << strerror(errno) << "\n";
// // //   //   gpiod_chip_close(chip);
// // //   //   return 1;
// // //   // }

// // //   int shm_fd = -1;
// // //   while (shm_fd < 0) {
// // //     shm_fd = shm_open(shm_name, O_RDWR, 0666);
// // //     if (shm_fd < 0) {
// // //       std::cerr << "Waiting for shared memory " << shm_name << ": "
// // //                 << strerror(errno) << "\n";
// // //       std::this_thread::sleep_for(std::chrono::milliseconds(500));
// // //     }
// // //   }

// // //   void *addr = mmap(nullptr, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED,
// // //                   shm_fd, 0);
// // //   if (addr == MAP_FAILED) {
// // //     std::cerr << "Failed to mmap shared memory: " << strerror(errno) << "\n";
// // //     close(shm_fd);
// // //     //gpiod_chip_close(chip);
// // //     gpiod_chip_close(chip);
// // //     return 1;
// // //   }

// // //   shm_xy *p_shm = static_cast<shm_xy *>(addr);

// // //   std::cout << "Waiting for coordinates from Python...\n";

// // //   uint32_t last_seq = 0;

// // //   // std::cout << "[C++] Starting motor-only test...\n";

// // //   // move_to_xy_mm(100.0, 0.0);
// // //   // std::this_thread::sleep_for(std::chrono::milliseconds(5000)); // 5 seconds

// // //   // move_to_xy_mm(0.0, 0.0);
// // //   // std::this_thread::sleep_for(std::chrono::milliseconds(5000));

// // //   // move_to_xy_mm(0.0, 100.0);
// // //   // std::this_thread::sleep_for(std::chrono::milliseconds(5000));

// // //   // move_to_xy_mm(0.0, 0.0);
// // //   // std::this_thread::sleep_for(std::chrono::milliseconds(5000));

// // //   // move_to_xy_mm(100.0, 100.0);
// // //   // std::this_thread::sleep_for(std::chrono::milliseconds(5000));

// // //   // move_to_xy_mm(0.0, 0.0);

// // //   // std::cout << "[C++] Finished motor-only test.\n";

// // //   while (true) {
// // //   // Ask Python for the next coordinate
// // //   p_shm->request_next = 1;

// // //   uint32_t kind = KIND_NONE;
// // //   double x_mm = 0.0;
// // //   double y_mm = 0.0;

// // //   while (true) {
// // //     if (read_stable_message(p_shm, last_seq, kind, x_mm, y_mm)) {
// // //       break;
// // //     }
// // //     std::this_thread::sleep_for(std::chrono::milliseconds(20));
// // //   }

// // //   std::cout << "[C++] accepted kind=" << kind_to_string(kind)
// // //             << " x_mm=" << x_mm
// // //             << " y_mm=" << y_mm << std::endl;

// // //   if (kind == KIND_RIGHT_SIDE) {
// // //     std::cout << "[C++] puck is on opponent side, not moving motors.\n";
// // //   } else if (kind == KIND_OUT_OF_RANGE) {
// // //     std::cout << "[C++] target y_mm outside robot allowed range, not moving motors.\n";
// // //   } else {
// // //     move_to_xy_mm(x_mm, y_mm);

// // //     // std::cout << "[C++] coordinate kind=" << kind_to_string(kind)
// // //     //           << ". Starting motor rotations...\n";
// // //     // rotate_both(iSV57T::CW);
// // //     // std::this_thread::sleep_for(std::chrono::milliseconds(500));
// // //     // rotate_both(iSV57T::CCW);
// // //     // std::this_thread::sleep_for(std::chrono::milliseconds(500));
// // //     // std::cout << "[C++] Ending motor rotations...\n";
// // //   }

// // //   // Tell Python we consumed this coordinate and want a fresh one next
// // //   p_shm->ready = 0;
// // //   p_shm->request_next = 1;
// // //   }

// // //   munmap(addr, shm_size);
// // //   close(shm_fd);
// // //   //gpiod_chip_close(chip);
// // //   gpiod_chip_close(chip);

// // //   return 0;
// // // }


// // // // #include <arpa/inet.h>

// // // // #include <csignal>
// // // // #include <cstdint>
// // // // #include <netinet/in.h>
// // // // #include <stdio.h>
// // // // #include <sys/socket.h>
// // // // #include <unistd.h>
// // // // //#include "include/gantry.hpp"
// // // // #include <chrono>
// // // // #include <cmath>
// // // // #include <cstring>
// // // // #include <iostream>
// // // // #include <string>
// // // // #include <thread>

// // // // #include <fcntl.h>
// // // // #include <sys/mman.h>
// // // // #include <sys/stat.h>

// // // // #include "iSV57T.hpp"
// // // // #include "include/iSV57T.hpp"

// // // // /** Valid GPIO Usuable Pieces
// // // //  * GPIO1 (gpiochip2 MXM3_1 Line 8) Pin 13
// // // //  * GPIO2 (gpiochip2 MXM3_3 Line 9) Pin 14
// // // //  * GPIO3 (gpiochip2 MXM3_5 Line 12) Pin 15
// // // //  * GPIO4 (gpiochip2 MXM3_7 Line 13) Pin 16
// // // //  * GPIO5 (gpiochip6 MXM3_11 Line 1) Pin 17
// // // //  * GPIO6 (gpiochip6 MXM3_13 Line 2) Pin 18
// // // //  */

// // // // #pragma pack(push, 1)
// // // // struct shm_xy {
// // // //   uint32_t seq;
// // // //   uint32_t ready;
// // // //   uint32_t request_next;
// // // //   uint32_t kind;
// // // //   double x_mm;
// // // //   double y_mm;
// // // // };
// // // // #pragma pack(pop)

// // // // enum coord_kind : uint32_t {
// // // //   KIND_NONE = 0,
// // // //   KIND_STILL = 1,
// // // //   KIND_INTERCEPT = 2,
// // // //   KIND_BOUNCE = 3,
// // // //   KIND_RIGHT_SIDE = 4,
// // // //   KIND_OUT_OF_RANGE = 5
// // // // };

// // // // const char *kind_to_string(uint32_t kind) {
// // // // switch (kind) {
// // // //   case KIND_STILL:
// // // //     return "STILL";
// // // //   case KIND_INTERCEPT:
// // // //     return "INTERCEPT";
// // // //   case KIND_BOUNCE:
// // // //     return "BOUNCE";
// // // //   case KIND_RIGHT_SIDE:
// // // //     return "RIGHT_SIDE";
// // // //   case KIND_OUT_OF_RANGE:
// // // //     return "OUT_OF_RANGE";
// // // //   default:
// // // //     return "NONE";
// // // //   }
// // // // }

// // // // bool read_stable_message(const shm_xy *p, uint32_t &last_seq, uint32_t &kind, double &x_mm, double &y_mm) {
// // // //   uint32_t s1 = p->seq;

// // // //   if (s1 == 0 || (s1 & 1))
// // // //     return false;

// // // //   if (!p->ready)
// // // //     return false;

// // // //   uint32_t k = p->kind;
// // // //   double x = p->x_mm;
// // // //   double y = p->y_mm;

// // // //   uint32_t s2 = p->seq;

// // // //   if (s1 != s2 || (s2 & 1))
// // // //     return false;

// // // //   if (s2 == last_seq)
// // // //     return false;

// // // //   last_seq = s2;
// // // //   kind = k;
// // // //   x_mm = x;
// // // //   y_mm = y;
// // // //   return true;
// // // // }

// // // // int main() {
// // // //   const char *gpio_chip_char = "/dev/gpiochip0";
// // // //   const unsigned m1_dir_line = 8;
// // // //   const unsigned m1_pul_line = 9;
// // // //   const unsigned m2_dir_line = 12;
// // // //   const unsigned m2_pul_line = 13;
// // // //   const uint16_t pulse_per_rev = 2000;

// // // //   const char *shm_name = "/puck_xy_mm";
// // // //   const size_t shm_size = sizeof(shm_xy);

// // // //   gpiod_chip *chip = gpiod_chip_open(gpio_chip_char);
// // // //   if (!chip) {
// // // //     fprintf(stderr, "Failed to open %s: %s\n", gpio_chip_char, strerror(errno));
// // // //     std::cerr << "Something bad happened\n";
// // // //     return 0;
// // // //   }

// // // //   std::cout << "Initializing motor object...\n";

// // // //   // iSV57T m1 = iSV57T(chip, m1_dir_line, m1_pul_line, pulse_per_rev);
// // // //   // iSV57T m2 = iSV57T(chip, m2_dir_line, m2_pul_line, pulse_per_rev);

// // // //   std::cout << "Creating m1...\n";
// // // //   iSV57T m1 = iSV57T(chip, m1_dir_line, m1_pul_line, pulse_per_rev);
// // // //   std::cout << "m1 created.\n";

// // // //   std::cout << "Creating m2...\n";
// // // //   iSV57T m2 = iSV57T(chip, m2_dir_line, m2_pul_line, pulse_per_rev);
// // // //   std::cout << "m2 created.\n";

// // // //   std::cout << "Finished motor object initialization!\n";

// // // //   // Tunable motion settings
// // // //   //constexpr double DEG_PER_MM = 123.84;
// // // //   constexpr double DEG_PER_MM = 8.84;
// // // //   constexpr double MIN_MOVE_MM = 1.0;

// // // //   try {
// // // //     m1.set_target_rpm(500);
// // // //     m2.set_target_rpm(500);
// // // //   } catch (const std::exception &e) {
// // // //     std::cerr << "Failed to set target RPM: " << e.what() << "\n";
// // // //     gpiod_chip_close(chip);
// // // //     return 1;
// // // //   }

// // // //   // Current gantry position in mm, based on commanded motion only.
// // // //   double curr_x_mm = 0.0;
// // // //   double curr_y_mm = 0.0;

// // // //   auto mm_to_deg = [&](double mm) -> double {
// // // //     return std::abs(mm) * DEG_PER_MM;
// // // //   };

// // // //   // Y motion: both motors same direction.
// // // //   // If physical motion is reversed, swap CW <-> CCW here.
// // // //   auto move_y_mm = [&](double dy_mm) {
// // // //     if (std::abs(dy_mm) < MIN_MOVE_MM)
// // // //       return;

// // // //     const double deg = mm_to_deg(dy_mm);
// // // //     const uint8_t dir = (dy_mm > 0.0) ? iSV57T::CW : iSV57T::CCW;

// // // //     std::thread t1([&]() { m1.rotate(dir, deg); });
// // // //     std::thread t2([&]() { m2.rotate(dir, deg); });
// // // //     t1.join();
// // // //     t2.join();

// // // //     curr_y_mm += dy_mm;
// // // //   };

// // // //   // X motion: motors opposite direction.
// // // //   // If physical motion is reversed, swap the two cases here.
// // // //   auto move_x_mm = [&](double dx_mm) {
// // // //     if (std::abs(dx_mm) < MIN_MOVE_MM)
// // // //       return;

// // // //     const double deg = mm_to_deg(dx_mm);

// // // //     if (dx_mm > 0.0) {
// // // //       // Positive X
// // // //       std::thread t1([&]() { m1.rotate(iSV57T::CCW, deg); });
// // // //       std::thread t2([&]() { m2.rotate(iSV57T::CW, deg); });
// // // //       t1.join();
// // // //       t2.join();
// // // //     } else {
// // // //       // Negative X
// // // //       std::thread t1([&]() { m1.rotate(iSV57T::CW, deg); });
// // // //       std::thread t2([&]() { m2.rotate(iSV57T::CCW, deg); });
// // // //       t1.join();
// // // //       t2.join();
// // // //     }

// // // //     curr_x_mm += dx_mm;
// // // //   };

// // // //   auto move_to_xy_mm = [&](double target_x_mm, double target_y_mm) {
// // // //     const double dx_mm = target_x_mm - curr_x_mm;
// // // //     const double dy_mm = target_y_mm - curr_y_mm;

// // // //     std::cout << "[C++] move request: target=("
// // // //               << target_x_mm << ", " << target_y_mm
// // // //               << ") curr=(" << curr_x_mm << ", " << curr_y_mm
// // // //               << ") delta=(" << dx_mm << ", " << dy_mm << ")\n";

// // // //     // Simple sequential decomposition.
// // // //     move_x_mm(dx_mm);
// // // //     move_y_mm(dy_mm);

// // // //     std::cout << "[C++] new estimated position=("
// // // //               << curr_x_mm << ", " << curr_y_mm << ")\n";
// // // //   };

// // // //   // gpiod_chip *chip = gpiod_chip_open(gpio_chip_char);
// // // //   // if (!chip) {
// // // //   //   fprintf(stderr, "Failed to open %s: %s\n", gpio_chip_char, strerror(errno));
// // // //   //   std::cerr << "Something bad happened\n";
// // // //   //   return 0;
// // // //   // }

// // // //   // std::cout << "Initializing motor object...\n";

// // // //   // iSV57T m1 = iSV57T(chip, m1_dir_line, m1_pul_line, pulse_per_rev);
// // // //   // iSV57T m2 = iSV57T(chip, m2_dir_line, m2_pul_line, pulse_per_rev);

// // // //   // auto rotate_both = [&](uint8_t dir) {
// // // //   //   std::thread t1([&]() { m1.rotate(dir, 360); });
// // // //   //   std::thread t2([&]() { m2.rotate(dir, 360); });
// // // //   //   t1.join();
// // // //   //   t2.join();
// // // //   // };

// // // //   std::cout << "Finished motor object initialization!\n";

// // // //   // int shm_fd = shm_open(shm_name, O_RDWR, 0666);
// // // //   // if (shm_fd < 0) {
// // // //   //   std::cerr << "Failed to open shared memory " << shm_name << ": "
// // // //   //             << strerror(errno) << "\n";
// // // //   //   gpiod_chip_close(chip);
// // // //   //   return 1;
// // // //   // }

// // // //   // int shm_fd = -1;
// // // //   // while (shm_fd < 0) {
// // // //   //   shm_fd = shm_open(shm_name, O_RDWR, 0666);
// // // //   //   if (shm_fd < 0) {
// // // //   //     std::cerr << "Waiting for shared memory " << shm_name << ": "
// // // //   //               << strerror(errno) << "\n";
// // // //   //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
// // // //   //   }
// // // //   // }

// // // //   // void *addr = mmap(nullptr, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED,
// // // //   //                 shm_fd, 0);
// // // //   // if (addr == MAP_FAILED) {
// // // //   //   std::cerr << "Failed to mmap shared memory: " << strerror(errno) << "\n";
// // // //   //   close(shm_fd);
// // // //   //   //gpiod_chip_close(chip);
// // // //   //   gpiod_chip_close(chip);
// // // //   //   return 1;
// // // //   // }

// // // //   // shm_xy *p_shm = static_cast<shm_xy *>(addr);

// // // //   // std::cout << "Waiting for coordinates from Python...\n";

// // // //   // uint32_t last_seq = 0;

// // // //   std::cout << "[C++] Starting motor-only test...\n";

// // // //   move_to_xy_mm(100.0, 0.0);
// // // //   std::this_thread::sleep_for(std::chrono::milliseconds(5000)); // 5 seconds

// // // //   move_to_xy_mm(0.0, 0.0);
// // // //   std::this_thread::sleep_for(std::chrono::milliseconds(5000));

// // // //   move_to_xy_mm(0.0, 100.0);
// // // //   std::this_thread::sleep_for(std::chrono::milliseconds(5000));

// // // //   move_to_xy_mm(0.0, 0.0);
// // // //   std::this_thread::sleep_for(std::chrono::milliseconds(5000));

// // // //   move_to_xy_mm(100.0, 100.0);
// // // //   std::this_thread::sleep_for(std::chrono::milliseconds(5000));

// // // //   move_to_xy_mm(0.0, 0.0);

// // // //   std::cout << "[C++] Finished motor-only test.\n";

// // // //   // while (true) {
// // // //   // // Ask Python for the next coordinate
// // // //   // p_shm->request_next = 1;

// // // //   // uint32_t kind = KIND_NONE;
// // // //   // double x_mm = 0.0;
// // // //   // double y_mm = 0.0;

// // // //   // while (true) {
// // // //   //   if (read_stable_message(p_shm, last_seq, kind, x_mm, y_mm)) {
// // // //   //     break;
// // // //   //   }
// // // //   //   std::this_thread::sleep_for(std::chrono::milliseconds(20));
// // // //   // }

// // // //   // std::cout << "[C++] accepted kind=" << kind_to_string(kind)
// // // //   //           << " x_mm=" << x_mm
// // // //   //           << " y_mm=" << y_mm << std::endl;

// // // //   // if (kind == KIND_RIGHT_SIDE) {
// // // //   //   std::cout << "[C++] puck is on opponent side, not moving motors.\n";
// // // //   // } else if (kind == KIND_OUT_OF_RANGE) {
// // // //   //   std::cout << "[C++] target y_mm outside robot allowed range, not moving motors.\n";
// // // //   // } else {
// // // //   //   move_to_xy_mm(x_mm, y_mm);

// // // //   //   // std::cout << "[C++] coordinate kind=" << kind_to_string(kind)
// // // //   //   //           << ". Starting motor rotations...\n";
// // // //   //   // rotate_both(iSV57T::CW);
// // // //   //   // std::this_thread::sleep_for(std::chrono::milliseconds(500));
// // // //   //   // rotate_both(iSV57T::CCW);
// // // //   //   // std::this_thread::sleep_for(std::chrono::milliseconds(500));
// // // //   //   // std::cout << "[C++] Ending motor rotations...\n";
// // // //   // }

// // // //   // // Tell Python we consumed this coordinate and want a fresh one next
// // // //   // p_shm->ready = 0;
// // // //   // p_shm->request_next = 1;
// // // //   // }

// // // //   // munmap(addr, shm_size);
// // // //   // close(shm_fd);
// // // //   // //gpiod_chip_close(chip);
// // // //   gpiod_chip_close(chip);

// // // //   return 0;
// // // // }



// // // // // #include <arpa/inet.h>

// // // // // #include <csignal>
// // // // // #include <cstdint>
// // // // // #include <netinet/in.h>
// // // // // #include <stdio.h>
// // // // // #include <sys/socket.h>
// // // // // #include <unistd.h>

// // // // // #include <chrono>
// // // // // #include <cmath>
// // // // // #include <cstring>
// // // // // #include <iostream>
// // // // // #include <string>
// // // // // #include <thread>

// // // // // #include <fcntl.h>
// // // // // #include <sys/mman.h>
// // // // // #include <sys/stat.h>

// // // // // #include "iSV57T.hpp"
// // // // // #include "include/iSV57T.hpp"

// // // // // /** Valid GPIO Usuable Pieces
// // // // //  * GPIO1 (gpiochip2 MXM3_1 Line 8) Pin 13
// // // // //  * GPIO2 (gpiochip2 MXM3_3 Line 9) Pin 14
// // // // //  * GPIO3 (gpiochip2 MXM3_5 Line 12) Pin 15
// // // // //  * GPIO4 (gpiochip2 MXM3_7 Line 13) Pin 16
// // // // //  * GPIO5 (gpiochip6 MXM3_11 Line 1) Pin 17
// // // // //  * GPIO6 (gpiochip6 MXM3_13 Line 2) Pin 18
// // // // //  */

// // // // // #pragma pack(push, 1)
// // // // // struct shm_xy {
// // // // //   uint32_t seq;
// // // // //   uint32_t ready;
// // // // //   uint32_t request_next;
// // // // //   uint32_t kind;
// // // // //   double x_mm;
// // // // //   double y_mm;
// // // // // };
// // // // // #pragma pack(pop)

// // // // // enum coord_kind : uint32_t {
// // // // //   KIND_NONE = 0,
// // // // //   KIND_STILL = 1,
// // // // //   KIND_INTERCEPT = 2,
// // // // //   KIND_BOUNCE = 3,
// // // // //   KIND_RIGHT_SIDE = 4,
// // // // //   KIND_OUT_OF_RANGE = 5
// // // // // };

// // // // // const char *kind_to_string(uint32_t kind) {
// // // // // switch (kind) {
// // // // //   case KIND_STILL:
// // // // //     return "STILL";
// // // // //   case KIND_INTERCEPT:
// // // // //     return "INTERCEPT";
// // // // //   case KIND_BOUNCE:
// // // // //     return "BOUNCE";
// // // // //   case KIND_RIGHT_SIDE:
// // // // //     return "RIGHT_SIDE";
// // // // //   case KIND_OUT_OF_RANGE:
// // // // //     return "OUT_OF_RANGE";
// // // // //   default:
// // // // //     return "NONE";
// // // // //   }
// // // // // }

// // // // // bool read_stable_message(const shm_xy *p, uint32_t &last_seq, uint32_t &kind, double &x_mm, double &y_mm) {
// // // // //   uint32_t s1 = p->seq;

// // // // //   if (s1 == 0 || (s1 & 1))
// // // // //   return false;

// // // // //   if (!p->ready)
// // // // //   return false;

// // // // //   uint32_t k = p->kind;
// // // // //   double x = p->x_mm;
// // // // //   double y = p->y_mm;

// // // // //   uint32_t s2 = p->seq;

// // // // //   if (s1 != s2 || (s2 & 1))
// // // // //     return false;

// // // // //   if (s2 == last_seq)
// // // // //     return false;

// // // // //   last_seq = s2;
// // // // //   kind = k;
// // // // //   x_mm = x;
// // // // //   y_mm = y;
// // // // //   return true;
// // // // // }

// // // // // int main() {
// // // // //   const char *gpio_chip_char = "/dev/gpiochip2";
// // // // //   const unsigned m1_dir_line = 8;
// // // // //   const unsigned m1_pul_line = 9;
// // // // //   const unsigned m2_dir_line = 12;
// // // // //   const unsigned m2_pul_line = 13;
// // // // //   const uint16_t pulse_per_rev = 2000;

// // // // //   const char *shm_name = "/puck_xy_mm";
// // // // //   const size_t shm_size = sizeof(shm_xy);

// // // // //   gpiod_chip *chip = gpiod_chip_open(gpio_chip_char);
// // // // //   if (!chip) {
// // // // //     fprintf(stderr, "Failed to open %s: %s\n", gpio_chip_char, strerror(errno));
// // // // //     std::cerr << "Something bad happened\n";
// // // // //     return 0;
// // // // //   }

// // // // //   std::cout << "Initializing motor object...\n";

// // // // //   iSV57T m1 = iSV57T(chip, m1_dir_line, m1_pul_line, pulse_per_rev);
// // // // //   iSV57T m2 = iSV57T(chip, m2_dir_line, m2_pul_line, pulse_per_rev);

// // // // //   // Tunable motion settings
// // // // //   constexpr double DEG_PER_MM = 123.84;
// // // // //   constexpr double MIN_MOVE_MM = 1.0;

// // // // //   try {
// // // // //     m1.set_target_rpm(700);
// // // // //     m2.set_target_rpm(700);
// // // // //   } catch (const std::exception &e) {
// // // // //     std::cerr << "Failed to set target RPM: " << e.what() << "\n";
// // // // //     gpiod_chip_close(chip);
// // // // //     return 1;
// // // // //   }

// // // // //   // Current gantry position in mm, based on commanded motion only.
// // // // //   double curr_x_mm = 0.0;
// // // // //   double curr_y_mm = 0.0;

// // // // //   auto mm_to_deg = [&](double mm) -> double {
// // // // //     return std::abs(mm) * DEG_PER_MM;
// // // // //   };

// // // // //   // Y motion: both motors same direction.
// // // // //   // If physical motion is reversed, swap CW <-> CCW here.
// // // // //   auto move_y_mm = [&](double dy_mm) {
// // // // //     if (std::abs(dy_mm) < MIN_MOVE_MM)
// // // // //       return;

// // // // //     const double deg = mm_to_deg(dy_mm);
// // // // //     const uint8_t dir = (dy_mm > 0.0) ? iSV57T::CW : iSV57T::CCW;

// // // // //     std::thread t1([&]() { m1.rotate(dir, deg); });
// // // // //     std::thread t2([&]() { m2.rotate(dir, deg); });
// // // // //     t1.join();
// // // // //     t2.join();

// // // // //     curr_y_mm += dy_mm;
// // // // //   };

// // // // //   // X motion: motors opposite direction.
// // // // //   // If physical motion is reversed, swap the two cases here.
// // // // //   auto move_x_mm = [&](double dx_mm) {
// // // // //     if (std::abs(dx_mm) < MIN_MOVE_MM)
// // // // //       return;

// // // // //     const double deg = mm_to_deg(dx_mm);

// // // // //     if (dx_mm > 0.0) {
// // // // //       // Positive X
// // // // //       std::thread t1([&]() { m1.rotate(iSV57T::CCW, deg); });
// // // // //       std::thread t2([&]() { m2.rotate(iSV57T::CW, deg); });
// // // // //       t1.join();
// // // // //       t2.join();
// // // // //     } else {
// // // // //       // Negative X
// // // // //       std::thread t1([&]() { m1.rotate(iSV57T::CW, deg); });
// // // // //       std::thread t2([&]() { m2.rotate(iSV57T::CCW, deg); });
// // // // //       t1.join();
// // // // //       t2.join();
// // // // //     }

// // // // //     curr_x_mm += dx_mm;
// // // // //   };

// // // // //   auto move_to_xy_mm = [&](double target_x_mm, double target_y_mm) {
// // // // //     const double dx_mm = target_x_mm - curr_x_mm;
// // // // //     const double dy_mm = target_y_mm - curr_y_mm;

// // // // //     std::cout << "[C++] move request: target=("
// // // // //               << target_x_mm << ", " << target_y_mm
// // // // //               << ") curr=(" << curr_x_mm << ", " << curr_y_mm
// // // // //               << ") delta=(" << dx_mm << ", " << dy_mm << ")\n";

// // // // //     // Simple sequential decomposition.
// // // // //     move_x_mm(dx_mm);
// // // // //     move_y_mm(dy_mm);

// // // // //     std::cout << "[C++] new estimated position=("
// // // // //               << curr_x_mm << ", " << curr_y_mm << ")\n";
// // // // //   };

// // // // //   // gpiod_chip *chip = gpiod_chip_open(gpio_chip_char);
// // // // //   // if (!chip) {
// // // // //   //   fprintf(stderr, "Failed to open %s: %s\n", gpio_chip_char, strerror(errno));
// // // // //   //   std::cerr << "Something bad happened\n";
// // // // //   //   return 0;
// // // // //   // }

// // // // //   // std::cout << "Initializing motor object...\n";

// // // // //   // iSV57T m1 = iSV57T(chip, m1_dir_line, m1_pul_line, pulse_per_rev);
// // // // //   // iSV57T m2 = iSV57T(chip, m2_dir_line, m2_pul_line, pulse_per_rev);

// // // // //   // auto rotate_both = [&](uint8_t dir) {
// // // // //   //   std::thread t1([&]() { m1.rotate(dir, 360); });
// // // // //   //   std::thread t2([&]() { m2.rotate(dir, 360); });
// // // // //   //   t1.join();
// // // // //   //   t2.join();
// // // // //   // };

// // // // //   std::cout << "Finished motor object initialization!\n";

// // // // //   // int shm_fd = shm_open(shm_name, O_RDWR, 0666);
// // // // //   // if (shm_fd < 0) {
// // // // //   //   std::cerr << "Failed to open shared memory " << shm_name << ": "
// // // // //   //             << strerror(errno) << "\n";
// // // // //   //   gpiod_chip_close(chip);
// // // // //   //   return 1;
// // // // //   // }

// // // // //   int shm_fd = -1;
// // // // //   while (shm_fd < 0) {
// // // // //     shm_fd = shm_open(shm_name, O_RDWR, 0666);
// // // // //     if (shm_fd < 0) {
// // // // //       std::cerr << "Waiting for shared memory " << shm_name << ": "
// // // // //                 << strerror(errno) << "\n";
// // // // //       std::this_thread::sleep_for(std::chrono::milliseconds(500));
// // // // //     }
// // // // //   }

// // // // //   void *addr = mmap(nullptr, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED,
// // // // //                   shm_fd, 0);
// // // // //   if (addr == MAP_FAILED) {
// // // // //     std::cerr << "Failed to mmap shared memory: " << strerror(errno) << "\n";
// // // // //     close(shm_fd);
// // // // //     //gpiod_chip_close(chip);
// // // // //     gpiod_chip_close(chip);
// // // // //     return 1;
// // // // //   }

// // // // //   shm_xy *p_shm = static_cast<shm_xy *>(addr);

// // // // //   std::cout << "Waiting for coordinates from Python...\n";

// // // // //   uint32_t last_seq = 0;

// // // // //   while (true) {
// // // // //   // Ask Python for the next coordinate
// // // // //   p_shm->request_next = 1;

// // // // //   uint32_t kind = KIND_NONE;
// // // // //   double x_mm = 0.0;
// // // // //   double y_mm = 0.0;

// // // // //   while (true) {
// // // // //     if (read_stable_message(p_shm, last_seq, kind, x_mm, y_mm)) {
// // // // //       break;
// // // // //     }
// // // // //     std::this_thread::sleep_for(std::chrono::milliseconds(20));
// // // // //   }

// // // // //   std::cout << "[C++] accepted kind=" << kind_to_string(kind)
// // // // //             << " x_mm=" << x_mm
// // // // //             << " y_mm=" << y_mm << std::endl;

// // // // //   if (kind == KIND_RIGHT_SIDE) {
// // // // //     std::cout << "[C++] puck is on opponent side, not moving motors.\n";
// // // // //   } else if (kind == KIND_OUT_OF_RANGE) {
// // // // //     std::cout << "[C++] target y_mm outside robot allowed range, not moving motors.\n";
// // // // //   } else {
// // // // //     move_to_xy_mm(x_mm, y_mm);

// // // // //     // std::cout << "[C++] coordinate kind=" << kind_to_string(kind)
// // // // //     //           << ". Starting motor rotations...\n";
// // // // //     // rotate_both(iSV57T::CW);
// // // // //     // std::this_thread::sleep_for(std::chrono::milliseconds(500));
// // // // //     // rotate_both(iSV57T::CCW);
// // // // //     // std::this_thread::sleep_for(std::chrono::milliseconds(500));
// // // // //     // std::cout << "[C++] Ending motor rotations...\n";
// // // // //   }

// // // // //   // Tell Python we consumed this coordinate and want a fresh one next
// // // // //   p_shm->ready = 0;
// // // // //   p_shm->request_next = 1;
// // // // //   }

// // // // //   munmap(addr, shm_size);
// // // // //   close(shm_fd);
// // // // //   //gpiod_chip_close(chip);
// // // // //   gpiod_chip_close(chip);

// // // // //   return 0;
// // // // // }




// // // #include <arpa/inet.h>

// // // #include <csignal>
// // // #include <cstdint>
// // // #include <netinet/in.h>
// // // #include <stdio.h>
// // // #include <sys/socket.h>
// // // #include <unistd.h>

// // // #include <chrono>
// // // #include <cstring>
// // // #include <iostream>
// // // #include <string>
// // // #include <thread>

// // // #include <fcntl.h>
// // // #include <sys/mman.h>
// // // #include <sys/stat.h>

// // // #include "iSV57T.hpp"
// // // #include "include/iSV57T.hpp"

// // // /** Valid GPIO Usuable Pieces
// // //  * GPIO1 (gpiochip2 MXM3_1 Line 8) Pin 13
// // //  * GPIO2 (gpiochip2 MXM3_3 Line 9) Pin 14
// // //  * GPIO3 (gpiochip2 MXM3_5 Line 12) Pin 15
// // //  * GPIO4 (gpiochip2 MXM3_7 Line 13) Pin 16
// // //  * GPIO5 (gpiochip6 MXM3_11 Line 1) Pin 17
// // //  * GPIO6 (gpiochip6 MXM3_13 Line 2) Pin 18
// // //  */

// // // #pragma pack(push, 1)
// // // struct shm_xy {
// // //   uint32_t seq;
// // //   uint32_t ready;
// // //   uint32_t request_next;
// // //   uint32_t kind;
// // //   double x_mm;
// // //   double y_mm;
// // // };
// // // #pragma pack(pop)

// // // enum coord_kind : uint32_t {
// // //   KIND_NONE = 0,
// // //   KIND_STILL = 1,
// // //   KIND_INTERCEPT = 2,
// // //   KIND_BOUNCE = 3,
// // //   KIND_RIGHT_SIDE = 4,
// // //   KIND_OUT_OF_RANGE = 5
// // // };

// // // const char *kind_to_string(uint32_t kind) {
// // // switch (kind) {
// // //   case KIND_STILL:
// // //     return "STILL";
// // //   case KIND_INTERCEPT:
// // //     return "INTERCEPT";
// // //   case KIND_BOUNCE:
// // //     return "BOUNCE";
// // //   case KIND_RIGHT_SIDE:
// // //     return "RIGHT_SIDE";
// // //   case KIND_OUT_OF_RANGE:
// // //     return "OUT_OF_RANGE";
// // //   default:
// // //     return "NONE";
// // //   }
// // // }

// // // bool read_stable_message(const shm_xy *p, uint32_t &last_seq, uint32_t &kind, double &x_mm, double &y_mm) {
// // //   uint32_t s1 = p->seq;

// // //   if (s1 == 0 || (s1 & 1))
// // //   return false;

// // //   if (!p->ready)
// // //   return false;

// // //   uint32_t k = p->kind;
// // //   double x = p->x_mm;
// // //   double y = p->y_mm;

// // //   uint32_t s2 = p->seq;

// // //   if (s1 != s2 || (s2 & 1))
// // //     return false;

// // //   if (s2 == last_seq)
// // //     return false;

// // //   last_seq = s2;
// // //   kind = k;
// // //   x_mm = x;
// // //   y_mm = y;
// // //   return true;
// // // }

// // // int main() {
// // //   const char *gpio_chip_char = "/dev/gpiochip2";
// // //   const unsigned m1_dir_line = 8;
// // //   const unsigned m1_pul_line = 9;
// // //   const unsigned m2_dir_line = 12;
// // //   const unsigned m2_pul_line = 13;
// // //   const uint16_t pulse_per_rev = 2000;

// // //   const char *shm_name = "/puck_xy_mm";
// // //   const size_t shm_size = sizeof(shm_xy);

// // //   // gpiod_chip *chip = gpiod_chip_open(gpio_chip_char);
// // //   // if (!chip) {
// // //   //   fprintf(stderr, "Failed to open %s: %s\n", gpio_chip_char, strerror(errno));
// // //   //   std::cerr << "Something bad happened\n";
// // //   //   return 0;
// // //   // }

// // //   // std::cout << "Initializing motor object...\n";

// // //   // iSV57T m1 = iSV57T(chip, m1_dir_line, m1_pul_line, pulse_per_rev);
// // //   // iSV57T m2 = iSV57T(chip, m2_dir_line, m2_pul_line, pulse_per_rev);

// // //   // auto rotate_both = [&](uint8_t dir) {
// // //   //   std::thread t1([&]() { m1.rotate(dir, 360); });
// // //   //   std::thread t2([&]() { m2.rotate(dir, 360); });
// // //   //   t1.join();
// // //   //   t2.join();
// // //   // };

// // //   std::cout << "Finished motor object initialization!\n";

// // //   // int shm_fd = shm_open(shm_name, O_RDWR, 0666);
// // //   // if (shm_fd < 0) {
// // //   //   std::cerr << "Failed to open shared memory " << shm_name << ": "
// // //   //             << strerror(errno) << "\n";
// // //   //   gpiod_chip_close(chip);
// // //   //   return 1;
// // //   // }

// // //   int shm_fd = -1;
// // //   while (shm_fd < 0) {
// // //     shm_fd = shm_open(shm_name, O_RDWR, 0666);
// // //     if (shm_fd < 0) {
// // //       std::cerr << "Waiting for shared memory " << shm_name << ": "
// // //                 << strerror(errno) << "\n";
// // //       std::this_thread::sleep_for(std::chrono::milliseconds(500));
// // //     }
// // //   }

// // //   void *addr = mmap(nullptr, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED,
// // //                   shm_fd, 0);
// // //   if (addr == MAP_FAILED) {
// // //     std::cerr << "Failed to mmap shared memory: " << strerror(errno) << "\n";
// // //     close(shm_fd);
// // //     //gpiod_chip_close(chip);
// // //     return 1;
// // //   }

// // //   shm_xy *p_shm = static_cast<shm_xy *>(addr);

// // //   std::cout << "Waiting for coordinates from Python...\n";

// // //   uint32_t last_seq = 0;

// // //   while (true) {
// // //   // Ask Python for the next coordinate
// // //   p_shm->request_next = 1;

// // //   uint32_t kind = KIND_NONE;
// // //   double x_mm = 0.0;
// // //   double y_mm = 0.0;

// // //   while (true) {
// // //     if (read_stable_message(p_shm, last_seq, kind, x_mm, y_mm)) {
// // //       break;
// // //     }
// // //     std::this_thread::sleep_for(std::chrono::milliseconds(20));
// // //   }

// // //   std::cout << "[C++] accepted kind=" << kind_to_string(kind)
// // //             << " x_mm=" << x_mm
// // //             << " y_mm=" << y_mm << std::endl;

// // //   if (kind == KIND_RIGHT_SIDE) {
// // //     std::cout << "[C++] puck is on opponent side, not moving motors.\n";
// // //   } else if (kind == KIND_OUT_OF_RANGE) {
// // //     std::cout << "[C++] target y_mm outside robot allowed range, not moving motors.\n";
// // //   } else {
// // //     // std::cout << "[C++] coordinate kind=" << kind_to_string(kind)
// // //     //           << ". Starting motor rotations...\n";
// // //     // rotate_both(iSV57T::CW);
// // //     // std::this_thread::sleep_for(std::chrono::milliseconds(500));
// // //     // rotate_both(iSV57T::CCW);
// // //     // std::this_thread::sleep_for(std::chrono::milliseconds(500));
// // //     // std::cout << "[C++] Ending motor rotations...\n";
// // //   }

// // //   // Tell Python we consumed this coordinate and want a fresh one next
// // //   p_shm->ready = 0;
// // //   p_shm->request_next = 1;
// // //   }

// // //   munmap(addr, shm_size);
// // //   close(shm_fd);
// // //   //gpiod_chip_close(chip);

// // //   return 0;
// // // }

// // // // // // // // #include <arpa/inet.h>

// // // // // // // // #include <csignal>
// // // // // // // // #include <cstdint>
// // // // // // // // #include <netinet/in.h>
// // // // // // // // #include <stdio.h>
// // // // // // // // #include <sys/socket.h>
// // // // // // // // #include <unistd.h>

// // // // // // // // #include <chrono>
// // // // // // // // #include <cstring>
// // // // // // // // #include <iostream>
// // // // // // // // #include <string>
// // // // // // // // #include <thread>

// // // // // // // // #include "iSV57T.hpp"
// // // // // // // // #include "include/iSV57T.hpp"

// // // // // // // // /** Valid GPIO Usuable Pieces
// // // // // // // //  * GPIO1 (gpiochip2 MXM3_1 Line 8) Pin 13
// // // // // // // //  * GPIO2 (gpiochip2 MXM3_3 Line 9) Pin 14
// // // // // // // //  * GPIO3 (gpiochip2 MXM3_5 Line 12) Pin 15
// // // // // // // //  * GPIO4 (gpiochip2 MXM3_7 Line 13) Pin 16
// // // // // // // //  * GPIO5 (gpiochip6 MXM3_11 Line 1) Pin 17
// // // // // // // //  * GPIO6 (gpiochip6 MXM3_13 Line 2) Pin 18
// // // // // // // //  */

// // // // // // // // int main() {
// // // // // // // //   const char *gpio_chip_char = "/dev/gpiochip2";
// // // // // // // //   const unsigned m1_dir_line = 8;
// // // // // // // //   const unsigned m1_pul_line = 9;
// // // // // // // //   const unsigned m2_dir_line = 12;
// // // // // // // //   const unsigned m2_pul_line = 13;
// // // // // // // //   const uint16_t pulse_per_rev = 2000;

// // // // // // // //   gpiod_chip *chip = gpiod_chip_open(gpio_chip_char);
// // // // // // // //   if (!chip) {
// // // // // // // //     fprintf(stderr, "Failed to open %s: %s\n", gpio_chip_char, strerror(errno));
// // // // // // // //     std::cerr << "Something bad happened\n";
// // // // // // // //     return 0;
// // // // // // // //   }

// // // // // // // //   std::cout << "Initializing motor object...\n";

// // // // // // // //   iSV57T m1 = iSV57T(chip, m1_dir_line, m1_pul_line, pulse_per_rev);
// // // // // // // //   iSV57T m2 = iSV57T(chip, m2_dir_line, m2_pul_line, pulse_per_rev);

// // // // // // // //   auto rotate_both = [&](uint8_t dir) {
// // // // // // // //     std::thread t1([&]() { m1.rotate(dir, 360); });
// // // // // // // //     std::thread t2([&]() { m2.rotate(dir, 360); });
// // // // // // // //     t1.join();
// // // // // // // //     t2.join();
// // // // // // // //   };

// // // // // // // //   std::cout << "Finished motor object initialization!\n";

// // // // // // // //   std::cout << "Starting motor rotations...\n";
// // // // // // // //   rotate_both(iSV57T::CW);
// // // // // // // //   std::this_thread::sleep_for(std::chrono::milliseconds(500));
// // // // // // // //   rotate_both(iSV57T::CCW);
// // // // // // // //   std::this_thread::sleep_for(std::chrono::milliseconds(500));
// // // // // // // //   std::cout << "Ending motor rotations...\n";

// // // // // // // //   gpiod_chip_close(chip);

// // // // // // // //   return 0;
// // // // // // // // }