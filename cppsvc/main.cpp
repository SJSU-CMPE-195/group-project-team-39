#include <arpa/inet.h>

#include <atomic>
#include <csignal>
#include <cstdint>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include <gpiod.h>

#include "gantry.hpp"
#include "iSV57T.hpp"
#include "include/gantry.hpp"
#include "include/iSV57T.hpp"
#include "limitSwitch.hpp"

std::atomic<bool> keep_running(true);

void handle_signal(int) {
  keep_running = false;
}

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
  KIND_TOP_SIDE = 3,
  KIND_NO_TRACK = 4
};

const char *kind_to_string(uint32_t kind) {
  switch (kind) {
    case KIND_STILL:
      return "STILL";
    case KIND_INTERCEPT:
      return "INTERCEPT";
    case KIND_TOP_SIDE:
      return "TOP_SIDE";
    case KIND_NO_TRACK:
      return "NO_TRACK";
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

  const char *gpio_chip4_char = "/dev/gpiochip4"; // For limit switch GPIO
  const char *gpio_chip0_char = "/dev/gpiochip0"; // For motor GPIO

  const unsigned m1_dir_line = 8;  // Physical Pin 13
  const unsigned m1_pul_line = 9;  // Physical Pin 14

  const unsigned m2_dir_line = 12; // Physical Pin 15
  const unsigned m2_pul_line = 13; // Physical Pin 16

  const unsigned sw1_line = 1; // Y limit, Physical Pin 17
  const unsigned sw2_line = 2; // X limit, Physical Pin 18

  const uint16_t pulse_per_rev = 2000;

  const char *shm_name = "/puck_xy_mm";
  const size_t shm_size = sizeof(shm_xy);

  gpiod_chip *chip0 = gpiod_chip_open(gpio_chip0_char);
  if (!chip0) {
    fprintf(stderr, "Failed to open %s for motor pins: %s\n", gpio_chip0_char,
            strerror(errno));
    return 1;
  }

  gpiod_chip *chip4 = gpiod_chip_open(gpio_chip4_char);
  if (!chip4) {
    fprintf(stderr, "Failed to open %s for limit switches: %s\n",
            gpio_chip4_char, strerror(errno));
    gpiod_chip_close(chip0);
    return 1;
  }

  limitSwitch sw_y(chip4, sw1_line);
  limitSwitch sw_x(chip4, sw2_line);

  iSV57T m_lower = iSV57T(chip0, m1_dir_line, m1_pul_line, pulse_per_rev);
  iSV57T m_upper = iSV57T(chip0, m2_dir_line, m2_pul_line, pulse_per_rev);

  try {
    m_lower.set_target_rpm(700);
    m_upper.set_target_rpm(700);
  } catch (const std::exception &e) {
    std::cerr << "Failed to set target RPM: " << e.what() << "\n";
    gpiod_chip_close(chip4);
    gpiod_chip_close(chip0);
    return 1;
  }

  gantry g = gantry(m_lower, m_upper, sw_x, sw_y);
  g.curr_x = 0;
  g.curr_y = 0;

  // g.calibration_test();

  g.move_to_rest_point();

  // try {
  //   g.move_to_origin();
  // } catch (const std::exception &e) {
  //   std::cerr << "[C++] Homing failed: " << e.what() << "\n";
  //   gpiod_chip_close(chip4);
  //   gpiod_chip_close(chip0);
  //   return 1;
  // }

  int shm_fd = -1;

  while (keep_running && shm_fd < 0) {
    shm_fd = shm_open(shm_name, O_RDWR, 0666);

    if (shm_fd < 0) {
      std::cerr << "Waiting for shared memory " << shm_name << ": "
                << strerror(errno) << "\n";

      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }

  if (!keep_running) {
    gpiod_chip_close(chip4);
    gpiod_chip_close(chip0);
    return 0;
  }

  void *addr = mmap(nullptr, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED,
                    shm_fd, 0);

  if (addr == MAP_FAILED) {
    std::cerr << "Failed to mmap shared memory: " << strerror(errno) << "\n";
    close(shm_fd);
    gpiod_chip_close(chip4);
    gpiod_chip_close(chip0);
    return 1;
  }

  shm_xy *p_shm = static_cast<shm_xy *>(addr);

  std::cout << "Waiting for coordinates from Python...\n";

  uint32_t last_seq = 0;

  const int rest_x = GANTRY_X_MAX_LENGTH / 2;
  const int rest_y = 0;

  uint32_t last_kind = KIND_NONE;

  /*
   * This time is only used when the last received coordinate was KIND_STILL.
   * If Python does not publish a new coordinate for 2 seconds after STILL,
   * C++ moves back to rest before asking for a new coordinate.
   */
  auto last_still_time = std::chrono::steady_clock::now();

  const int STILL_TIMEOUT_MS = 2000;

  while (keep_running) {
    p_shm->request_next = 1;

    uint32_t kind = KIND_NONE;
    double x_mm = 0.0;
    double y_mm = 0.0;

    bool got_new_message = false;
    bool still_timeout = false;

    while (keep_running) {
      if (read_stable_message(p_shm, last_seq, kind, x_mm, y_mm)) {
        got_new_message = true;
        last_kind = kind;

        if (kind == KIND_STILL) {
          last_still_time = std::chrono::steady_clock::now();
        }

        break;
      }

      /*
       * Only check elapsed time if the last valid coordinate was STILL.
       * INTERCEPT, TOP_SIDE, and NO_TRACK do not use this timer.
       */
      if (last_kind == KIND_STILL) {
        auto now = std::chrono::steady_clock::now();

        auto elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_still_time
            ).count();

        if (elapsed_ms >= STILL_TIMEOUT_MS) {
          still_timeout = true;
          break;
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    if (!keep_running)
      break;

    if (still_timeout) {
      /*
       * IMPORTANT:
       * Move back to rest first. Only after that do we clear the old STILL
       * packet and request a new coordinate.
       */
      try {
        std::cout << "[C++] STILL coordinate stale for 2 seconds. "
                  << "Moving to rest before requesting a new coordinate.\n";
        if (g.curr_x != rest_x || g.curr_y != rest_y) {
            g.move_to_rest_point();
          }
      } catch (const std::exception &e) {
        std::cerr << "[C++] move to rest point failed after STILL timeout: "
                  << e.what() << "\n";
      }

      /*
       * Clear the old STILL state so C++ does not keep timing out on the same
       * stale packet.
       */
      last_kind = KIND_NONE;

      /*
       * Now request a fresh coordinate from Python.
       */
      p_shm->ready = 0;
      p_shm->request_next = 1;

      continue;
    }

    if (!got_new_message) {
      continue;
    }

    std::cout << "[C++] Received kind=" << kind_to_string(kind)
              << " x_mm=" << x_mm
              << " y_mm=" << y_mm << "\n";

    if (kind == KIND_TOP_SIDE || kind == KIND_NO_TRACK) {
      try {
        // Only move to rest if we are not already there.
        if (g.curr_x != rest_x || g.curr_y != rest_y) {
          g.move_to_rest_point();
        }
      } catch (const std::exception &e) {
        std::cerr << "[C++] move to rest point failed: " << e.what() << "\n";
      }
    } else if (kind == KIND_STILL || kind == KIND_INTERCEPT) {
      int target_x = static_cast<int>(std::lround(x_mm));
      int target_y = static_cast<int>(std::lround(y_mm));

      if (target_x < 0 || target_x > GANTRY_X_MAX_LENGTH ||
          target_y < 0 || target_y > GANTRY_Y_MAX_LENGTH) {
        // If out of boundary, move back to rest.
        try {
          if (g.curr_x != rest_x || g.curr_y != rest_y) {
            g.move_to_rest_point();
          }
        } catch (const std::exception &e) {
          std::cerr << "[C++] move to rest point failed after boundary check: "
                    << e.what() << "\n";
        }
      } else {
        try {
          g.move_to_coord(
              static_cast<unsigned>(target_x),
              static_cast<unsigned>(target_y)
          );
        } catch (const std::exception &e) {
          std::cerr << "[C++] move failed: " << e.what() << "\n";
        }
      }
    } else {
      // KIND_NONE or unknown kind: do nothing.
    }

    p_shm->ready = 0;
    p_shm->request_next = 1;
  }

  munmap(addr, shm_size);
  close(shm_fd);
  gpiod_chip_close(chip4);
  gpiod_chip_close(chip0);

  return 0;
}


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

// // #include "gantry.hpp"
// // #include "iSV57T.hpp"
// // #include "include/gantry.hpp"
// // #include "include/iSV57T.hpp"
// // #include "limitSwitch.hpp"

// // int main() {
// //   const char *gpio_chip4_char = "/dev/gpiochip4"; // For the limit switch GPIO
// //   const char *gpio_chip0_char = "/dev/gpiochip0"; // For the motors GPIO

// //   // m1
// //   const unsigned m1_dir_line = 8; // Physical Pin 13
// //   const unsigned m1_pul_line = 9; // Physical Pin 14

// //   // m2
// //   const unsigned m2_dir_line = 12; // Physical Pin 15
// //   const unsigned m2_pul_line = 13; // Physical Pin 16

// //   // limit switch
// //   const unsigned sw1_line = 1; // Physical Pin 17
// //   const unsigned sw2_line = 2; // Physical Pin 18

// //   const uint16_t pulse_per_rev = 2000;

// //   gpiod_chip *chip0 = gpiod_chip_open(gpio_chip0_char);
// //   if (!chip0) {
// //     fprintf(stderr, "Failed to open %s for motor pins: %s\n", gpio_chip0_char,
// //             strerror(errno));
// //     gpiod_chip_close(chip0);
// //     return 0;
// //   }

// //   gpiod_chip *chip4 = gpiod_chip_open(gpio_chip4_char);
// //   if (!chip4) {
// //     fprintf(stderr, "Failed to open %s for limit switches: %s\n",
// //             gpio_chip4_char, strerror(errno));
// //     gpiod_chip_close(chip4);
// //     return 0;
// //   }

// //   std::cout << "Initializing limit switch objects...\n";

// //   limitSwitch sw_y(chip4, sw1_line);
// //   limitSwitch sw_x(chip4, sw2_line);

// //   std::cout << "Finished limit switch object initialization!\n";

// //   // Testing the limit switch for a reading
// //   // std::thread sw1_thread([&]() {
// //   //   while (sw1.read() != 1) {
// //   //   }
// //   //   std::cout << "Limit switch 1 (Y-axis) triggered!\n";
// //   // });

// //   // std::thread sw2_thread([&]() {
// //   //   while (sw2.read() != 1) {
// //   //   }
// //   //   std::cout << "Limit switch 2 (X-axis) triggered!\n";
// //   // });

// //   std::cout << "Initializing motor object...\n";

// //   iSV57T m_lower = iSV57T(chip0, m1_dir_line, m1_pul_line, pulse_per_rev);
// //   iSV57T m_upper = iSV57T(chip0, m2_dir_line, m2_pul_line, pulse_per_rev);

// //   // Setting RPM Test
// //   m_lower.set_target_rpm(500);
// //   m_upper.set_target_rpm(500);

// //   std::cout << "Finished motor object initialization!\n";

// //   // Gantry Object Initialization
// //   std::cout << "Initializing gantry object...\n";

// //   gantry g = gantry(m_lower, m_upper, sw_x, sw_y);

// //   g.curr_x = 0;
// //   g.curr_y = 0;

// //   std::cout << "Finished gantry object initialization!\n";

// //   std::cout << "Starting motor rotations...\n";

// //   // Make the rotate_motor function public when testing.
// //   // Moving North
// //    g.rotate_motors(1440, iSV57T::CCW, iSV57T::CW, gantry::MotorSelect::BOTH);

// //   // Moving South
// //    g.rotate_motors(1440, iSV57T::CW, iSV57T::CCW, gantry::MotorSelect::BOTH);

// //   // Moving East
// //    g.rotate_motors(1440, iSV57T::CCW, iSV57T::CCW, gantry::MotorSelect::BOTH);

// //   // Moving West
// //    g.rotate_motors(1440, iSV57T::CW, iSV57T::CW, gantry::MotorSelect::BOTH);

// //   // Moving Diagonally
// //   // Without Limit Switches
// //   // g.curr_x = 200;
// //   // g.curr_y = 200;
// //   // g.move_diagonal(100, 1);
// //   // std::this_thread::sleep_for(std::chrono::milliseconds(500));
// //   // g.move_diagonal(100, 0);
// //   // g.move_diagonal(100, 2);
// //   // std::this_thread::sleep_for(std::chrono::milliseconds(500));
// //   // g.move_diagonal(100, 3);

// //   // Move to Origin Test
// //   // std::cout << "Moving to origin...\n";

// //   // g.move_to_origin();

// //   // std::cout << "Finished moving to origin!\n";

// //   // Calibration Function
// //   // std::cout << "Calibration testing...\n";
// //   // // Without Limit Switches
// //   // g.curr_x = 0;
// //   // g.curr_y = 0;

// //   // g.calibration_test();

// //   // std::cout << "Calibration Test Complete!\n";

// //   // Delay line
// //   // std::this_thread::sleep_for(std::chrono::milliseconds(500));

// //   std::cout << "Ending motor rotations...\n";

// //   // sw1_thread.join();
// //   // sw2_thread.join();

// //   gpiod_chip_close(chip4);
// //   gpiod_chip_close(chip0);

// //   return 0;
// // }




// #include <arpa/inet.h>

// #include <atomic>
// #include <csignal>
// #include <cstdint>
// #include <netinet/in.h>
// #include <stdio.h>
// #include <sys/socket.h>
// #include <unistd.h>

// #include <chrono>
// #include <cmath>
// #include <cstring>
// #include <iostream>
// #include <string>
// #include <thread>

// #include <fcntl.h>
// #include <sys/mman.h>
// #include <sys/stat.h>

// #include <gpiod.h>

// #include "gantry.hpp"
// #include "iSV57T.hpp"
// #include "include/gantry.hpp"
// #include "include/iSV57T.hpp"
// #include "limitSwitch.hpp"

// std::atomic<bool> keep_running(true);

// void handle_signal(int) {
//   keep_running = false;
// }

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
//   KIND_TOP_SIDE = 3,
//   KIND_NO_TRACK = 4
// };

// const char *kind_to_string(uint32_t kind) {
//   switch (kind) {
//     case KIND_STILL:
//       return "STILL";
//     case KIND_INTERCEPT:
//       return "INTERCEPT";
//     case KIND_TOP_SIDE:
//       return "TOP_SIDE";
//     case KIND_NO_TRACK:
//       return "NO_TRACK";
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
//   std::signal(SIGINT, handle_signal);
//   std::signal(SIGTERM, handle_signal);

//   const char *gpio_chip4_char = "/dev/gpiochip4"; //For the limit switch GPIO
//   const char *gpio_chip0_char = "/dev/gpiochip0"; //For the motors GPIO

//   const unsigned m1_dir_line = 8; // Physical Pin 13
//   const unsigned m1_pul_line = 9; // Physical Pin 14

//   const unsigned m2_dir_line = 12; // Physical Pin 15
//   const unsigned m2_pul_line = 13; // Physical Pin 16

//   const unsigned sw1_line = 1; // Y limit, Physical Pin 17
//   const unsigned sw2_line = 2; // X limit, // Physical Pin 18

//   const uint16_t pulse_per_rev = 2000;

//   const char *shm_name = "/puck_xy_mm";
//   const size_t shm_size = sizeof(shm_xy);

//   gpiod_chip *chip0 = gpiod_chip_open(gpio_chip0_char);
//   if (!chip0) {
//     fprintf(stderr, "Failed to open %s for motor pins: %s\n", gpio_chip0_char,
//             strerror(errno));
//     return 1;
//   }

//   gpiod_chip *chip4 = gpiod_chip_open(gpio_chip4_char);
//   if (!chip4) {
//     fprintf(stderr, "Failed to open %s for limit switches: %s\n",
//             gpio_chip4_char, strerror(errno));
//     gpiod_chip_close(chip0);
//     return 1;
//   }

//   limitSwitch sw_y(chip4, sw1_line);
//   limitSwitch sw_x(chip4, sw2_line);

//   iSV57T m_lower = iSV57T(chip0, m1_dir_line, m1_pul_line, pulse_per_rev);
//   iSV57T m_upper = iSV57T(chip0, m2_dir_line, m2_pul_line, pulse_per_rev);

//   try {
//     m_lower.set_target_rpm(1000);
//     m_upper.set_target_rpm(1000);
//   } catch (const std::exception &e) {
//     std::cerr << "Failed to set target RPM: " << e.what() << "\n";
//     gpiod_chip_close(chip4);
//     gpiod_chip_close(chip0);
//     return 1;
//   }

//   gantry g = gantry(m_lower, m_upper, sw_x, sw_y);
//   g.curr_x = 0;
//   g.curr_y = 0;

//   //g.calibration_test();

//   g.move_to_rest_point();

//   // try {
//   //   g.move_to_origin();
//   // } catch (const std::exception &e) {
//   //   std::cerr << "[C++] Homing failed: " << e.what() << "\n";
//   //   gpiod_chip_close(chip4);
//   //   gpiod_chip_close(chip0);
//   //   return 1;
//   // }

//   int shm_fd = -1;
//   while (keep_running && shm_fd < 0) {
//     shm_fd = shm_open(shm_name, O_RDWR, 0666);
//     if (shm_fd < 0) {
//       std::cerr << "Waiting for shared memory " << shm_name << ": "
//                 << strerror(errno) << "\n";
//       std::this_thread::sleep_for(std::chrono::milliseconds(500));
//     }
//   }

//   if (!keep_running) {
//     gpiod_chip_close(chip4);
//     gpiod_chip_close(chip0);
//     return 0;
//   }

//   void *addr = mmap(nullptr, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED,
//                     shm_fd, 0);
//   if (addr == MAP_FAILED) {
//     std::cerr << "Failed to mmap shared memory: " << strerror(errno) << "\n";
//     close(shm_fd);
//     gpiod_chip_close(chip4);
//     gpiod_chip_close(chip0);
//     return 1;
//   }

//   shm_xy *p_shm = static_cast<shm_xy *>(addr);

//   std::cout << "Waiting for coordinates from Python...\n";

//   uint32_t last_seq = 0;

//   const int rest_x = GANTRY_X_MAX_LENGTH / 2;
//   const int rest_y = 0;

//   while (keep_running) {
//     p_shm->request_next = 1;

//     uint32_t kind = KIND_NONE;
//     double x_mm = 0.0;
//     double y_mm = 0.0;

//     while (keep_running) {
//       if (read_stable_message(p_shm, last_seq, kind, x_mm, y_mm)) {
//         break;
//       }
//       std::this_thread::sleep_for(std::chrono::milliseconds(20));
//     }

//     if (!keep_running)
//       break;

//     if (kind == KIND_TOP_SIDE || kind == KIND_NO_TRACK) {
//       try {
//         // Only move to rest if we are not already there.
//         if (g.curr_x != rest_x || g.curr_y != rest_y) {
//           g.move_to_rest_point();
//         }
//       } catch (const std::exception &e) {
//         std::cerr << "[C++] move to rest point failed: " << e.what() << "\n";
//       }
//     } else if (kind == KIND_STILL || kind == KIND_INTERCEPT) {
//       int target_x = static_cast<int>(std::lround(x_mm));
//       int target_y = static_cast<int>(std::lround(y_mm));

//       if (target_x < 0 || target_x > GANTRY_X_MAX_LENGTH ||
//           target_y < 0 || target_y > GANTRY_Y_MAX_LENGTH) {
//             //If out of boundary, then move back to rest.
//             if (g.curr_x != rest_x || g.curr_y != rest_y) {
//               g.move_to_rest_point();
//           }
//       } else {
//         try {
//           g.move_to_coord(static_cast<unsigned>(target_x), static_cast<unsigned>(target_y));
//           if (target_x < 0 || target_x > GANTRY_X_MAX_LENGTH || target_y < 0 || target_y > GANTRY_Y_MAX_LENGTH) {
//             g.move_to_rest_point();
//           }
//         } catch (const std::exception &e) {
//           std::cerr << "[C++] move failed: " << e.what() << "\n";
//         }
//       }
//     } else {
//     }

//     p_shm->ready = 0;
//     p_shm->request_next = 1;
//   }

//   munmap(addr, shm_size);
//   close(shm_fd);
//   gpiod_chip_close(chip4);
//   gpiod_chip_close(chip0);

//   return 0;
// }