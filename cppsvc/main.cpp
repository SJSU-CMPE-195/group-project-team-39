// main.cpp - gantry side of the Python <-> C++ shared-memory loop.
//
// What this version does:
//   - Reads the 64-byte mm-based struct produced by ipc.py (wire v2)
//   - Only moves the X axis; Y is locked to a defensive line
//   - DEFEND   -> move X to (mallet_home + x_mm)
//   - IGNORE   -> recenter to rest X
//   - NO_TRACK -> recenter to rest X
//
// Coordinate convention:
//   Python publishes mallet-relative millimeters (x_mm = 0 means the puck is
//   directly above the mallet's resting position).  The mallet's physical
//   position in the gantry's own frame (measured from the X/Y limit switches)
//   is MALLET_HOME_X_MM.  So:
//        gantry_target_x_mm = MALLET_HOME_X_MM + x_mm
//
// Calibrate MALLET_HOME_X_MM once: home the gantry, jog it until the
// physical mallet sits exactly under what the camera reports as (0, 0) mm,
// and read g.curr_x.

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdio.h>
#include <string>
#include <thread>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <gpiod.h>

#include "gantry.hpp"
#include "iSV57T.hpp"
#include "limitSwitch.hpp"

// ----------------------------------------------------------------------------
// PHYSICAL CONSTANTS - set these for your rig
// ----------------------------------------------------------------------------

// X position (mm, gantry frame) where the mallet sits when Python reports x_mm = 0.
// MEASURE THIS ONCE on the real hardware and hardcode it.
static constexpr int MALLET_HOME_X_MM = 400;   // <-- change to your value

// Y position (mm, gantry frame) the mallet stays at the whole game.
// This is your "defensive line."
static constexpr int DEFENSIVE_Y_MM   = 100;   // <-- change to your value

// Where to park when there's no puck to defend (typically center of X travel).
static constexpr int REST_X_MM        = MALLET_HOME_X_MM;

// ----------------------------------------------------------------------------
// WIRE FORMAT - must match ipc.py exactly
// ----------------------------------------------------------------------------

#pragma pack(push, 1)
struct shm_xy {
  uint32_t seq;            //  0
  uint32_t ready;          //  4
  uint32_t request_next;   //  8
  uint32_t kind;           // 12
  uint64_t timestamp_ns;   // 16
  double   x_mm;           // 24
  double   y_mm;           // 32
  double   vx_mm_s;        // 40
  double   vy_mm_s;        // 48
  float    confidence;     // 56
  uint32_t version;        // 60
};
#pragma pack(pop)
static_assert(sizeof(shm_xy) == 64, "shm_xy layout must match ipc.py");

enum coord_kind : uint32_t {
  KIND_NONE     = 0,
  KIND_DEFEND   = 1,
  KIND_IGNORE   = 2,
  KIND_NO_TRACK = 3,
};

static constexpr uint32_t WIRE_VERSION = 2;

// ----------------------------------------------------------------------------
// Globals
// ----------------------------------------------------------------------------

std::atomic<bool> keep_running(true);
void handle_signal(int) { keep_running = false; }

// Seq-lock read.  Returns true only when (a) seq is even (stable), (b) ready
// is set, (c) seq didn't change mid-read, and (d) it's a new sequence number
// we haven't seen.
static bool read_stable(const shm_xy *p, uint32_t &last_seq,
                        uint32_t &kind_out, double &x_mm_out, double &y_mm_out,
                        double &vx_out, double &vy_out) {
  uint32_t s1 = p->seq;
  if (s1 == 0 || (s1 & 1u)) return false;
  if (!p->ready) return false;

  uint32_t k  = p->kind;
  double x    = p->x_mm;
  double y    = p->y_mm;
  double vx   = p->vx_mm_s;
  double vy   = p->vy_mm_s;

  uint32_t s2 = p->seq;
  if (s1 != s2 || (s2 & 1u)) return false;
  if (s2 == last_seq) return false;

  last_seq  = s2;
  kind_out  = k;
  x_mm_out  = x;
  y_mm_out  = y;
  vx_out    = vx;
  vy_out    = vy;
  return true;
}

// Convert mallet-relative x_mm to gantry-frame X in mm, clamped to travel.
// (No unit scaling -- both sides speak mm now.)
static int mallet_x_to_gantry_x(double x_mm) {
  long r = std::lround(static_cast<double>(MALLET_HOME_X_MM) + x_mm);
  if (r < 0)                   r = 0;
  if (r > GANTRY_X_MAX_LENGTH) r = GANTRY_X_MAX_LENGTH;
  return static_cast<int>(r);
}

// ----------------------------------------------------------------------------
// main
// ----------------------------------------------------------------------------

int main() {
  std::signal(SIGINT,  handle_signal);
  std::signal(SIGTERM, handle_signal);

  const char *gpio_chip4_char = "/dev/gpiochip4"; // For the limit switch GPIO
  const char *gpio_chip0_char = "/dev/gpiochip0"; // For the motors GPIO

  const unsigned m1_dir_line = 8;  // Physical Pin 13
  const unsigned m1_pul_line = 9;  // Physical Pin 14

  const unsigned m2_dir_line = 12; // Physical Pin 15
  const unsigned m2_pul_line = 13; // Physical Pin 16

  const unsigned sw1_line = 1;     // Y limit, Physical Pin 17
  const unsigned sw2_line = 2;     // X limit, Physical Pin 18

  const uint16_t pulse_per_rev = 2000;

  const char  *shm_name = "/puck_xy_mm";
  const size_t shm_size = sizeof(shm_xy);

  gpiod_chip *chip0 = gpiod_chip_open(gpio_chip0_char);
  if (!chip0) {
    fprintf(stderr, "Failed to open %s for motor pins: %s\n",
            gpio_chip0_char, strerror(errno));
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

  // Park on the defensive line.  After this, every move keeps Y at
  // DEFENSIVE_Y_MM, so only the X motor physically rotates.
  try {
    g.move_to_coord(REST_X_MM, DEFENSIVE_Y_MM);
  } catch (const std::exception &e) {
    std::cerr << "Initial park to defensive line failed: " << e.what() << "\n";
    gpiod_chip_close(chip4);
    gpiod_chip_close(chip0);
    return 1;
  }

  // -------- Attach to shared memory ----------------------------------------
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

  if (p_shm->version != 0 && p_shm->version != WIRE_VERSION) {
    std::cerr << "shm wire version mismatch: got " << p_shm->version
              << " expected " << WIRE_VERSION
              << ".  Did you forget to delete /dev/shm" << shm_name
              << " after a layout change?\n";
    munmap(addr, shm_size);
    close(shm_fd);
    gpiod_chip_close(chip4);
    gpiod_chip_close(chip0);
    return 1;
  }

  std::cout << "[C++] attached to " << shm_name
            << " (wire v" << WIRE_VERSION << ", units=mm), "
            << "waiting for coordinates from Python...\n";

  // -------- Main loop ------------------------------------------------------
  uint32_t last_seq = 0;

  while (keep_running) {
    p_shm->request_next = 1;

    uint32_t kind = KIND_NONE;
    double   x_mm = 0.0, y_mm = 0.0, vx = 0.0, vy = 0.0;

    // Tight poll for a new stable message.
    while (keep_running) {
      if (read_stable(p_shm, last_seq, kind, x_mm, y_mm, vx, vy)) break;
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }

    if (!keep_running) break;

    try {
      switch (kind) {
        case KIND_DEFEND: {
          int target_x = mallet_x_to_gantry_x(x_mm);
          // Y stays at DEFENSIVE_Y_MM, so this is effectively a pure X move.
          g.move_to_coord(static_cast<unsigned>(target_x),
                          static_cast<unsigned>(DEFENSIVE_Y_MM));
          std::cout << "[C++] DEFEND  x_mm=" << x_mm
                    << " -> gantry_x=" << target_x << " mm\n";
          break;
        }
        case KIND_IGNORE:
        case KIND_NO_TRACK:
          if (g.curr_x != REST_X_MM) {
            g.move_to_coord(static_cast<unsigned>(REST_X_MM),
                            static_cast<unsigned>(DEFENSIVE_Y_MM));
            std::cout << "[C++] "
                      << (kind == KIND_IGNORE ? "IGNORE" : "NO_TRACK")
                      << " -> rest\n";
          }
          break;
        default:
          break;
      }
    } catch (const std::exception &e) {
      std::cerr << "[C++] move failed: " << e.what() << "\n";
    }

    p_shm->ready = 0;  // we consumed it
  }

  munmap(addr, shm_size);
  close(shm_fd);
  gpiod_chip_close(chip4);
  gpiod_chip_close(chip0);

  return 0;
}