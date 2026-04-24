#include <arpa/inet.h>

#include <csignal>
#include <cstdint>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <netinet/in.h>
#include <poll.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include "gantry.hpp"
#include "iSV57T.hpp"
#include "limitSwitch.hpp"

namespace {

std::atomic<bool> g_running{true};

void on_sigint(int /*signo*/) {
  // async-signal-safe: just flip the flag; the main loop wakes via poll
  // timeout and drains.
  g_running.store(false, std::memory_order_relaxed);
}

// Test whether a specific key bit is set in the keybits array returned by
// EVIOCGBIT(EV_KEY, ...).
bool key_bit_set(const unsigned long *keybits, int key) {
  constexpr int bits_per_long = 8 * sizeof(unsigned long);
  return (keybits[key / bits_per_long] >> (key % bits_per_long)) & 1UL;
}

// Probe an evdev device and return true if it looks like a keyboard that
// supports the WASD keys.
bool evdev_supports_wasd(int fd) {
  constexpr int bits_per_long = 8 * sizeof(unsigned long);
  constexpr int nlongs = (KEY_MAX + bits_per_long) / bits_per_long;
  unsigned long keybits[nlongs] = {0};
  if (ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(keybits)), keybits) < 0) {
    return false;
  }
  return key_bit_set(keybits, KEY_W) && key_bit_set(keybits, KEY_A) &&
         key_bit_set(keybits, KEY_S) && key_bit_set(keybits, KEY_D);
}

// Find and open a USB keyboard on /dev/input. Prefers the stable
// /dev/input/by-id/*-event-kbd symlink (udev-provided) and falls back to
// scanning /dev/input/event* for any device reporting the WASD keys.
// Returns a non-blocking fd on success, or -1 on failure.
int open_keyboard_evdev() {
  const char *by_id_dir = "/dev/input/by-id";
  if (DIR *d = opendir(by_id_dir)) {
    struct dirent *ent;
    while ((ent = readdir(d)) != nullptr) {
      const std::string name = ent->d_name;
      // udev names USB keyboards like "usb-<vendor>-event-kbd".
      if (name.size() >= 9 &&
          name.compare(name.size() - 9, 9, "event-kbd") == 0) {
        const std::string path = std::string(by_id_dir) + "/" + name;
        int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd >= 0) {
          std::cout << "Opened keyboard: " << path << "\n";
          closedir(d);
          return fd;
        }
      }
    }
    closedir(d);
  }

  for (int i = 0; i < 32; ++i) {
    const std::string path = "/dev/input/event" + std::to_string(i);
    int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
      continue;
    }
    if (evdev_supports_wasd(fd)) {
      std::cout << "Opened keyboard: " << path << "\n";
      return fd;
    }
    close(fd);
  }
  return -1;
}

// Degrees rotated per worker-thread iteration while a WASD key is held. Chosen
// as a trade-off: large enough to keep per-chunk overhead (thread spawn inside
// gantry::rotate_motors) negligible; small enough that releasing a key stops
// the gantry within a single chunk duration. At 500 RPM with 2000 PPR, 50 deg
// takes ~17 ms.
constexpr float CONTINUOUS_CHUNK_DEG = 50.0f;

// Idle poll period for the worker thread when no direction is active.
constexpr auto WORKER_IDLE_SLEEP = std::chrono::milliseconds(2);

// Velocity profile: ramp from MIN_RPM → MAX_RPM on key press, hold at MAX_RPM
// while key is held, then ramp from current RPM → MIN_RPM on key release
// before stopping.
constexpr float MIN_RPM = 500.0f;
constexpr float MAX_RPM = 700.0f;
constexpr auto RAMP_UP_MS = std::chrono::milliseconds(150);
constexpr auto RAMP_DOWN_MS = std::chrono::milliseconds(150);

enum class MotionPhase { IDLE, ACCEL, CRUISE, DECEL };

enum class TeleopDir { NONE, NORTH, SOUTH, EAST, WEST };

TeleopDir key_code_to_dir(uint16_t code) {
  switch (code) {
  case KEY_W:
    return TeleopDir::NORTH;
  case KEY_S:
    return TeleopDir::SOUTH;
  case KEY_A:
    return TeleopDir::WEST;
  case KEY_D:
    return TeleopDir::EAST;
  default:
    return TeleopDir::NONE;
  }
}

// Translate a logical direction into the (lower, upper) motor CW/CCW pair used
// by gantry::rotate_motors. Mapping matches the reference comments in the
// original main.cpp and the move_x / move_y direction conventions in
// gantry.cpp.
void dir_to_motor_dirs(TeleopDir d, uint8_t &lower_dir, uint8_t &upper_dir,
                       const char *&label) {
  switch (d) {
  case TeleopDir::NORTH:
    lower_dir = iSV57T::CCW;
    upper_dir = iSV57T::CW;
    label = "North";
    break;
  case TeleopDir::SOUTH:
    lower_dir = iSV57T::CW;
    upper_dir = iSV57T::CCW;
    label = "South";
    break;
  case TeleopDir::WEST:
    lower_dir = iSV57T::CW;
    upper_dir = iSV57T::CW;
    label = "West";
    break;
  case TeleopDir::EAST:
    lower_dir = iSV57T::CCW;
    upper_dir = iSV57T::CCW;
    label = "East";
    break;
  case TeleopDir::NONE:
  default:
    label = "";
    break;
  }
}

// Shared state between the keyboard reader (main thread) and the motor worker.
// - g_desired_dir: direction the worker should drive the gantry in, or NONE
//   to idle. Updated on key-press / key-release from evdev events.
// - g_active_key: the evdev key code that currently "owns" g_desired_dir. A
//   key release only clears the direction if it matches the active key, so
//   unrelated release events (e.g. stale repeats for a key that was already
//   overridden) do not accidentally stop motion.
std::atomic<TeleopDir> g_desired_dir{TeleopDir::NONE};
std::atomic<uint16_t> g_active_key{0};

} // namespace

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

  std::cout << "Initializing limit switch objects...\n";
  limitSwitch sw_y(chip4, sw1_line);
  limitSwitch sw_x(chip4, sw2_line);
  std::cout << "Finished limit switch object initialization!\n";

  std::cout << "Initializing motor object...\n";
  iSV57T m_lower = iSV57T(chip0, m1_dir_line, m1_pul_line, pulse_per_rev);
  iSV57T m_upper = iSV57T(chip0, m2_dir_line, m2_pul_line, pulse_per_rev);
  m_lower.set_target_rpm(500);
  m_upper.set_target_rpm(500);
  std::cout << "Finished motor object initialization!\n";

  std::cout << "Initializing gantry object...\n";
  gantry g = gantry(m_lower, m_upper, sw_x, sw_y);
  std::cout << "Finished gantry object initialization!\n";

  // Home the gantry so curr_x/curr_y are well-defined before any movement.
  std::cout << "Homing gantry to origin...\n";
  try {
    g.move_to_origin();
  } catch (const std::exception &e) {
    fprintf(stderr, "Homing failed: %s\n", e.what());
    gpiod_chip_close(chip4);
    gpiod_chip_close(chip0);
    return 1;
  }
  std::cout << "Gantry homed.\n";

  // SIGINT/SIGTERM exit the teleop loop cleanly; GPIO cleanup happens below.
  std::signal(SIGINT, on_sigint);
  std::signal(SIGTERM, on_sigint);

  int kbd_fd = open_keyboard_evdev();
  if (kbd_fd < 0) {
    fprintf(stderr,
            "No USB keyboard found under /dev/input. Plug in a keyboard and "
            "ensure /dev/input is accessible to this container.\n");
    gpiod_chip_close(chip4);
    gpiod_chip_close(chip0);
    return 1;
  }

  std::cout << "\nWASD teleop ready.\n"
            << "  W = North   S = South   A = West   D = East\n"
            << "  Hold a key to keep moving; release to stop. Ctrl+C to exit.\n"
            << "  Chunk: " << CONTINUOUS_CHUNK_DEG << " deg | RPM: " << MIN_RPM
            << "-" << MAX_RPM << " | ramp: " << RAMP_UP_MS.count() << "/"
            << RAMP_DOWN_MS.count() << " ms\n\n";

  // Motor worker thread with velocity profile state machine:
  //   IDLE  → key press  → ACCEL  (ramp MIN_RPM → MAX_RPM)
  //   ACCEL → ramp done  → CRUISE (hold MAX_RPM)
  //   ACCEL → key release → DECEL  (ramp current RPM → MIN_RPM, then stop)
  //   CRUISE→ key release → DECEL
  //   DECEL → ramp done  → IDLE
  //   DECEL → key press  → ACCEL  (ramp from current RPM → MAX_RPM)
  std::thread worker([&]() {
    using clock = std::chrono::steady_clock;

    MotionPhase phase = MotionPhase::IDLE;
    TeleopDir drive_dir = TeleopDir::NONE;
    clock::time_point phase_start{};
    float decel_start_rpm = MIN_RPM;

    while (g_running.load(std::memory_order_relaxed)) {
      const TeleopDir desired = g_desired_dir.load(std::memory_order_acquire);
      const auto now = clock::now();

      // --- Phase transitions ---

      if (phase == MotionPhase::IDLE) {
        if (desired == TeleopDir::NONE) {
          std::this_thread::sleep_for(WORKER_IDLE_SLEEP);
          continue;
        }
        phase = MotionPhase::ACCEL;
        drive_dir = desired;
        phase_start = now;
        const char *l = "";
        uint8_t ld = 0, ud = 0;
        dir_to_motor_dirs(drive_dir, ld, ud, l);
        // std::cout << "-> " << l << " (accel)\n";
      }

      if (phase == MotionPhase::ACCEL || phase == MotionPhase::CRUISE) {
        if (desired == TeleopDir::NONE) {
          // Key released — begin deceleration from whatever RPM we're at.
          float elapsed_ms =
              std::chrono::duration<float, std::milli>(now - phase_start)
                  .count();
          if (phase == MotionPhase::ACCEL) {
            float t = std::min(1.0f, elapsed_ms / float(RAMP_UP_MS.count()));
            decel_start_rpm = MIN_RPM + (MAX_RPM - MIN_RPM) * t;
          } else {
            decel_start_rpm = MAX_RPM;
          }
          phase = MotionPhase::DECEL;
          phase_start = now;
          // std::cout << "-> decel (from " << decel_start_rpm << " RPM)\n";
        } else if (desired != drive_dir) {
          // Direction changed while held — restart acceleration in new dir.
          drive_dir = desired;
          phase = MotionPhase::ACCEL;
          phase_start = now;
          const char *l = "";
          uint8_t ld = 0, ud = 0;
          dir_to_motor_dirs(drive_dir, ld, ud, l);
          // std::cout << "-> " << l << " (accel)\n";
        }
      }

      if (phase == MotionPhase::DECEL && desired != TeleopDir::NONE) {
        // Key pressed during deceleration — resume accelerating. Start the
        // ramp from the RPM we've decelerated to so the transition is smooth.
        float elapsed_ms =
            std::chrono::duration<float, std::milli>(now - phase_start).count();
        float t = std::min(1.0f, elapsed_ms / float(RAMP_DOWN_MS.count()));
        float current_rpm = decel_start_rpm - (decel_start_rpm - MIN_RPM) * t;
        // Re-use ACCEL phase; offset phase_start backwards so the ramp
        // function yields current_rpm at "now".
        float frac = (current_rpm - MIN_RPM) / (MAX_RPM - MIN_RPM);
        auto offset = std::chrono::duration_cast<clock::duration>(
            std::chrono::duration<float, std::milli>(
                frac * float(RAMP_UP_MS.count())));
        drive_dir = desired;
        phase = MotionPhase::ACCEL;
        phase_start = now - offset;
        const char *l = "";
        uint8_t ld = 0, ud = 0;
        dir_to_motor_dirs(drive_dir, ld, ud, l);
        // std::cout << "-> " << l << " (accel from " << current_rpm << "
        // RPM)\n";
      }

      // --- Compute RPM for the current phase ---

      float rpm = MIN_RPM;

      switch (phase) {
      case MotionPhase::ACCEL: {
        float elapsed_ms =
            std::chrono::duration<float, std::milli>(now - phase_start).count();
        float t = std::min(1.0f, elapsed_ms / float(RAMP_UP_MS.count()));
        rpm = MIN_RPM + (MAX_RPM - MIN_RPM) * t;
        if (t >= 1.0f) {
          phase = MotionPhase::CRUISE;
          // std::cout << "-> cruise (" << MAX_RPM << " RPM)\n";
        }
        break;
      }
      case MotionPhase::CRUISE:
        rpm = MAX_RPM;
        break;
      case MotionPhase::DECEL: {
        float elapsed_ms =
            std::chrono::duration<float, std::milli>(now - phase_start).count();
        float t = std::min(1.0f, elapsed_ms / float(RAMP_DOWN_MS.count()));
        rpm = decel_start_rpm - (decel_start_rpm - MIN_RPM) * t;
        if (t >= 1.0f) {
          phase = MotionPhase::IDLE;
          m_lower.set_target_rpm(MIN_RPM);
          m_upper.set_target_rpm(MIN_RPM);
          // std::cout << "-> stop\n";
          continue;
        }
        break;
      }
      case MotionPhase::IDLE:
        continue;
      }

      // --- Apply RPM and drive one chunk ---

      m_lower.set_target_rpm(rpm);
      m_upper.set_target_rpm(rpm);

      uint8_t lower_dir = iSV57T::CW;
      uint8_t upper_dir = iSV57T::CW;
      const char *label = "";
      dir_to_motor_dirs(drive_dir, lower_dir, upper_dir, label);

      try {
        g.rotate_motors(CONTINUOUS_CHUNK_DEG, lower_dir, upper_dir,
                        gantry::MotorSelect::BOTH);
      } catch (const std::exception &e) {
        fprintf(stderr, "Motor rotation failed: %s\n", e.what());
        phase = MotionPhase::IDLE;
        m_lower.set_target_rpm(MIN_RPM);
        m_upper.set_target_rpm(MIN_RPM);
        g_desired_dir.store(TeleopDir::NONE, std::memory_order_release);
        g_active_key.store(0, std::memory_order_release);
      }
    }
  });

  // Keyboard reader: translates evdev key-press/key-release into
  // g_desired_dir / g_active_key updates. Autorepeat (value==2) is ignored
  // because the worker thread provides continuous motion while the key is
  // physically held (value 1 -> 0 spans the whole hold period).
  struct pollfd pfd {};
  pfd.fd = kbd_fd;
  pfd.events = POLLIN;

  while (g_running.load(std::memory_order_relaxed)) {
    const int rc = poll(&pfd, 1, 100);
    if (rc < 0) {
      if (errno == EINTR) {
        continue;
      }
      fprintf(stderr, "poll() failed: %s\n", strerror(errno));
      break;
    }
    if (rc == 0) {
      continue; // timeout; re-check g_running
    }
    if (!(pfd.revents & POLLIN)) {
      continue;
    }

    struct input_event ev {};
    ssize_t n = read(kbd_fd, &ev, sizeof(ev));
    while (n == static_cast<ssize_t>(sizeof(ev))) {
      if (ev.type == EV_KEY) {
        const TeleopDir mapped = key_code_to_dir(ev.code);
        if (mapped != TeleopDir::NONE) {
          if (ev.value == 1) {
            // Press: take ownership of the motion state. Last-press wins, so
            // pressing D while W is held switches the gantry to East.
            g_active_key.store(ev.code, std::memory_order_release);
            g_desired_dir.store(mapped, std::memory_order_release);
          } else if (ev.value == 0) {
            // Release: only stop if this key is the one currently driving
            // motion. Releasing a key that was already overridden (e.g. W
            // after D took over) must not stop the gantry.
            const uint16_t expected = ev.code;
            uint16_t active = g_active_key.load(std::memory_order_acquire);
            if (active == expected) {
              g_active_key.store(0, std::memory_order_release);
              g_desired_dir.store(TeleopDir::NONE, std::memory_order_release);
            }
          }
          // ev.value == 2 (kernel autorepeat): intentionally ignored.
        }
      }
      n = read(kbd_fd, &ev, sizeof(ev));
    }

    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      fprintf(stderr, "read() from keyboard failed: %s\n", strerror(errno));
      break;
    }
  }

  std::cout << "\nShutting down teleop...\n";

  // Tell the worker to stop driving the motors; the next chunk it's in will
  // finish and then the worker observes g_running=false and exits.
  g_desired_dir.store(TeleopDir::NONE, std::memory_order_release);
  g_active_key.store(0, std::memory_order_release);
  worker.join();

  close(kbd_fd);
  gpiod_chip_close(chip4);
  gpiod_chip_close(chip0);

  return 0;
}

// 63.7919 left to right mmm = 7900 degrees

// 7800 degrees top to bottom

// 125 mm diagonal == 43 rotation of 360 = 15480 degrees

// Golden Ratio: 123.84 degrees per 1 mm
