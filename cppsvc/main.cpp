#include <algorithm>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>

#include "gantry.hpp"
#include "iSV57T.hpp"
#include "ipc_handler.hpp"
#include "limitSwitch.hpp"

// Default gantry position when idle (center-south, mm)
static constexpr unsigned DEFAULT_X_MM = 434;
static constexpr unsigned DEFAULT_Y_MM = 0;

// Graceful shutdown on SIGINT / SIGTERM
static volatile sig_atomic_t g_running = 1;
static void signal_handler(int) { g_running = 0; }

int main() {
  // ── GPIO chip handles ─────────────────────────────────────────────────────
  const char *gpio_chip4_char = "/dev/gpiochip4"; // limit switches
  const char *gpio_chip0_char = "/dev/gpiochip0"; // motors

  // Motor pin assignments
  const unsigned m1_dir_line = 8;  // Physical Pin 13
  const unsigned m1_pul_line = 9;  // Physical Pin 14
  const unsigned m2_dir_line = 12; // Physical Pin 15
  const unsigned m2_pul_line = 13; // Physical Pin 16

  // Limit switch pin assignments
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

  // ── Hardware object construction ──────────────────────────────────────────
  std::cout << "Initializing limit switches...\n";
  limitSwitch sw_x(chip4, sw1_line);
  limitSwitch sw_y(chip4, sw2_line);
  std::cout << "Limit switches ready.\n";

  std::cout << "Initializing motors...\n";
  iSV57T m_lower(chip0, m1_dir_line, m1_pul_line, pulse_per_rev);
  iSV57T m_upper(chip0, m2_dir_line, m2_pul_line, pulse_per_rev);
  std::cout << "Motors ready.\n";

  std::cout << "Initializing gantry...\n";
  gantry g(m_lower, m_upper, sw_x, sw_y);
  std::cout << "Gantry ready.\n";

  // ── IPC handler ───────────────────────────────────────────────────────────
  std::cout << "Opening IPC...\n";
  IPCHandler ipc;
  std::cout << "IPC ready.\n";

  // ── Signal handlers ───────────────────────────────────────────────────────
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  // ── Startup sequence ──────────────────────────────────────────────────────
  std::cout << "Homing gantry...\n";
  ipc.write_status(0.0f, 0.0f, AirHockey::HOMING, 0);

  // g.move_to_origin();

  std::cout << "Homing complete.\n";

  std::cout << "Moving to default position (" << DEFAULT_X_MM << ", "
            << DEFAULT_Y_MM << ")...\n";
  g.move_to_coord(DEFAULT_X_MM, DEFAULT_Y_MM);
  ipc.write_status(static_cast<float>(DEFAULT_X_MM),
                   static_cast<float>(DEFAULT_Y_MM), AirHockey::STATE_IDLE,
                   1 // ready
  );
  std::cout << "Ready. Entering command loop.\n";

  // ── Command loop ──────────────────────────────────────────────────────────
  float curr_x = static_cast<float>(DEFAULT_X_MM);
  float curr_y = static_cast<float>(DEFAULT_Y_MM);

  while (g_running) {
    IPCHandler::CommandData cmd;
    ipc.wait_for_command(cmd);

    if (!g_running)
      break;

    std::cout << "[CMD] seq=" << cmd.seq
              << " cmd=" << static_cast<int>(cmd.command) << " target=("
              << cmd.target_x_mm << "," << cmd.target_y_mm << ")\n";

    // Signal that we are moving
    ipc.write_status(curr_x, curr_y, AirHockey::MOVING, 0);

    // Clamp target to valid gantry range
    unsigned tx = static_cast<unsigned>(
        std::clamp(static_cast<int>(cmd.target_x_mm), 0, GANTRY_X_MAX_LENGTH));
    unsigned ty = static_cast<unsigned>(
        std::clamp(static_cast<int>(cmd.target_y_mm), 0, GANTRY_Y_MAX_LENGTH));

    try {
      g.move_to_coord(tx, ty);
    } catch (const std::exception &e) {
      std::cerr << "[ERROR] move_to_coord failed: " << e.what() << "\n";
      ipc.write_status(curr_x, curr_y, AirHockey::ERROR, 0);
      continue;
    }

    curr_x = static_cast<float>(tx);
    curr_y = static_cast<float>(ty);

    ipc.write_status(curr_x, curr_y, AirHockey::STATE_IDLE, 1);
    std::cout << "[DONE] at (" << curr_x << "," << curr_y << ")\n";
  }

  std::cout << "Shutting down...\n";
  gpiod_chip_close(chip4);
  gpiod_chip_close(chip0);
  return 0;
}
