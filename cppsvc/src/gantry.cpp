#include "gantry.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <stdexcept>
#include <string>
#include <thread>

// Private functions

void gantry::run_motors(float lower_deg, uint8_t lower_dir, float upper_deg,
                        uint8_t upper_dir, bool profiled) {
  const bool run_lower = (lower_deg > 0.0f);
  const bool run_upper = (upper_deg > 0.0f);

  if (!run_lower && !run_upper)
    return;

  // Compute per-motor ramp parameters so that both motors start and finish at
  // the same wall-clock time: each motor's cruise RPM is scaled by the fraction
  // of the total move it carries (dominant motor = RAMP_BASE_RPM).
  float lower_start = 0.0f, lower_cruise = 0.0f, lower_end = 0.0f;
  float upper_start = 0.0f, upper_cruise = 0.0f, upper_end = 0.0f;

  if (profiled) {
    const float max_deg = std::max(lower_deg, upper_deg);
    const float lower_scale = run_lower ? (lower_deg / max_deg) : 0.0f;
    const float upper_scale = run_upper ? (upper_deg / max_deg) : 0.0f;

    lower_cruise = std::clamp(RAMP_BASE_RPM * lower_scale, 0.0f, 3000.0f);
    upper_cruise = std::clamp(RAMP_BASE_RPM * upper_scale, 0.0f, 3000.0f);
    lower_start = lower_cruise * RAMP_START_RPM_FRAC;
    lower_end = lower_cruise * RAMP_END_RPM_FRAC;
    upper_start = upper_cruise * RAMP_START_RPM_FRAC;
    upper_end = upper_cruise * RAMP_END_RPM_FRAC;
  }

  std::exception_ptr ep_lower = nullptr, ep_upper = nullptr;

  std::thread lower_thread, upper_thread;

  if (run_lower) {
    lower_thread = std::thread([&]() {
      try {
        if (profiled)
          m_lower_motor.rotate_profiled(lower_dir, lower_deg, lower_start,
                                        lower_cruise, lower_end,
                                        RAMP_UP_FRACTION, RAMP_DOWN_FRACTION);
        else
          m_lower_motor.rotate(lower_dir, lower_deg);
      } catch (...) {
        ep_lower = std::current_exception();
      }
    });
  }

  if (run_upper) {
    upper_thread = std::thread([&]() {
      try {
        if (profiled)
          m_upper_motor.rotate_profiled(upper_dir, upper_deg, upper_start,
                                        upper_cruise, upper_end,
                                        RAMP_UP_FRACTION, RAMP_DOWN_FRACTION);
        else
          m_upper_motor.rotate(upper_dir, upper_deg);
      } catch (...) {
        ep_upper = std::current_exception();
      }
    });
  }

  if (lower_thread.joinable())
    lower_thread.join();
  if (upper_thread.joinable())
    upper_thread.join();

  if (ep_lower) {
    try {
      std::rethrow_exception(ep_lower);
    } catch (const std::exception &e) {
      throw std::runtime_error(std::string("Lower motor failed: ") + e.what());
    }
  }
  if (ep_upper) {
    try {
      std::rethrow_exception(ep_upper);
    } catch (const std::exception &e) {
      throw std::runtime_error(std::string("Upper motor failed: ") + e.what());
    }
  }
}

bool gantry::move_relative(int dx, int dy) {
  if (dx == 0 && dy == 0)
    return true;

  const int new_x = curr_x + dx;
  const int new_y = curr_y + dy;

  if (new_x < 0 || new_x > GANTRY_X_MAX_LENGTH || new_y < 0 ||
      new_y > GANTRY_Y_MAX_LENGTH) {
    throw std::runtime_error("Move goes out of bounds.");
  }

  // CoreXY mixing: convert (dx, dy) mm into per-motor signed degrees.
  // Sign convention: +dx = East, +dy = North, matching move_x / move_y.
  const float lower_signed = -float(dx) * X_DEG_TO_MM - float(dy) * Y_DEG_TO_MM;
  const float upper_signed = -float(dx) * X_DEG_TO_MM + float(dy) * Y_DEG_TO_MM;

  const float lower_deg = std::abs(lower_signed);
  const float upper_deg = std::abs(upper_signed);

  const uint8_t lower_dir = (lower_signed >= 0.0f) ? iSV57T::CW : iSV57T::CCW;
  const uint8_t upper_dir = (upper_signed >= 0.0f) ? iSV57T::CW : iSV57T::CCW;

  run_motors(lower_deg, lower_dir, upper_deg, upper_dir, /*profiled=*/true);

  curr_x = new_x;
  curr_y = new_y;

  return true;
}

// Public functions

gantry::gantry(iSV57T &p_lower_motor, iSV57T &p_upper_motor,
               limitSwitch &p_x_origin, limitSwitch &p_y_origin)
    : m_lower_motor(p_lower_motor), m_upper_motor(p_upper_motor),
      m_x_origin(p_x_origin), m_y_origin(p_y_origin) {}

bool gantry::move_x(unsigned int p_mm, bool p_direction) {
  if (p_mm == 0)
    return true;
  // true = West (decreasing x), false = East (increasing x)
  const int dx = p_direction ? -int(p_mm) : int(p_mm);
  return move_relative(dx, 0);
}

bool gantry::move_y(unsigned int p_mm, bool p_direction) {
  if (p_mm == 0)
    return true;
  // true = North (increasing y), false = South (decreasing y)
  const int dy = p_direction ? int(p_mm) : -int(p_mm);
  return move_relative(0, dy);
}

bool gantry::move_to_origin() {
  // Phase 1: move South until m_y_origin triggers
  float total_y_deg = 0.0f;

  while (m_y_origin.read() != 1) {
    if (total_y_deg >= GANTRY_Y_MAX_ROTATIONS)
      throw std::runtime_error(
          "Y-axis homing failed: limit switch not reached.");
    run_motors(HOMING_STEP_DEG, iSV57T::CW, HOMING_STEP_DEG, iSV57T::CCW,
               /*profiled=*/false); // South
    total_y_deg += HOMING_STEP_DEG;
  }

  // Phase 2: move West until m_x_origin triggers
  float total_x_deg = 0.0f;

  while (m_x_origin.read() != 1) {
    if (total_x_deg >= GANTRY_X_MAX_ROTATIONS)
      throw std::runtime_error(
          "X-axis homing failed: limit switch not reached.");
    run_motors(HOMING_STEP_DEG, iSV57T::CW, HOMING_STEP_DEG, iSV57T::CW,
               /*profiled=*/false); // West
    total_x_deg += HOMING_STEP_DEG;
  }

  curr_x = 0;
  curr_y = 0;
  return true;
}

bool gantry::move_to_coord(unsigned p_x_target, unsigned p_y_target) {
  if ((int)p_x_target > GANTRY_X_MAX_LENGTH ||
      (int)p_y_target > GANTRY_Y_MAX_LENGTH)
    throw std::runtime_error("Target coordinate is out of bounds.");

  return move_relative((int)p_x_target - curr_x, (int)p_y_target - curr_y);
}

bool gantry::calibration_test() {
  if (!move_to_origin())
    return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  if (!move_to_coord(GANTRY_X_MAX_LENGTH, 0))
    return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  if (!move_to_coord(0, 0))
    return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  if (!move_to_coord(0, GANTRY_Y_MAX_LENGTH))
    return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  if (!move_to_coord(0, 0))
    return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  if (!move_to_coord(GANTRY_X_MAX_LENGTH, GANTRY_Y_MAX_LENGTH))
    return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  if (!move_to_coord(0, 0))
    return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  return true;
}
