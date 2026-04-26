#include "gantry.hpp"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <gpiod.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <algorithm>
#include <exception>

// Private Functions
void gantry::rotate_motors(float p_deg, uint8_t p_lower_dir,
                           uint8_t p_upper_dir, MotorSelect p_select) {
  std::exception_ptr ep_lower_motor = nullptr, ep_upper_motor = nullptr;

  const bool run_lower =
      (p_select == MotorSelect::BOTH || p_select == MotorSelect::LOWER_ONLY);
  const bool run_upper =
      (p_select == MotorSelect::BOTH || p_select == MotorSelect::UPPER_ONLY);

  std::thread lower_motor_thread, upper_motor_thread;

  if (run_lower) {
    lower_motor_thread = std::thread([&]() {
      try {
        m_lower_motor.rotate(p_lower_dir, p_deg);
      } catch (...) {
        ep_lower_motor = std::current_exception();
      }
    });
  }

  if (run_upper) {
    upper_motor_thread = std::thread([&]() {
      try {
        m_upper_motor.rotate(p_upper_dir, p_deg);
      } catch (...) {
        ep_upper_motor = std::current_exception();
      }
    });
  }

  if (run_lower)
    lower_motor_thread.join();
  if (run_upper)
    upper_motor_thread.join();

  if (ep_lower_motor) {
    try {
      std::rethrow_exception(ep_lower_motor);
    } catch (const std::exception &e) {
      throw std::runtime_error(std::string("Lower motor failed: ") + e.what());
    }
  }
  if (ep_upper_motor) {
    try {
      std::rethrow_exception(ep_upper_motor);
    } catch (const std::exception &e) {
      throw std::runtime_error(std::string("Upper motor failed: ") + e.what());
    }
  }
}

// Public Functions
gantry::gantry(iSV57T &p_lower_motor, iSV57T &p_upper_motor,
               limitSwitch &p_x_origin, limitSwitch &p_y_origin)
    : m_lower_motor(p_lower_motor), m_upper_motor(p_upper_motor),
      m_x_origin(p_x_origin), m_y_origin(p_y_origin) {}

bool gantry::move_x(unsigned int p_mm, bool p_direction) {
  // If a given input was 0 we don't need to move anywhere so the function is
  // complete
  if (p_mm == 0)
    return true;

  // Getting the degrees to rotate
  float target_deg = float(p_mm) * X_DEG_TO_MM;
  int new_x;
  uint8_t target_dir;

  // Gets the direction of the motors to spin to move East/West
  if (p_direction) { // Moving West
    target_dir = iSV57T::CW;
    new_x = curr_x + p_mm;
  } else { // Moving East
    target_dir = iSV57T::CCW;
    new_x = curr_x - p_mm;
  }

  // Checks if moving p_mm will go beyond out gantry's boundaries
  if (new_x > GANTRY_X_MAX_LENGTH || new_x < 0) {
    throw std::runtime_error(std::string("Unable to move to in x-axis by ") +
                             std::to_string(p_mm) +
                             std::string(" because it went out of bounds."));
    return false;
  }

  rotate_motors(target_deg, target_dir, target_dir, MotorSelect::BOTH);

  // Set curr_x position
  curr_x = new_x;

  return true; // Was able to move the run each motor on individual threads
               // successfully
}

bool gantry::move_y(unsigned int p_mm, bool p_direction) {
  // If a given input was 0 we don't need to move anywhere so the function is
  // complete
  if (p_mm == 0)
    return true;

  // Getting the degrees to rotate
  float target_deg = float(p_mm) * Y_DEG_TO_MM;
  int new_y;
  uint8_t tar_lower_dir;
  uint8_t tar_upper_dir;

  // Gets the direction of the motors to spin to move East/West
  if (p_direction) { // Moving North
    tar_lower_dir = iSV57T::CCW;
    tar_upper_dir = iSV57T::CW;
    new_y = curr_y + p_mm;
  } else { // Moving South
    tar_lower_dir = iSV57T::CW;
    tar_upper_dir = iSV57T::CCW;
    new_y = curr_y - p_mm;
  }

  // Checks if moving p_mm will go beyond out gantry's boundaries
  if (new_y > GANTRY_Y_MAX_LENGTH || new_y < 0) {
    throw std::runtime_error(std::string("Unable to move to in y-axis by ") +
                             std::to_string(p_mm) +
                             std::string(" because it went out of bounds."));
    return false;
  }

  rotate_motors(target_deg, tar_lower_dir, tar_upper_dir, MotorSelect::BOTH);

  // Set curr_y position
  curr_y = new_y;

  return true;
}

bool gantry::move_to_origin() {
  // Phase 1: move South until m_y_origin triggers
  float total_y_deg = 0.0f;

  m_lower_motor.set_target_rpm(1000);
  m_upper_motor.set_target_rpm(1000);

  while (m_y_origin.read() != 1) {
    if (total_y_deg >= GANTRY_Y_MAX_ROTATIONS) {
      throw std::runtime_error(
          "Y-axis homing failed: limit switch not reached.");
      return false;
    }
    rotate_motors(HOMING_STEP_DEG, iSV57T::CW, iSV57T::CCW,
                  MotorSelect::BOTH); // Moving South
    total_y_deg += HOMING_STEP_DEG;
  }

  // Phase 2: move East until m_x_origin triggers
  float total_x_deg = 0.0f;
  while (m_x_origin.read() != 1) {
    if (total_x_deg >= GANTRY_X_MAX_ROTATIONS) {
      throw std::runtime_error(
          "X-axis homing failed: limit switch not reached.");
      return false;
    }
    rotate_motors(HOMING_STEP_DEG, iSV57T::CCW, iSV57T::CCW,
                  MotorSelect::BOTH); // Moving East
    total_x_deg += HOMING_STEP_DEG;
  }

  curr_x = 0;
  curr_y = 0;
  return true;
}

bool gantry::move_to_coord(unsigned p_x_target, unsigned p_y_target) {
  if ((int)p_x_target > GANTRY_X_MAX_LENGTH ||
      (int)p_y_target > GANTRY_Y_MAX_LENGTH) {
    throw std::runtime_error("Target coordinate is out of bounds.");
    return false;
  }

  int diff_x = (int)p_x_target - curr_x;
  int diff_y = (int)p_y_target - curr_y;

  // Use diagonal movement as much as possible, then finish with axis moves.
  // Note: move_diagonal decomposes euclidean distance into integer mm deltas,
  // so we recompute the residual diffs after the diagonal move.
better_move(diff_x, diff_y);
  return true;
}

bool gantry::move_diagonal(int p_mm, uint8_t p_direction) {
  if (p_mm == 0)
    return true;

  if (p_direction > 3)
    throw std::runtime_error("Invalid diagonal direction: " +
                             std::to_string(p_direction));

  MotorSelect motor_select;
  uint8_t lower_dir = 0;
  uint8_t upper_dir = 0;
  int x_sign, y_sign;

  switch (p_direction) {
  case 0: // SouthEast
    motor_select = MotorSelect::UPPER_ONLY;
    upper_dir = iSV57T::CCW;
    x_sign = -1;
    y_sign = -1;
    break;
  case 1: // NorthWest
    motor_select = MotorSelect::UPPER_ONLY;
    upper_dir = iSV57T::CW;
    x_sign = 1;
    y_sign = 1;
    break;
  case 2: // SouthWest
    motor_select = MotorSelect::LOWER_ONLY;
    lower_dir = iSV57T::CW;
    x_sign = 1;
    y_sign = -1;
    break;
  case 3: // NorthEast
    motor_select = MotorSelect::LOWER_ONLY;
    lower_dir = iSV57T::CCW;
    x_sign = -1;
    y_sign = 1;
    break;
  default:
    return false;
  }

  // In CoreXY, when only one motor spins θ degrees, each belt (X and Y) moves
  // θ/2 belt-degrees.  Converting belt-degrees to mm: X_mm = (θ/2)/X_DEG_TO_MM
  // and Y_mm = (θ/2)/Y_DEG_TO_MM.  The euclidean travel distance is therefore:
  //   p_mm = (θ/2) · √( inv_x² + inv_y² )   where inv_* = 1/DEG_TO_MM
  // Rearranging gives θ = 2·p_mm / norm.
  float inv_x = 1.0f / X_DEG_TO_MM;
  float inv_y = 1.0f / Y_DEG_TO_MM;
  float norm = std::sqrt(inv_x * inv_x + inv_y * inv_y);
  float theta =
      2.0f * float(p_mm) / norm; // degrees the selected motor must spin

  // Decompose the euclidean distance into per-axis mm shifts using the same
  // unit vector components (inv_x/norm and inv_y/norm).
  int delta_x = (int)std::round(float(p_mm) * inv_x / norm);
  int delta_y = (int)std::round(float(p_mm) * inv_y / norm);

  // Apply directional signs to get the candidate destination coordinates.
  int new_x = curr_x + x_sign * delta_x;
  int new_y = curr_y + y_sign * delta_y;

  // Checks if the we move out of bounds.
  if (new_x < 0 || new_x > GANTRY_X_MAX_LENGTH || new_y < 0 ||
      new_y > GANTRY_Y_MAX_LENGTH) {
    throw std::runtime_error(std::string("Unable to move diagonally by ") +
                             std::to_string(p_mm) +
                             std::string(" mm because it went out of bounds."));
    return false;
  }

  rotate_motors(theta, lower_dir, upper_dir, motor_select);

  // Update current position in the coordinate field
  curr_x = new_x;
  curr_y = new_y;

  return true;
}

bool gantry::calibration_test() {
  // Step 1 : Home to origin
  if (!move_to_origin())
    return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Step 2: Drive to X maximum (y = 0)
  if (!move_to_coord(GANTRY_X_MAX_LENGTH, 0))
    return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Step 3: Return to origin
  if (!move_to_coord(0, 0))
    return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Step 4: Drive to Y maximum (x = 0)
  if (!move_to_coord(0, GANTRY_Y_MAX_LENGTH))
    return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Step 5: Return to origin
  if (!move_to_coord(0, 0))
    return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Step 6: Drive to both X and Y maximum
  if (!move_to_coord(GANTRY_X_MAX_LENGTH, GANTRY_Y_MAX_LENGTH))
    return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Step 7: Return to origin
  if (!move_to_coord(0, 0))
    return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  return true;
}

bool gantry::better_move(int dx, int dy) {
  if (dx == 0 && dy == 0)
    return true;

  int new_x = curr_x + dx;
  int new_y = curr_y + dy;

  if (new_x < 0 || new_x > GANTRY_X_MAX_LENGTH ||
      new_y < 0 || new_y > GANTRY_Y_MAX_LENGTH) {
    throw std::runtime_error("Move goes out of bounds.");
    return false;
  }

  int lower_mm = dx - dy;
  int upper_mm = dx + dy;

  float lower_deg = std::abs(lower_mm) * X_DEG_TO_MM;
  float upper_deg = std::abs(upper_mm) * X_DEG_TO_MM;

  uint8_t lower_dir = (lower_mm >= 0) ? iSV57T::CW : iSV57T::CCW;
  uint8_t upper_dir = (upper_mm >= 0) ? iSV57T::CW : iSV57T::CCW;

  rotate_motors_independent(lower_deg, lower_dir, upper_deg, upper_dir);

  //this one function above should handle all movements (x,y,diagonal(different angles))
  curr_x = new_x;
  curr_y = new_y;

  return true;
}

bool gantry::rotate_motors_independent(float lower_deg, uint8_t lower_dir,
                                       float upper_deg, uint8_t upper_dir) {
  if (lower_deg == 0.0f && upper_deg == 0.0f)
    return true;

  float max_deg = std::max(lower_deg, upper_deg);
  if (max_deg <= 0.0f)
    return true;

//we can modify this to be how ever fast we wnat
  const float base_rpm = 1000.0f;

  float lower_rpm = 0.0f;
  float upper_rpm = 0.0f;
//set the rpm based on ratio of how many degrees each motor's gotta move
  if (lower_deg > 0.0f)
    lower_rpm = base_rpm * (lower_deg / max_deg);
  if (upper_deg > 0.0f)
    upper_rpm = base_rpm * (upper_deg / max_deg);

//technically we can set the lower rpm limit to 0 but it's ok
  if (lower_deg > 0.0f)
    lower_rpm = std::clamp(lower_rpm, 0.0f, 3000.0f);
  if (upper_deg > 0.0f)
    upper_rpm = std::clamp(upper_rpm, 0.0f, 3000.0f);

  if (lower_deg > 0.0f)
    m_lower_motor.set_target_rpm(lower_rpm);
  if (upper_deg > 0.0f)
    m_upper_motor.set_target_rpm(upper_rpm);

  std::exception_ptr eptr = nullptr;

  std::thread lower_thread([&]() {
    try {
      if (lower_deg > 0.0f)
        m_lower_motor.rotate(lower_dir, lower_deg);
    } catch (...) {
      eptr = std::current_exception();
    }
  });

  std::thread upper_thread([&]() {
    try {
      if (upper_deg > 0.0f)
        m_upper_motor.rotate(upper_dir, upper_deg);
    } catch (...) {
      eptr = std::current_exception();
    }
  });

  lower_thread.join();
  upper_thread.join();

  if (eptr)
    std::rethrow_exception(eptr);

  return true;
}
