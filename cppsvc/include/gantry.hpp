#pragma once

#include "iSV57T.hpp"
#include "limitSwitch.hpp"

#include <cstdint>

/**
 * @brief The interface for the gantry system
 */
static constexpr int GANTRY_X_MAX_LENGTH = 869; // In millimeters
static constexpr int GANTRY_Y_MAX_LENGTH = 901; // In millimeters
static constexpr int GANTRY_X_MAX_ROTATIONS =
    7925; // West-East from play side POV; Units in degrees
static constexpr int GANTRY_Y_MAX_ROTATIONS =
    8000; // North-South from play side POV; Units in degrees
static constexpr float HOMING_STEP_DEG = 10.0f;

// Trapezoidal ramp profile for move_relative.
// Adjust these constants and recompile to tune the feel of each move.
static constexpr float RAMP_BASE_RPM =
    1200.0f; // cruise RPM for the dominant motor
static constexpr float RAMP_START_RPM_FRAC =
    0.10f; // start speed as fraction of cruise RPM
static constexpr float RAMP_END_RPM_FRAC =
    0.10f; // end speed as fraction of cruise RPM
static constexpr float RAMP_UP_FRACTION =
    0.0f; // fraction of move spent ramping up
static constexpr float RAMP_DOWN_FRACTION =
    0.15f; // fraction of move spent ramping down

static constexpr float X_DEG_TO_MM =
    float(GANTRY_X_MAX_ROTATIONS) /
    float(GANTRY_X_MAX_LENGTH); // Ratio to define how many degrees to move 1
                                // millimeter in the x-axis.
static constexpr float Y_DEG_TO_MM =
    float(GANTRY_Y_MAX_ROTATIONS) /
    float(GANTRY_Y_MAX_LENGTH); // Ratio to define how many degrees to move 1
                                // millimeter in the y-axis.

class gantry {
public:
  /**
   * @brief Constructs a new (Core XY) Gantry object.
   *
   * @param p_lower_motor refers to the lower motor controlling the gantry. Pass
   * an iSV57T object.
   * @param p_upper_motor refers to the upper motor controlling the gantry. Pass
   * an iSV57T object.
   * @param p_x_origin refers to the x-axis limit switch at the origin point of
   * the gantry. Pass a limitSwitch object.
   * @param p_y_origin refers to the y-axis limit switch at the origin point of
   * the gantry. Pass a limitSwitch object.
   */
  gantry(iSV57T &p_lower_motor, iSV57T &p_upper_motor, limitSwitch &p_x_origin,
         limitSwitch &p_y_origin);

  /**
   * @brief Moves to the edges of the gantry system to see if all movement
   * functions work.
   */
  bool calibration_test();

  /**
   * @brief Moves the center of the gantry from any point it started from to
   * origin. Will use limit switches as sensors to know when coordinate 0 in
   * both axes are hit.
   */
  bool move_to_origin();

  /**
   * @brief Moves the center of the gantry system to a desired coordinate point.
   * The coordinate grid is defined in millimeters. The gantry will move in a
   * straight line to the destination using CoreXY motor mixing. The function
   * will return true if the action is completed. Otherwise, return false.
   *
   * @param puck_x current puck x position
   * @param puck_y current puck y position
   * @param target_x predicted target x
   * @param target_y predicted target y
   */
  bool move_to_adjusted_coord(unsigned puck_x, unsigned puck_y, unsigned target_x, unsigned target_y);

  /**
   * @brief Moves the center in the West-East direction (given the play POV) of
   * the gantry system. Takes a millimeter input and converts it to the
   * appropriate motor degrees using CoreXY mixing. After moving, the function
   * will update its current coordinates. Returns true on success.
   *
   * DETAILS: Both motors need to spin in the same direction to move in this
   * axis. Moving CLOCKWISE will move the gantry WEST. Moving COUNTER CLOCKWISE
   * will move the gantry EAST.
   *
   * @param p_mm gives the amount of millimeters the gantry system should move
   * from its current position.
   * @param p_direction tells the direction to move. A FALSE input will move the
   * center towards the EAST direction. A TRUE input will move the center
   * towards the WEST direction.
   */
  bool move_x(unsigned int p_mm, bool p_direction);

  /**
   * @brief Moves the center in the North-South direction (given the play POV)
   * of the gantry system. Takes a millimeter input and converts it to the
   * appropriate motor degrees using CoreXY mixing. After moving, the function
   * will update its current coordinates. Returns true on success.
   *
   * DETAILS: The motors need to spin in opposite directions to move in this
   * axis. Lower motor CCW + upper motor CW moves NORTH. Lower motor CW +
   * upper motor CCW moves SOUTH.
   *
   * @param p_mm gives the amount of millimeters the gantry system should move
   * from its current position.
   * @param p_direction A FALSE input will move the center towards the SOUTH
   * direction. A TRUE input will move the center towards the NORTH direction.
   */
  bool move_y(unsigned int p_mm, bool p_direction);

private:
  /**
   * @brief Runs one or both motors concurrently on separate threads. A motor
   * is skipped when its degree argument is zero. When profiled is true, uses
   * a trapezoidal velocity ramp (RAMP_* constants); otherwise rotates at the
   * motor's current constant speed. Throws std::runtime_error identifying
   * which motor failed if an exception occurs.
   *
   * @param lower_deg Degrees for the lower motor (skipped if 0).
   * @param lower_dir Direction for the lower motor (iSV57T::CW or CCW).
   * @param upper_deg Degrees for the upper motor (skipped if 0).
   * @param upper_dir Direction for the upper motor (iSV57T::CW or CCW).
   * @param profiled If true, uses rotate_profiled; if false, uses rotate.
   */
  void run_motors(float lower_deg, uint8_t lower_dir, float upper_deg,
                  uint8_t upper_dir, bool profiled);

  /**
   * @brief Moves the gantry by (dx, dy) millimeters from its current position
   * using CoreXY motor mixing. Bounds-checks the destination, runs the motors
   * with the ramped profile, and updates curr_x / curr_y on success.
   *
   * @param dx Signed displacement in the X axis (+ = East, - = West).
   * @param dy Signed displacement in the Y axis (+ = North, - = South).
   */
  bool move_relative(int dx, int dy);

  iSV57T &m_lower_motor;
  iSV57T &m_upper_motor;
  limitSwitch &m_x_origin;
  limitSwitch &m_y_origin;

  int curr_x = 0;
  int curr_y = 0;
};
