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
    7900; // West-East from play side POV; Units in degrees
static constexpr int GANTRY_Y_MAX_ROTATIONS =
    8095; // North-South from play side POV; Units in degrees
static constexpr float HOMING_STEP_DEG = 10.0f;
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
   * @param p_y_origin refers to the y-axis limit swictch at the origin point of
   * the gantry. Pass a limtiSwitch object.
   */
  gantry(iSV57T &p_lower_motor, iSV57T &p_upper_motor, limitSwitch &p_x_origin,
         limitSwitch &p_y_origin);

  /**
   * @brief TODO
   */
  [[deprecated("Work in Progress")]] void move_to_push();

  /**
   * @brief Moves to the edges of the gantry system to see if all movement
   * function works.
   */
  bool calibration_test();

  /**
   * @brief Moves the center of the gantry from any point it started from to
   * origin. Will use limit switches as sensors to know when coordinate 0 in
   * both axises are hit.
   */
  bool move_to_origin();

  /**
   * @brief Moves the center of the gantry system to a desired coordinate point.
   * The coordinate grid is defined in millimeters. If possible, the motor will
   * move diagonally to cover the shortest distance. Than whatever is remaining,
   * the gantry will move horizontally and/or vertically to reach its
   * destination. The function will return true if the action is completed.
   * Otherwise, return false.
   *
   * NOTE: The coordinate system is in units of millimeters (mm).
   *
   * @param p_x_target is the target x-coordinate.
   * @param p_y_target is the target y-coordinate.
   */
  bool move_to_coord(unsigned p_x_target, unsigned p_y_target);

  /**
   * @brief Moves the center in the West-East direction (given the play POV) of
   * the gantry system. Takes an millimeter input and converts it to the
   * appropriate # of rotations for the motors to spin. After moving the motor,
   * the function will update its current coordinates. If the action completed
   * successfully, the function will return true. Otherwise, the function will
   * return false.
   *
   * DETAILS: Both m_lower_motor and m_top_motor needs to spin in the same
   * direction to move in this direction. Moving CLOCKWISE will move the gantry
   * WEST. Moving COUNTER CLOCKWISE will move the gantry EAST.
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
   * appropriate # of rotations for the motors to spin. After moving the motor,
   * the function will update its current coordinates. If the action completed
   * successfully, the function will return true. Otherwise, the function will
   * return false.
   *
   * DETAILS: The m_lower_motor and m_top_motor needs to spin in the oppposite
   * direction to move in this direction. m_lower_motor moving in COUNTER
   * CLOCKWISE and m_upper_motor moving in CLOCKWISE will move the gantry in the
   * NORTH direction. m_lower_motor moving in CLOCKWISE and m_upper_motor moving
   * in COUNTER CLOCKWISE will move the gantry in the SOUTH direction.
   *
   * @param p_mm gives the amount of millimeteres the gantry system should move
   * from its current position.
   * @param p_direction A FALSE input will move the center towards
   * the SOUTH direction. A TRUE input will move the center towards the
   * NORTH direction.
   */
  bool move_y(unsigned int p_mm, bool p_direction);

  /**
   * @brief Moves the center diagonally. So the motor can either move in the
   * SouthWest-NorthEast direction OR SouthEast-NorthWest direction (given the
   * play POV) of the gantry system. Takes a millimeter input and converts it to
   * the appropriate # of rotations for the motor to spin. After moving the
   * motor, the function will update its current coordinates. If the action
   * completed successfully, the function will return true. Otherwise, the
   * function will return false.
   *
   * DETAILS: One motor needs to spin at a time to move diagonally. If only
   * rotating the m_upper_motor Clockwise, the gantry moves in the NorthWest
   * direction. If m_upper_motor is rotated Counter Clockwise, the gantry moves
   * SouthEast. If only m_lower_motor Clockwise, the gantry moves in the
   * SouthWest direction. If the m_lower_motor moves Counter Clockwise, the
   * gantry moves in the NorthEast direction.
   *
   * @param p_mm gives the amount of millimeters the gantry system should move
   * diagonally from its current position.
   * @param p_direction gives the diagonal direction the gantry should move.
   * There are 4 values that p_direction can be: 0 - move in the SouthEast
   * direction, 1 - move in the NorthWest direction, 2 - move in the SouthWest
   * direction, 3 - move in the NorthEast direction.
   */
  bool move_diagonal(int p_mm, uint8_t p_direction);

  // Testing Purposes; Uncomment if you want to use raw motor movement function
  // enum class MotorSelect { BOTH, LOWER_ONLY, UPPER_ONLY };
  // void rotate_motors(float p_deg, uint8_t p_lower_dir, uint8_t p_upper_dir,
  //                    MotorSelect p_select);
  // int curr_x;
  // int curr_y;

  bool better_move(int dx, int dy);
  bool rotate_motors_independent(float lower_deg, uint8_t lower_dir,
                                 float upper_deg, uint8_t upper_dir);

private:
  /**
   * @brief Selects which motor(s) to run in rotate_both_motors.
   */
  enum class MotorSelect { BOTH, LOWER_ONLY, UPPER_ONLY };

  /**
   * @brief Helper function that runs one or both motors concurrently on
   * separate threads. Throws a std::runtime_error if a selected motor fails,
   * identifying which motor failed and the original error message.
   *
   * @param p_deg Degrees for the motor(s) to rotate.
   * @param p_lower_dir Direction for the lower motor (iSV57T::CW or
   * iSV57T::CCW).
   * @param p_upper_dir Direction for the upper motor (iSV57T::CW or
   * iSV57T::CCW).
   * @param p_select Selects which motor(s) to run. Defaults to BOTH.
   */
  void rotate_motors(float p_deg, uint8_t p_lower_dir, uint8_t p_upper_dir,
                     MotorSelect p_select);

  iSV57T &m_lower_motor;
  iSV57T &m_upper_motor;
  limitSwitch &m_x_origin;
  limitSwitch &m_y_origin;

  // Let's deal with whole numbers for simiplicity
  int curr_x;
  int curr_y;
};