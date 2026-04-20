#include "iSV57T.hpp"
#include "limitSwitch.hpp"
/**
 * @brief The interface for the gantry system
 */
constexpr int GANTRY_X_MAX_LENGTH = 0; // In millimeters
constexpr int GANTRY_Y_MAX_LENGTH = 0; // In millimeters
int GANTRY_X_MAX_ROTATIONS = 0;        // In degrees
int GANTRY_Y_MAX_ROTATIONS = 0;        // In degrees

class gantry {
public:
  gantry(iSV57T *p_lower_motor, iSV57T *p_upper_motor, limitSwitch *p_x_origin,
         limitSwitch *p_y_origin);

  void compute_desire_coord(); // Prob mutate a list or something idk
  void move_to_push();
  bool move_to_coord();
  bool move_x();
  bool move_y();
  bool move_diagonal();

private:
  iSV57T m_lower_motor;
  iSV57T m_upper_motor;
  limitSwitch m_x_origin;
  limitSwitch m_y_origin;

  // Let's deal with whole numbers for simiplicity
  int curr_x;
  int curr_y;
};