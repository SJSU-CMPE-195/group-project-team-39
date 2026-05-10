#include <cstring>
#include <iostream>
#include <sstream>
#include <string>

#include <gpiod.h>

#include "gantry.hpp"
#include "iSV57T.hpp"
#include "limitSwitch.hpp"

int main() {
  gpiod_chip *chip0 = gpiod_chip_open("/dev/gpiochip0");
  if (!chip0) {
    fprintf(stderr, "open gpiochip0: %s\n", strerror(errno));
    return 1;
  }

  gpiod_chip *chip4 = gpiod_chip_open("/dev/gpiochip4");
  if (!chip4) {
    fprintf(stderr, "open gpiochip4: %s\n", strerror(errno));
    gpiod_chip_close(chip0);
    return 1;
  }

  limitSwitch sw_y(chip4, 1);
  limitSwitch sw_x(chip4, 2);

  iSV57T m_lower(chip0, 8, 9, 2000);
  iSV57T m_upper(chip0, 12, 13, 2000);

  try {
    m_lower.set_target_rpm(500);
    m_upper.set_target_rpm(500);
  } catch (const std::exception &e) {
    std::cerr << "RPM set failed: " << e.what() << "\n";
    gpiod_chip_close(chip4);
    gpiod_chip_close(chip0);
    return 1;
  }

  gantry g(m_lower, m_upper, sw_x, sw_y);
  g.curr_x = GANTRY_X_MAX_LENGTH / 2;
  g.curr_y = GANTRY_Y_MAX_LENGTH / 2;

  std::cout << "Gantry ready. Assumed position: x=" << g.curr_x
            << " y=" << g.curr_y << " mm (midpoint — jog freely).\n\n"
            << "Commands:\n"
            << "  x <mm>    jog X  (+East / -West)\n"
            << "  y <mm>    jog Y  (+North / -South)\n"
            << "  pos       print current position\n"
            << "  mark x    record X as MALLET_HOME_X_MM\n"
            << "  mark y    record Y as DEFENSIVE_Y_MM\n"
            << "  q         quit and print values\n\n";

  int home_x = -1, defensive_y = -1;
  std::string line;

  while (std::getline(std::cin, line)) {
    if (line.empty())
      continue;

    std::istringstream ss(line);
    std::string cmd;
    ss >> cmd;

    if (cmd == "q") {
      break;

    } else if (cmd == "pos") {
      std::cout << "x=" << g.curr_x << " y=" << g.curr_y << " mm\n";

    } else if (cmd == "mark") {
      std::string axis;
      ss >> axis;
      if (axis == "x") {
        home_x = g.curr_x;
        std::cout << "MALLET_HOME_X_MM = " << home_x << "\n";
      } else if (axis == "y") {
        defensive_y = g.curr_y;
        std::cout << "DEFENSIVE_Y_MM = " << defensive_y << "\n";
      } else {
        std::cerr << "usage: mark x  OR  mark y\n";
      }

    } else if (cmd == "x") {
      int mm = 0;
      if (!(ss >> mm)) {
        std::cerr << "usage: x <mm>\n";
        continue;
      }
      try {
        if (mm > 0)
          g.move_x((unsigned)mm, false); // East
        else if (mm < 0)
          g.move_x((unsigned)-mm, true); // West
        std::cout << "x=" << g.curr_x << " y=" << g.curr_y << " mm\n";
      } catch (const std::exception &e) {
        std::cerr << "move failed: " << e.what() << "\n";
      }

    } else if (cmd == "y") {
      int mm = 0;
      if (!(ss >> mm)) {
        std::cerr << "usage: y <mm>\n";
        continue;
      }
      try {
        if (mm > 0)
          g.move_y((unsigned)mm, true); // North
        else if (mm < 0)
          g.move_y((unsigned)-mm, false); // South
        std::cout << "x=" << g.curr_x << " y=" << g.curr_y << " mm\n";
      } catch (const std::exception &e) {
        std::cerr << "move failed: " << e.what() << "\n";
      }

    } else {
      std::cerr << "unknown: " << cmd << "\n";
    }
  }

  std::cout << "\n=== Paste into main.cpp ===\n";
  std::cout << "static constexpr int MALLET_HOME_X_MM = " << home_x << ";\n";
  std::cout << "static constexpr int DEFENSIVE_Y_MM   = " << defensive_y << ";\n";

  gpiod_chip_close(chip4);
  gpiod_chip_close(chip0);
  return 0;
}
