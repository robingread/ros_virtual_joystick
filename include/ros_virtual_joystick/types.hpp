#pragma once

#include <vector>

namespace ros_virtual_joystick {

/**
 * @brief Enum outlining the options for the joystick arrangement.
 *
 * Current options are:
 * - Single joystick + buttons.
 * - Dual joystick + buttons.
 */
enum Layout { SINGLE = 0, DUAL = 1 };

/**
 * @brief Structure to capture the state of the Joystick GUI.
 *
 * Axes values should be in the range (-1, 1). Button values are either [0, 1] where 0 denotes released, and 1 denotes
 * pressed.
 */
struct JoystickState {
  std::vector<float> axes;
  std::vector<int> buttons;
};

}  // namespace ros_virtual_joystick
