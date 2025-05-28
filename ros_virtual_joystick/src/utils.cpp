#include "utils.hpp"
#include <algorithm>
#include <cmath>
#include <format>
#include <stdexcept>

namespace ros_virtual_joystick {
namespace utils {

float normaliseValue(const float value, const float min_value, const float max_value) {
  // Check the input arguments are value.
  if (min_value > max_value) {
    throw std::invalid_argument(std::format(
        "The min_value is greater than the max_value - min_value: {}, max_value: {}", min_value, max_value));
  }

  const float v = std::clamp(value, min_value, max_value);
  const float range = max_value - min_value;
  const float n = (v - min_value) / range;
  return (n * 2.0) - 1.0;
}

QPointF normaliseCoordinate(const QPointF &coord, const float min_value, const float max_value) {
  const float x_norm = normaliseValue(coord.x(), min_value, max_value);
  const float y_norm = normaliseValue(coord.y(), min_value, max_value);
  return QPointF(x_norm, y_norm);
}

QPointF clampPositionToCircle(const QPointF &coord) {
  const float dist = std::sqrt(std::pow(coord.x(), 2.0) + std::pow(coord.y(), 2.0));
  if (dist <= 1.0) {
    return coord;
  }
  const float angle = std::atan2(coord.y(), coord.x());
  const float x = std::cos(angle);
  const float y = std::sin(angle);
  return QPointF(x, y);
}

QPointF calculateAxisLockPosition(const QPointF &coord) {
  float x = 0.0f;
  float y = 0.0f;
  if (std::fabs(coord.x()) > std::fabs(coord.y())) {
    x = coord.x();
  } else {
    y = coord.y();
  }
  return QPointF(x, y);
}

}  // namespace utils
}  // namespace ros_virtual_joystick
