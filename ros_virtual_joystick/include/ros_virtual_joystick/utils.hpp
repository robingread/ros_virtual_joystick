#pragma once

#include <QPointF>

namespace ros_virtual_joystick {
namespace utils {

/**
 * @brief Normalise a value so that it falls into the range (-1, 1).
 *
 * The input value is clamped to fall into the range (min_value, max_value).
 *
 * @param value Input value to normalise.
 * @param min_value The minimum value of value range.
 * @param max_value The maximum value of value range.
 * @return The normalised value.
 * @throws Exception if min_value > max_value.
 */
float normaliseValue(const float value, const float min_value, const float max_value);

/**
 * @brief Normalise a coordinate so that it falls into the range (-1, 1).
 *
 * The input coordinate values are each clamped to fall into the range (min_value, max_value).
 *
 * @param coord The coordinate to normalise.
 * @param min_value The minimum value of value range for each axis.
 * @param max_value The maximum value of value range for each axis.
 * @return The normalised point.
 */
QPointF normaliseCoorindate(const QPointF &coord, const float min_value, const float max_value);

/**
 * @brief Clamp a coordinate so that it falls into a normalised circle, where (0, 0) is the centre of the circle.
 *
 * @param coord Coordinate to clamp.
 * @return Clamped coordinate.
 */
QPointF clampPositionToCircle(const QPointF &coord);

/**
 * @brief Calculate the axis lock tracking position given a normalised coordinate in the range [-1, 1].
 * @param coord Normalised coordinate.
 * @return Position sitting on either the X or Y axis.
 */
QPointF calculateAxisLockPosition(const QPointF &coord);

}  // namespace utils
}  // namespace ros_virtual_joystick
