#pragma once

#include "types.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace ros_virtual_joystick {

/**
 * @brief Base class for a Publisher object that should publish the JoystickState data.
 */
class PublisherBase {
public:
  /**
   * @brief Default deconstructor.
   */
  ~PublisherBase() = default;

  /**
   * @brief Publish the JoystickState data.
   * @param state Data to publish.
   */
  virtual void publishState(const JoystickState &state) = 0;
};

/**
 * @brief Concrete implementation of a ROS2 publisher that will publish the JoystickState data as a sensor_msgs/msg/Joy
 * message.
 */
class Publisher : public PublisherBase {
public:
  /**
   * @brief Construct the ROS2 publisher object.
   * @param node The ROS node used to create the publisher.
   */
  Publisher(rclcpp::Node &node);

  /**
   * @brief Deconstructor that will disconnect the publisher.
   */
  ~Publisher();

  /**
   * @brief Publish the JoystickState data.
   * @param state Data to publish.
   */
  void publishState(const JoystickState &state) override;

private:
  rclcpp::Node &node_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_;
};
}  // namespace ros_virtual_joystick
