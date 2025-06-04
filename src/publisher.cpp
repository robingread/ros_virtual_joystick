#include "ros_virtual_joystick/publisher.hpp"

namespace ros_virtual_joystick {

Publisher::Publisher(rclcpp::Node &node, const std::string &topic) :
      node_(node), publisher_(node.create_publisher<sensor_msgs::msg::Joy>(topic, 10)) {}

Publisher::~Publisher() { publisher_.reset(); }

void Publisher::publishState(const JoystickState &state) {
  sensor_msgs::msg::Joy msg;
  msg.header.stamp = node_.get_clock()->now();
  msg.axes.insert(msg.axes.end(), state.axes.begin(), state.axes.end());
  msg.buttons.insert(msg.buttons.end(), state.buttons.begin(), state.buttons.end());
  publisher_->publish(msg);
}

}  // namespace ros_virtual_joystick
