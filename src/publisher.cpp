#include "ros_virtual_joystick/publisher.hpp"

namespace ros_virtual_joystick {

Publisher::Publisher(rclcpp::Node &node, const std::string &topic) : node_(node) { reset(topic); }

Publisher::~Publisher() { publisher_.reset(); }

void Publisher::publishState(const JoystickState &state) {
  if (publisher_ == nullptr) {
    return;
  }

  sensor_msgs::msg::Joy msg;
  msg.header.stamp = node_.get_clock()->now();
  msg.axes.insert(msg.axes.end(), state.axes.begin(), state.axes.end());
  msg.buttons.insert(msg.buttons.end(), state.buttons.begin(), state.buttons.end());
  publisher_->publish(msg);
}

std::string Publisher::getTopic() const { return publisher_->get_topic_name(); }

void Publisher::reset(const std::string &topic) {
  publisher_.reset();
  publisher_ = node_.create_publisher<sensor_msgs::msg::Joy>(topic, 10);
}

}  // namespace ros_virtual_joystick
