#include <QApplication>
#include <QTimer>
#include "ros_virtual_joystick/widget.hpp"
#include "ros_virtual_joystick/publisher.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  // Initialise both Qt and ROS
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  // Create a ROS node and spin in the background
  auto node = std::make_shared<rclcpp::Node>("joystick_gui_node");
  std::thread ros_spin_thread([node]() { rclcpp::spin(node); });

  // Create the publisher and the widget
  auto publisher = std::make_unique<ros_virtual_joystick::Publisher>(*node);

  const ros_virtual_joystick::Widget::Config cfg{250, ros_virtual_joystick::Layout::DUAL};
  ros_virtual_joystick::Widget widget(cfg);
  widget.show();

  // Create a callback method that can be used to get the widget state can publish in on updates.
  QObject::connect(&widget, &ros_virtual_joystick::Widget::stateUpdated, [&publisher, &widget]() {
    const auto state = widget.getState();
    publisher->publishState(state);
  });

  // Add a QTimer that periodically checks for shutdown
  QTimer timer;
  QObject::connect(&timer, &QTimer::timeout, [&app]() {
    if (!rclcpp::ok()) {
      app.quit();
    }
  });
  timer.start(100);  // check every 100 ms

  // Execute Qt loop
  int ret = app.exec();

  // Shutdown and cleanup
  rclcpp::shutdown();
  ros_spin_thread.join();

  return ret;
}
