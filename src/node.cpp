#include <QApplication>
#include <QTimer>
#include "ros_virtual_joystick/widget.hpp"
#include "publisher.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  // Initialise both Qt and ROS
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  // Create a ROS node and spin in the background
  auto node = std::make_shared<rclcpp::Node>("joystick_gui_node");
  const std::string joy_topic = node->declare_parameter("joy_topic", "joy");
  const int count = node->declare_parameter("num_pads", 2);
  const ros_virtual_joystick::Layout layout = static_cast<ros_virtual_joystick::Layout>(count - 1);

  std::thread ros_spin_thread([node]() { rclcpp::spin(node); });

  // Create the publisher and the widget
  auto publisher = std::make_unique<ros_virtual_joystick::Publisher>(*node, joy_topic);

  const ros_virtual_joystick::Widget::Config cfg{250, layout, joy_topic};
  ros_virtual_joystick::Widget widget(cfg);
  widget.show();

  // Create a callback method that can be used to get the widget state can publish in on updates.
  QObject::connect(&widget, &ros_virtual_joystick::Widget::stateUpdated, [&publisher, &widget]() {
    const auto state = widget.getState();
    publisher->publishState(state);
  });

  QObject::connect(&widget, &ros_virtual_joystick::Widget::topicUpdated, [&publisher](const QString &topic) {
    publisher->reset(topic.toStdString());
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
