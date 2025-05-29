#pragma once

#include "widget.hpp"
#include "publisher.hpp"
#include <memory>
#include <QPointer>
#include <rqt_gui_cpp/plugin.h>

namespace ros_virtual_joystick {
namespace plugins {

/**
 * @brief This RqtPlugin class, as the name suggests, provides a Plugin  allowing the virtual joystick to be used within
 * an instance of the rqt_gui, and publish the state of the joystick to ROS via a sensor_msgs/msg/Joy data type.
 */
class RqtPlugin : public rqt_gui_cpp::Plugin {
  Q_OBJECT

public:
  /**
   * @brief Construct a new RqtPlugin object.
   *
   */
  RqtPlugin();

  /**
   * @brief Destroy the RqtPlugin object.
   */
  ~RqtPlugin();

  /**
   * @brief Initialise the plugin.
   *
   * @param context The context within which the plugin is running.
   */
  void initPlugin(qt_gui_cpp::PluginContext &context) override;

  /**
   * @brief Shutdown the plugin.
   */
  void shutdownPlugin() override;

public slots:
  /**
   * @brief Callback run when the Joystick GUI has a state change.
   *
   * This callback will get the new state and publish it to the ROS system.
   */
  void onUpdate();

private:
  QPointer<Widget> widget_;
  std::unique_ptr<PublisherBase> publisher_;
};

}  // namespace plugins
}  // namespace ros_virtual_joystick
