#pragma once

#include "widget.hpp"
#include "publisher.hpp"
#include <memory>
#include <rviz_common/panel.hpp>

namespace ros_virtual_joystick {
namespace plugins {

/**
 * @brief The RViz Panel plugin, as the name suggests, provides a Panel that can be loaded into RViz and display the
 * virtual joystick and publish the state to ROS via  sensor_msgs/msg/Joy data type.
 */
class RVizPanel : public rviz_common::Panel {
  Q_OBJECT

public:
  /**
   * @brief Construct a new RVizPanel object
   * @param parent Parent widget.
   */
  RVizPanel(QWidget *parent = nullptr);

  /**
   * @brief Initialise the Plugin.
   */
  void onInitialize() override;

public slots:
  /**
   * @brief Callback run when the the Joystick GUI has a state change.
   *
   * This callback will get the new state and publish it to the ROS system.
   */
  void onUpdate();

private:
  std::shared_ptr<Widget> widget_;
  std::unique_ptr<PublisherBase> publisher_;
};

}  // namespace plugins
}  // namespace ros_virtual_joystick
