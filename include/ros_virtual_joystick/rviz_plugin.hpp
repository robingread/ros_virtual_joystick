#pragma once

#include "widget.hpp"
#include "publisher.hpp"
#include <memory>
#include <rviz_common/panel.hpp>

namespace ros_virtual_joystick {
namespace plugins {
namespace rviz {

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

  void load(const rviz_common::Config &config) override;

  void save(rviz_common::Config config) const override;

public slots:
  /**
   * @brief Callback run when the the Joystick GUI has a state change.
   *
   * This callback will get the new state and publish it to the ROS system.
   */
  void onUpdate();

protected:
  virtual Widget::Config makeConfig() const = 0;

private:
  QVBoxLayout *layout_;
  std::shared_ptr<Widget> widget_;
  std::unique_ptr<PublisherBase> publisher_;
};

class SinglePadJoystickPlugin : public RVizPanel {
protected:
  Widget::Config makeConfig() const override {
    Widget::Config cfg;
    cfg.layout = Layout::SINGLE;
    return cfg;
  }
};

class DualPadJoystickPlugin : public RVizPanel {
protected:
  Widget::Config makeConfig() const override {
    Widget::Config cfg;
    cfg.layout = Layout::DUAL;
    return cfg;
  }
};

}  // namespace rviz
}  // namespace plugins
}  // namespace ros_virtual_joystick
