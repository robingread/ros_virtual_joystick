#include <rviz_common/display_context.hpp>
#include "ros_virtual_joystick/rviz_plugin.hpp"

namespace ros_virtual_joystick {
namespace plugins {
namespace rviz {

RVizPanel::RVizPanel(QWidget *parent) : rviz_common::Panel(parent), layout_(new QVBoxLayout(parent)) {
  setLayout(layout_);
}

void RVizPanel::onInitialize() {
  widget_ = std::make_shared<Widget>(makeConfig(), this);
  layout_->addWidget(widget_.get());

  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  publisher_ = std::make_unique<Publisher>(*node);

  QObject::connect(widget_.get(), &Widget::stateUpdated, this, &RVizPanel::onUpdate);
}

void RVizPanel::onUpdate() {
  const auto state = widget_->getState();
  publisher_->publishState(state);
}

}  // namespace rviz
}  // namespace plugins
}  // namespace ros_virtual_joystick

// This macro registers the plugin
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ros_virtual_joystick::plugins::rviz::SinglePadJoystickPlugin, rviz_common::Panel);
PLUGINLIB_EXPORT_CLASS(ros_virtual_joystick::plugins::rviz::DualPadJoystickPlugin, rviz_common::Panel);
