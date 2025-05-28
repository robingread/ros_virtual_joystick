#include <rviz_common/display_context.hpp>
#include "ros_virtual_joystick/rviz_plugin.hpp"

namespace ros_virtual_joystick {
namespace plugins {

RVizPanel::RVizPanel(QWidget *parent) : rviz_common::Panel(parent) {
  const Widget::Config cfg;
  widget_ = std::make_shared<Widget>(cfg, this);

  QVBoxLayout *layout = new QVBoxLayout(this);
  layout->addWidget(widget_.get());
  setLayout(layout);
}

void RVizPanel::onInitialize() {
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  publisher_ = std::make_unique<Publisher>(*node);

  QObject::connect(widget_.get(), &Widget::stateUpdated, this, &RVizPanel::onUpdate);
}

void RVizPanel::onUpdate() {
  const auto state = widget_->getState();
  publisher_->publishState(state);
}

}  // namespace plugins
}  // namespace ros_virtual_joystick

// This macro registers the plugin
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ros_virtual_joystick::plugins::RVizPanel, rviz_common::Panel)
