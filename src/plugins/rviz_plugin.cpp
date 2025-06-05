#include <rviz_common/display_context.hpp>
#include "rviz_plugin.hpp"

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

  QObject::connect(
      widget_.get(), &Widget::topicUpdated, [&](const QString &topic) { publisher_->reset(topic.toStdString()); });

  RCLCPP_INFO(node->get_logger(), "On Initialise");
}

void RVizPanel::onUpdate() {
  const auto state = widget_->getState();
  publisher_->publishState(state);
}

void RVizPanel::load(const rviz_common::Config &config) {
  rviz_common::Panel::load(config);

  Widget::Config cfg = makeConfig();
  QString topic;
  if (config.mapGetString("topic", &topic)) {
    cfg.topic = topic.toStdString();
  }
  widget_->setConfig(cfg);
}

void RVizPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);

  const QString topic = QString::fromStdString(publisher_->getTopic());
  config.mapSetValue("topic", topic);
}

}  // namespace rviz
}  // namespace plugins
}  // namespace ros_virtual_joystick

// This macro registers the plugin
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ros_virtual_joystick::plugins::rviz::SinglePadJoystickPlugin, rviz_common::Panel);
PLUGINLIB_EXPORT_CLASS(ros_virtual_joystick::plugins::rviz::DualPadJoystickPlugin, rviz_common::Panel);
