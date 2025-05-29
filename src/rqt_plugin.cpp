#include "ros_virtual_joystick/rqt_plugin.hpp"

#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>

namespace ros_virtual_joystick {
namespace plugins {
namespace rqt_gui {

RqtPlugin::RqtPlugin() : rqt_gui_cpp::Plugin() { setObjectName("RqtPlugin"); }

void RqtPlugin::initPlugin(qt_gui_cpp::PluginContext &context) {
  publisher_ = std::make_unique<Publisher>(*node_);

  widget_ = new Widget(makeConfig());

  context.addWidget(widget_);

  QObject::connect(widget_, &Widget::stateUpdated, this, &RqtPlugin::onUpdate);
}

RqtPlugin::~RqtPlugin() {}

void RqtPlugin::shutdownPlugin() {
  widget_.clear();
  publisher_.reset();
}

void RqtPlugin::onUpdate() {
  const auto state = widget_->getState();
  publisher_->publishState(state);
}

}  // namespace rqt_gui
}  // namespace plugins
}  // namespace ros_virtual_joystick

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ros_virtual_joystick::plugins::rqt_gui::SinglePadJoystickPlugin, rqt_gui_cpp::Plugin);
PLUGINLIB_EXPORT_CLASS(ros_virtual_joystick::plugins::rqt_gui::DualPadJoystickPlugin, rqt_gui_cpp::Plugin);
