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

  QObject::connect(
      widget_, &Widget::topicUpdated, [&](const QString &topic) { publisher_->reset(topic.toStdString()); });
}

RqtPlugin::~RqtPlugin() {}

void RqtPlugin::shutdownPlugin() {
  widget_.clear();
  publisher_.reset();
}

void RqtPlugin::saveSettings(qt_gui_cpp::Settings & /*plugin_settings*/, qt_gui_cpp::Settings &instance_settings)
    const {
  const QString topic = QString::fromStdString(publisher_->getTopic());
  instance_settings.setValue("topic", topic);
}

void RqtPlugin::restoreSettings(
    const qt_gui_cpp::Settings & /*plugin_settings*/,
    const qt_gui_cpp::Settings &instance_settings) {
  Widget::Config config = makeConfig();
  const auto topic = instance_settings.value("topic", QString::fromStdString(config.topic));
  config.topic = topic.toString().toStdString();
  widget_->setConfig(config);
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
