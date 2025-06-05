#pragma once

#include "button_group_widget.hpp"
#include "joystick_widget.hpp"
#include "topic_widget.hpp"

#include "ros_virtual_joystick/types.hpp"
#include "ros_virtual_joystick/widget.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QObject>

#include <memory>
#include <vector>

namespace ros_virtual_joystick {

class Widget::Impl : public QObject {
  Q_OBJECT

public:
  Impl(const Widget::Config &cfg, Widget *parent, QVBoxLayout *main_layout_);
  ~Impl() = default;

  /**
   * @brief Get the state of the joystick.
   * @return The overall state of the buttons and axes.
   */
  JoystickState getState() const;

private slots:
  /**
   * @brief Qt Slot used as a callback when a UI element has been interacted with.
   *
   * The intention is that when UI element state changes, this callback emits the stateUpdated() signal.
   */
  void onUpdate();

private:
  Widget *parent_;

  QHBoxLayout *joystick_layout_;

  widgets::ButtonGroup *button_group_;
  widgets::TopicWidget *topic_widget_;
  std::vector<widgets::JoystickWidget *> joysticks_;
};

}  // namespace ros_virtual_joystick
