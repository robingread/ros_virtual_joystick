#include "widget_impl.hpp"

namespace ros_virtual_joystick {

Widget::Impl::Impl(const Widget::Config &cfg, Widget *parent, QVBoxLayout *main_layout_) :
      parent_(parent),
      joystick_layout_(new QHBoxLayout()),
      button_group_(new widgets::ButtonGroup(parent)),
      topic_widget_(new widgets::TopicWidget(parent, QString(cfg.topic.c_str()))) {
  // Setup the Joystick Pads based on the number specified in the config
  joystick_layout_->addStretch();
  const std::size_t pad_count = std::size_t(cfg.layout) + 1;
  for (std::size_t i = 0; i < pad_count; ++i) {
    auto *joystick = new widgets::JoystickWidget(parent, cfg.size);
    joystick_layout_->addWidget(joystick, 0, Qt::AlignHCenter);
    QObject::connect(joystick, &widgets::JoystickWidget::stateUpdated, this, &Widget::Impl::onUpdate);
    joysticks_.push_back(joystick);
  }
  joystick_layout_->addStretch();

  // Setup the vertical layout with the widgets in it.
  main_layout_->addLayout(joystick_layout_);
  main_layout_->addWidget(button_group_, 0, Qt::AlignHCenter);
  main_layout_->addWidget(topic_widget_);
  main_layout_->addStretch();

  QObject::connect(button_group_, &widgets::ButtonGroup::stateUpdated, this, &Widget::Impl::onUpdate);

  QObject::connect(topic_widget_, &widgets::TopicWidget::topicUpdated, [&](const QString &topic) {
    emit parent_->topicUpdated(topic);
  });
}

JoystickState Widget::Impl::getState() const {
  JoystickState state;
  for (const auto joystick : joysticks_) {
    const auto &js_state = joystick->getState();
    state.axes.insert(state.axes.end(), js_state.axes.begin(), js_state.axes.end());
  }
  state.buttons = button_group_->getState();
  return state;
}

void Widget::Impl::onUpdate() { parent_->stateUpdated(); }

}  // namespace ros_virtual_joystick
