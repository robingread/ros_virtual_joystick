#include "joystick_widget.hpp"

namespace ros_virtual_joystick {
JoystickWidget::JoystickWidget(QWidget *parent, const std::size_t size) : QWidget(parent) {
  graphics_view_ = new JoystickGraphicsViewWidget(this, size);

  reset_on_release_button_ = new QPushButton("Hold Position", this);
  reset_on_release_button_->setCheckable(true);

  lock_axis_tracking_button_ = new QPushButton("Axis Lock", this);
  lock_axis_tracking_button_->setCheckable(true);

  h_layout_ = new QHBoxLayout();
  h_layout_->addStretch();
  h_layout_->addWidget(reset_on_release_button_);
  h_layout_->addWidget(lock_axis_tracking_button_);
  h_layout_->addStretch();

  // Setup the vertical layout with the widgets in it.
  v_layout_ = new QVBoxLayout(this);
  v_layout_->addWidget(graphics_view_);
  v_layout_->addLayout(h_layout_);
  v_layout_->addStretch();

  QObject::connect(
      reset_on_release_button_, &QPushButton::clicked, graphics_view_, &JoystickGraphicsViewWidget::toggleHoldPosition);

  QObject::connect(
      lock_axis_tracking_button_,
      &QPushButton::clicked,
      graphics_view_,
      &JoystickGraphicsViewWidget::toggleLockAxisTracking);

  QObject::connect(graphics_view_, &JoystickGraphicsViewWidget::stateUpdated, this, &JoystickWidget::onUpdate);
}

JoystickState JoystickWidget::getState() const {
  JoystickState state;
  state.axes = graphics_view_->getState();
  return state;
}

void JoystickWidget::onUpdate() { emit stateUpdated(); }

}  // namespace ros_virtual_joystick
