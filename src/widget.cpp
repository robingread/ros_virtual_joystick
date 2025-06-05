#include "ros_virtual_joystick/widget.hpp"
#include "widget_impl.hpp"

namespace ros_virtual_joystick {

Widget::Widget(const Config &config, QWidget *parent) :
      QWidget(parent), layout_(new QVBoxLayout(this)), impl_(std::make_unique<Widget::Impl>(config, this, layout_)) {}

Widget::~Widget() { impl_.release(); }

JoystickState Widget::getState() const { return impl_->getState(); }

}  // namespace ros_virtual_joystick
