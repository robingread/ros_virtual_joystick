#include "ros_virtual_joystick/widget.hpp"
#include "widgets/widget_impl.hpp"

namespace ros_virtual_joystick {
void clearLayout(QLayout *layout) {
  if (!layout)
    return;
  QLayoutItem *item;
  while ((item = layout->takeAt(0)) != nullptr) {
    if (QWidget *widget = item->widget()) {
      widget->setParent(nullptr);
      widget->deleteLater();  // Safe async deletion
    } else if (QLayout *childLayout = item->layout()) {
      clearLayout(childLayout);  // Recursively clear nested layouts
    }
    delete item;
  }
}

Widget::Widget(const Config &config, QWidget *parent) :
      QWidget(parent), layout_(new QVBoxLayout(this)), impl_(std::make_unique<Widget::Impl>(config, this, layout_)) {}

Widget::~Widget() = default;

void Widget::setConfig(const Config &config) {
  clearLayout(layout_);

  impl_.reset();
  impl_ = std::make_unique<Widget::Impl>(config, this, layout_);

  emit topicUpdated(QString(config.topic.c_str()));
}

JoystickState Widget::getState() const { return impl_->getState(); }

}  // namespace ros_virtual_joystick
