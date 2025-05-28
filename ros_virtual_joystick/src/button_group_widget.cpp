#include "button_group_widget.hpp"

namespace ros_virtual_joystick {
ButtonGroup::ButtonGroup(QWidget *parent) : QWidget(parent) {
  layout_ = new QHBoxLayout(this);

  for (int i = 0; i < 4; ++i) {
    const QString label(QChar('A' + i));
    buttons_[i] = new QPushButton(label, this);
    layout_->addWidget(buttons_[i]);
    QObject::connect(buttons_[i], &QPushButton::pressed, this, &ButtonGroup::onButtonEvent);
    QObject::connect(buttons_[i], &QPushButton::released, this, &ButtonGroup::onButtonEvent);
  }
}

QPushButton *ButtonGroup::getButton(const ButtonId id) { return buttons_[id]; }

std::vector<int> ButtonGroup::getState() const {
  std::vector<int> state;
  for (const auto &button : buttons_) {
    state.push_back(int(button->isDown()));
  }
  return state;
}

void ButtonGroup::onButtonEvent() { emit stateUpdated(); }

}  // namespace ros_virtual_joystick
