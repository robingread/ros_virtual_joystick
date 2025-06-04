#include "topic_widget.hpp"

namespace ros_virtual_joystick {
namespace widgets {

TopicWidget::TopicWidget(QWidget *parent, const QString &topic) :
      QWidget(parent),
      layout_(new QHBoxLayout(this)),
      label_(new QLabel(this)),
      line_edit_(new QLineEdit(topic, this)),
      button_(new QPushButton("Publish", this)) {
  label_->setText("Topic:");

  line_edit_->setMinimumWidth(250);
  line_edit_->setMaximumWidth(260);

  // Populate the layout.
  layout_->addStretch();
  layout_->addWidget(label_);
  layout_->addWidget(line_edit_);
  layout_->addWidget(button_);
  layout_->addStretch();
  setLayout(layout_);

  // Connect the button pressed event to a callback that emits the topicUpdated() signal.
  QObject::connect(button_, &QPushButton::clicked, [this]() { emit topicUpdated(this->topic()); });
}

QPushButton *TopicWidget::pushButton() const { return button_; }

void TopicWidget::setTopic(const QString &topic) { line_edit_->setText(topic); }

QString TopicWidget::topic() const { return line_edit_->text(); }

}  // namespace widgets
}  // namespace ros_virtual_joystick
