#pragma once

#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QString>
#include <QWidget>

namespace ros_virtual_joystick {
namespace widgets {

/**
 * @brief A simple Qt widget for displaying and editing a ROS topic name with a publish button.
 *
 * TopicWidget provides a horizontal layout containing a label, a line edit for entering a topic name,
 * and a button labeled "Publish". When the button is clicked, the widget emits the `topicUpdated`
 * signal with the current text in the line edit. This is useful for UI components that need to let
 * users specify a topic name and trigger an action based on it, such as publishing a message or
 * reconfiguring a ROS node.
 */
class TopicWidget : public QWidget {
  Q_OBJECT

public:
  /**
   * @brief Construct a new TopicWidget instance.
   * @param parent Parent widget.
   * @param topic Topic name to set in the QLineEdit.
   */
  TopicWidget(QWidget *parent, const QString &topic);

  /**
   * @brief Get a pointer to the QPushButton
   * @return Button pointer.
   */
  QPushButton *pushButton() const;

  /**
   * @brief Set the topic name/text in the QLineEdit.
   * @param topic Topic name to set.
   */
  void setTopic(const QString &topic);

  /**
   * @brief Get the topic name/text from the QLineEdit.
   * @return Topic name.
   */
  QString topic() const;

signals:
  /**
   * @brief Signal to emit when the Publish button is clicked, with the topic name as a payload.
   * @param topic The topic name/text contained in the QLineEdit.
   */
  void topicUpdated(const QString &topic);

private:
  QHBoxLayout *layout_;
  QLabel *label_;
  QLineEdit *line_edit_;
  QPushButton *button_;
};

}  // namespace widgets
}  // namespace ros_virtual_joystick
