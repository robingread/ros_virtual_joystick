#pragma once

#include "ros_virtual_joystick/types.hpp"

#include <QVBoxLayout>
#include <QWidget>

namespace ros_virtual_joystick {

/**
 * @brief Widget that provdes a Qt based GUI that provides a single or dual pad joystick and set of buttons.
 *
 * The object emits a a stateUpdated() signal whenever the GUI state changes (e.g, a button press/release, a joystick
 * moved), and the current state can be retrieved via the getState() method.
 *
 */
class Widget : public QWidget {
  Q_OBJECT

public:
  /**
   * @brief Configuration object for the Widget.
   *
   * Attributes:
   *  - size: The width/height of the widget in pixels.
   *  - layout: Whether this should be a single or dual pad joystick.
   */
  struct Config {
    std::size_t size = 250;
    Layout layout = Layout::DUAL;
    std::string topic = "/joy";
  };

  /**
   * @brief Construct a new Widget object
   * @param config Configuration parameters.
   * @param parent Pareent QWidget object.
   */
  Widget(const Config &config, QWidget *parent = nullptr);

  /**
   * @brief Deconstructor.
   */
  ~Widget() override;

  void setConfig(const Config &config);

  /**
   * @brief Get the state of the joystick.
   * @return The overall state of the buttons and axes.
   */
  JoystickState getState() const;

signals:
  /**
   * @brief Signal emitted when there is an update to the state of one of the UI elements,
   *
   * It does not contain any state information. The intention is that this triggers the gathering of state via the
   * getState() method.
   */
  void stateUpdated();

  /**
   * @brief Signal emitted when there is an update to the ROS topic and a reconnection has been requested.
   *
   * The intention is the a listener of this signal handles setting up a new Publisher on the specified topic.
   *
   * @param topic The name of the ROS topic.
   */
  void topicUpdated(const QString &topic);

private:
  QVBoxLayout *layout_;
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ros_virtual_joystick
