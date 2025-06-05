#pragma once

#include "ros_virtual_joystick/types.hpp"
#include "button_group_widget.hpp"
#include "joystick_graphics_view_widget.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QWidget>

namespace ros_virtual_joystick {

/**
 * @brief The JoystickWidget provides the Qt based GUI for a virtual/screen joystick.
 *
 * It contains both the XY axis pad, as well as the two buttons for controlling the axis lock and hold position
 * options.
 *
 * When there is an update to the XY pad, this widget will emit the statusUpdated() signal, which can be used to trigger
 * external objects to retrieve the new state via the getState() method.
 */
class JoystickWidget : public QWidget {
  Q_OBJECT

public:
  /**
   * @brief Construct a new JoystickWidget object.
   * @param parent QWidget parent, or a nullptr.
   * @param size The size of an edge of the widget.
   */
  explicit JoystickWidget(QWidget *parent, const std::size_t size);
  /**
   * @brief Deconstructor.
   */
  ~JoystickWidget() {}

  /**
   * @brief Get the state of all the UI joystick and button elements.
   * @return The overall joystick state.
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

private slots:
  /**
   * @brief Callback run when any of the UI elements has a state change.
   *
   * When called, this will emit the stateUpdated() signal.
   */
  void onUpdate();

private:
  QHBoxLayout *h_layout_;
  QVBoxLayout *v_layout_;
  JoystickGraphicsViewWidget *graphics_view_;
  QPushButton *reset_on_release_button_;
  QPushButton *lock_axis_tracking_button_;
};

}  // namespace ros_virtual_joystick
