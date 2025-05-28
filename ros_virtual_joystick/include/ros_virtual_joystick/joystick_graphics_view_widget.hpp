#pragma once

#include <QGraphicsItem>
#include <QGraphicsTextItem>
#include <QGraphicsView>
#include <QWidget>
#include <vector>

namespace ros_virtual_joystick {

/**
 * @brief The JoystickGraphicsViewWidget class provides a graphical representation of a single 2D joystick. it is
 * written in Qt and so can be easily loaded into any Qt gui.
 *
 * The default is behaviour is for users to click in the display are and move the mouse around. The joystick will follow
 * the mouse. As the state of the GUI changes, this object emits a stateUpdated() signal, which can be used to inform
 * other objects that there has been a state change. The values for each axis are clamped to teh range (-1, 1). By
 * default, when you release the mouse, the joystick returns to (0, 0).
 *
 * There two main options:
 * - Hold position: Meaning that the joystick holds the last position rather than returning to (0, 0).
 * - Axis locking: This allow movement only on one of the two axis.
 */
class JoystickGraphicsViewWidget : public QGraphicsView {
  Q_OBJECT

public:
  /**
   * @brief Construct a new JoystickGraphicsViewWidget object.
   * @param parent QWidget parent, or a nullptr.
   * @param size The size of an edge of the widget.
   */
  explicit JoystickGraphicsViewWidget(QWidget *parent = nullptr, const int size = 250);

  /**
   * @brief Deconstructor.
   */
  ~JoystickGraphicsViewWidget();

  /**
   * @brief Get the state of the joystick axis.
   *
   * The returned values will be in the range (-1, 1), in the (x, y) format.
   *
   * @return The axis state.
   */
  std::vector<float> getState() const;

  /**
   * @brief Reset the joystick.
   *
   * This will reset so that the state values are (0, 0) and the GUI si also reset.
   */
  void reset();

  /**
   * @brief Return whether axis tracking is on/off.
   * @return true if the tracking is on, else false.
   */
  bool axisTracking() const;

  /**
   * @brief Return whether holding the position is on/off.
   * @return true is holding position is on, else false.
   */
  bool holdPosition() const;

  /**
   * @brief Set whether axis tracking should be on/off.
   * @param value The state value.
   */
  void setLockAxisTracking(const bool value);

  /**
   * @brief Set whether holding position should be on/off.
   * @param value The state value.
   */
  void setHoldPosition(const bool value);

  /**
   * @brief Update the state of the joystick.
   *
   * This allows for the state (including the GUI) to be set by an external entity.
   *
   * @param pos The new position to set, in pixel space, in the format (x, y).
   */
  void updateState(const QPointF &pos);

signals:
  /**
   * @brief Signal emitted when there is an update to the state of one of the UI elements,
   *
   * It does not contain any state information. The intention is that this triggers the gathering of state via the
   * getState() method.
   */
  void stateUpdated() const;

public slots:
  /**
   * @brief Toggle whether axis tracking is on/off.
   */
  void toggleLockAxisTracking();

  /**
   * @brief Toggle whether holding position is on/off.
   */
  void toggleHoldPosition();

protected:
  /**
   * @brief Callback when the mouse is pressed in the graphics view.
   *
   * This will trigger a GUI/state update.
   *
   * @param event Mouse event.
   */
  void mousePressEvent(QMouseEvent *event) override;

  /**
   * @brief Callback when the mouse is moved in the graphics view.
   *
   * This will trigger a GUI/state update, as long as the mouse has already been clicked.
   *
   * @param event Mouse event.
   */
  void mouseMoveEvent(QMouseEvent *event) override;

  /**
   * @brief Callback when the mouse is released in the graphics view.
   *
   * This will trigger a GUI/state update, as long as the mouse has already been clicked.
   *
   * @param event Mouse event.
   */
  void mouseReleaseEvent(QMouseEvent *event) override;

private:
  bool dragging_;
  bool hold_position_;
  bool axis_tracking_;

  QGraphicsScene *scene_;

  QGraphicsEllipseItem *background_circle_;
  QGraphicsEllipseItem *focus_circle_;

  // Guide lines and circle
  QGraphicsLineItem *vertical_line_;
  QGraphicsLineItem *horizontal_line_;
  QGraphicsEllipseItem *centre_circle_;

  // Axis labels
  QGraphicsTextItem *x_axis_label_;
  QGraphicsTextItem *y_axis_label_;

  QPointF state_;
};
}  // namespace ros_virtual_joystick
