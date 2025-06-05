#pragma once

#include <QHBoxLayout>
#include <QPushButton>
#include <QWidget>
#include <vector>

namespace ros_virtual_joystick {
namespace widgets {

/**
 * @brief The ButtonGroup class is the UI Element that contains the main buttons for the virtual joystick interface.
 *
 * Each button can be in one of two states pressed/down (1) and released/up (0). These buttons to not latch, so when the
 * mouse is pressed/released, the state changes. Watchers of this object can be notified about state changes via
 * connecting to the stateUpdated() signal, and any state change will cause the stateUpdated() signal to be emitted,
 */
class ButtonGroup : public QWidget {
  Q_OBJECT

public:
  /**
   * @brief Identifier for each button in the group.
   */
  enum ButtonId { A = 0, B, C, D, ButtonCount };

  /**
   * @brief Construct a new ButtonGroup object.
   * @param parent Parent QWidget. If set, this will be the parent of this object, and all the child UI elements.
   */
  explicit ButtonGroup(QWidget *parent = nullptr);

  /**
   * @brief Deconstructor.
   */
  ~ButtonGroup() override = default;

  QPushButton *getButton(const ButtonId id);

  /**
   * @brief Get the state of the Buttons in the UI group.
   *
   * The state is encoded in a vector, where button A is the first element, B the second, etc.
   *
   * @return Vector containing the state of each button.
   */
  std::vector<int> getState() const;

signals:
  /**
   * @brief Signal emitted when there is an update to the state of one of the UI elements,
   *
   * It does not contain any state information. The intention is that this triggers the gathering of state via the
   * getState() method.
   */
  void stateUpdated() const;

private slots:
  /**
   * @brief Qt Slot used as a callback when a UI element has been interacted with.
   *
   * The intention is that when UI element state changes, this callback emits the stateUpdated() signal.
   */
  void onButtonEvent();

private:
  QHBoxLayout *layout_;
  std::array<QPushButton *, ButtonCount> buttons_;
};

}  // namespace widgets
}  // namespace ros_virtual_joystick
