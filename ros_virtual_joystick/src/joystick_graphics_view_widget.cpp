#include "joystick_graphics_view_widget.hpp"
#include "utils.hpp"
#include <QMouseEvent>
#include <cmath>
#include <iostream>

namespace ros_virtual_joystick {

JoystickGraphicsViewWidget::JoystickGraphicsViewWidget(QWidget *parent, const int size) :
      QGraphicsView(parent), dragging_(false), hold_position_(false), axis_tracking_(false) {
  const int half_size = float(size) / 2.0f;

  setFixedSize(size, size);
  setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setAlignment(Qt::AlignLeft | Qt::AlignTop);

  scene_ = new QGraphicsScene(this);
  scene_->setBackgroundBrush(Qt::darkGray);
  scene_->setSceneRect(0, 0, size, size);
  setScene(scene_);

  // Setup the background circle with a gradient
  const int offset = 2;
  background_circle_ = new QGraphicsEllipseItem(offset, offset, size - (offset * 2), size - (offset * 2));
  QRadialGradient gradient(half_size, half_size, half_size);  // center (x, y), radius
  gradient.setColorAt(0.0, QColor(255, 255, 255));            // Light gray center
  gradient.setColorAt(1.0, QColor(220, 220, 220));            // Dark gray edge
  background_circle_->setBrush(QBrush(gradient));

  QPen pen(Qt::red);
  pen.setStyle(Qt::DashLine);
  pen.setWidth(1);

  vertical_line_ = new QGraphicsLineItem(half_size, 0, half_size, size);
  vertical_line_->setPen(pen);
  horizontal_line_ = new QGraphicsLineItem(0, half_size, size, half_size);
  horizontal_line_->setPen(pen);

  centre_circle_ = new QGraphicsEllipseItem(-5, -5, 10, 10);
  centre_circle_->setBrush(QBrush(Qt::red));
  centre_circle_->setPen(Qt::NoPen);
  centre_circle_->setPos(half_size, half_size);

  focus_circle_ = new QGraphicsEllipseItem(-15, -15, 30, 30);
  focus_circle_->setBrush(QBrush(Qt::darkGray));

  // Axis text labels
  x_axis_label_ = new QGraphicsTextItem("X");
  x_axis_label_->setPos(size - x_axis_label_->boundingRect().width(), half_size);

  y_axis_label_ = new QGraphicsTextItem("Y");
  y_axis_label_->setPos(half_size, 0);

  scene_->addItem(background_circle_);
  scene_->addItem(vertical_line_);
  scene_->addItem(horizontal_line_);
  scene_->addItem(centre_circle_);
  scene_->addItem(x_axis_label_);
  scene_->addItem(y_axis_label_);
  scene_->addItem(focus_circle_);

  reset();
}

JoystickGraphicsViewWidget::~JoystickGraphicsViewWidget() { scene_->clear(); }

std::vector<float> JoystickGraphicsViewWidget::getState() const {
  const float x = state_.x();
  const float y = state_.y();
  std::vector<float> state{x, y};
  return state;
}

void JoystickGraphicsViewWidget::reset() {
  const float size = width();
  QPointF reset_pos(size * 0.5, size * 0.5);
  updateState(reset_pos);
}

bool JoystickGraphicsViewWidget::axisTracking() const { return axis_tracking_; }

bool JoystickGraphicsViewWidget::holdPosition() const { return hold_position_; }

void JoystickGraphicsViewWidget::setLockAxisTracking(const bool value) { axis_tracking_ = value; }

void JoystickGraphicsViewWidget::setHoldPosition(const bool value) {
  hold_position_ = value;
  if (!hold_position_) {
    reset();
  }
}

void JoystickGraphicsViewWidget::toggleLockAxisTracking() {
  const bool state = !axisTracking();
  setLockAxisTracking(state);
}

void JoystickGraphicsViewWidget::toggleHoldPosition() {
  const bool state = !holdPosition();
  setHoldPosition(state);
}

void JoystickGraphicsViewWidget::updateState(const QPointF &pos) {
  // Get the normalised coordinate
  const float size = minimumWidth();
  QPointF pos_norm = utils::normaliseCoordinate(pos, 0.0f, size);
  pos_norm = utils::clampPositionToCircle(pos_norm);

  QPointF marker_pos;

  if (!axis_tracking_) {
    state_ = pos_norm;
    const float x = ((pos_norm.x() + 1.0) * 0.5) * size;
    const float y = ((pos_norm.y() + 1.0) * 0.5) * size;
    marker_pos.setX(x);
    marker_pos.setY(y);
  } else {
    const QPointF pos = utils::calculateAxisLockPosition(pos_norm);
    state_ = pos;
    const float x = ((pos.x() + 1.0) * 0.5) * size;
    const float y = ((pos.y() + 1.0) * 0.5) * size;
    marker_pos.setX(x);
    marker_pos.setY(y);
  }

  focus_circle_->setPos(marker_pos);

  // Emit the signal that the state has been stateUpdated.
  emit stateUpdated();
}

void JoystickGraphicsViewWidget::mousePressEvent(QMouseEvent *event) {
  QGraphicsView::mousePressEvent(event);

  if (dragging_) {
    return;
  }

  dragging_ = true;
  updateState(event->pos());
}

void JoystickGraphicsViewWidget::mouseMoveEvent(QMouseEvent *event) {
  QGraphicsView::mouseMoveEvent(event);

  if (!dragging_) {
    return;
  }

  dragging_ = true;
  updateState(event->pos());
}

void JoystickGraphicsViewWidget::mouseReleaseEvent(QMouseEvent *event) {
  QGraphicsView::mouseReleaseEvent(event);

  if (!dragging_) {
    return;
  }

  dragging_ = false;

  if (!hold_position_) {
    reset();
  }
}

}  // namespace ros_virtual_joystick
