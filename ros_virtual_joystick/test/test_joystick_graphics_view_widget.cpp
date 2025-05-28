#include <catch2/catch_all.hpp>
#include <QApplication>
#include <QSignalSpy>
#include <QTest>
#include <iostream>

#include "ros_virtual_joystick/joystick_graphics_view_widget.hpp"

namespace ros_virtual_joystick {

TEST_CASE("JoystickGraphicsViewWidget class") {
  int argc = 0;
  QApplication app(argc, nullptr);

  JoystickGraphicsViewWidget widget(nullptr, 400);

  SECTION("Test the class state upon initialisation") {
    THEN("The state should be a vector of zeros") {
      const auto state = widget.getState();
      const std::vector<float> exp(2, 0.0);
      REQUIRE(state == exp);
    }

    THEN("The axis tracking should be false") {
      const bool state = widget.axisTracking();
      REQUIRE_FALSE(state);
    }

    THEN("The hold position should be false") {
      const bool state = widget.holdPosition();
      REQUIRE_FALSE(state);
    }
  }

  SECTION("Update the state") {
    REQUIRE_FALSE(widget.axisTracking());
    REQUIRE_FALSE(widget.holdPosition());

    WHEN("The updateState() method is called with a position of (400.0, 200.0)") {
      QSignalSpy spy(&widget, SIGNAL(stateUpdated()));
      REQUIRE(spy.isValid());

      const QPointF coord(400.0f, 200.0f);
      widget.updateState(coord);

      THEN("A signal should have been emitted") { REQUIRE(spy.count() == 1); }
      THEN("The state should have been stateUpdated") {
        const auto state = widget.getState();
        REQUIRE_THAT(state[0], Catch::Matchers::WithinAbs(1.0, 0.001));
        REQUIRE_THAT(state[1], Catch::Matchers::WithinAbs(0.0, 0.001));
      }
    }

    WHEN("The updateState() method is called with a position of (200.0, 400.0)") {
      QSignalSpy spy(&widget, SIGNAL(stateUpdated()));
      REQUIRE(spy.isValid());

      const QPointF coord(200.0f, 400.0f);
      widget.updateState(coord);

      THEN("A signal should have been emitted") { REQUIRE(spy.count() == 1); }
      THEN("The state should have been stateUpdated") {
        const auto state = widget.getState();
        REQUIRE_THAT(state[0], Catch::Matchers::WithinAbs(0.0, 0.001));
        REQUIRE_THAT(state[1], Catch::Matchers::WithinAbs(1.0, 0.001));
      }
    }
  }

  SECTION("Toggle hold position on/off") {
    REQUIRE_FALSE(widget.axisTracking());
    REQUIRE_FALSE(widget.holdPosition());

    widget.toggleHoldPosition();

    THEN("The hold position should be turned on") {
      const bool state = widget.holdPosition();
      REQUIRE(state);
    }

    WHEN("The updateState() method is called") {
      const QPointF coord(200.0f, 400.0f);
      widget.updateState(coord);

      WHEN("The hold position is set to False") {
        widget.setHoldPosition(false);

        THEN("The state should have been stateUpdated to (0.0, 0.0)") {
          const auto state = widget.getState();
          REQUIRE_THAT(state[0], Catch::Matchers::WithinAbs(0.0, 0.001));
          REQUIRE_THAT(state[1], Catch::Matchers::WithinAbs(0.0, 0.001));
        }
      }
    }
  }

  SECTION("Toggle axis lock on/off") {
    REQUIRE_FALSE(widget.axisTracking());
    REQUIRE_FALSE(widget.holdPosition());

    widget.toggleLockAxisTracking();

    THEN("The axis lock should be turned on") {
      const bool state = widget.axisTracking();
      REQUIRE(state);
    }

    WHEN("The updateState() method is called with (400, 300)") {
      const QPointF coord(400.0f, 300.0f);
      widget.updateState(coord);

      THEN("The state should have been stateUpdated") {
        const auto state = widget.getState();
        REQUIRE(state[0] > 0.0f);
        REQUIRE_THAT(state[1], Catch::Matchers::WithinAbs(0.0, 0.001));
      }
    }

    WHEN("The updateState() method is called with (300, 400)") {
      const QPointF coord(300.0f, 400.0f);
      widget.updateState(coord);

      THEN("The state should have been stateUpdated") {
        const auto state = widget.getState();
        REQUIRE_THAT(state[0], Catch::Matchers::WithinAbs(0.0, 0.001));
        REQUIRE(state[1] > 0.0f);
      }
    }
  }
}

}  // namespace ros_virtual_joystick
