#include <catch2/catch_test_macros.hpp>
#include <QApplication>
#include <QSignalSpy>
#include <QTest>

#include "widgets/button_group_widget.hpp"

namespace ros_virtual_joystick {

TEST_CASE("ButtonGroup emits signal and updates state correctly") {
  int argc = 0;
  QApplication app(argc, nullptr);  // Required by QWidget

  ros_virtual_joystick::ButtonGroup group;

  SECTION("Initial state is all released") {
    auto state = group.getState();
    REQUIRE(state == std::vector<int>{0, 0, 0, 0});
  }

  SECTION("Button click changes state and emits signal") {
    QSignalSpy spy(&group, SIGNAL(stateUpdated()));
    REQUIRE(spy.isValid());

    auto button = group.getButton(ButtonGroup::ButtonId::A);

    WHEN("The first button is clicked") {
      // Simulate button click
      auto button = group.getButton(ButtonGroup::ButtonId::A);
      button->click();

      // Check that signal was emitted at least once
      THEN("There should be two signals emitted") { REQUIRE(spy.count() == 2); }
    }
    WHEN("The first button is pressed") {
      QTest::mousePress(button, Qt::LeftButton);
      THEN("The state should be a vector [1, 0, 0, 0]") {
        const auto state = group.getState();
        const auto exp = std::vector<int>{1, 0, 0, 0};
        REQUIRE(state == exp);
      }

      WHEN("The button is released") {
        QTest::mouseRelease(button, Qt::LeftButton);
        THEN("The state should be a vector of zeros") {
          const auto state = group.getState();
          const auto exp = std::vector<int>{0, 0, 0, 0};
          REQUIRE(state == exp);
        }
      }
    }
  }
}

}  // namespace ros_virtual_joystick
