#include <catch2/catch_all.hpp>
#include "widgets/topic_widget.hpp"

#include <QApplication>
#include <QSignalSpy>
#include <QTest>

namespace ros_virtual_joystick {
namespace widgets {

TEST_CASE("Test the TopicWidget class") {
  int argc = 0;
  QApplication app(argc, nullptr);  // Required by QWidget

  GIVEN("A widget") {
    TopicWidget widget(nullptr, "/test");

    THEN("The topic should be correct") { REQUIRE(widget.topic().toStdString() == "/test"); }

    WHEN("The button is clicked") {
      QSignalSpy spy(&widget, &TopicWidget::topicUpdated);
      REQUIRE(spy.isValid());
      widget.pushButton()->click();

      THEN("There should have been one signal emitted") { REQUIRE(spy.count() == 1); }
      THEN("The argument to the signal should be as expected") {
        QList<QVariant> args = spy.takeFirst();
        REQUIRE(args.at(0).toString() == "/test");
      }
    }

    WHEN("Setting a different topic") {
      widget.setTopic("new_topic");
      THEN("The topic should be updated") { REQUIRE(widget.topic() == "new_topic"); }
    }
  }
}

}  // namespace widgets
}  // namespace ros_virtual_joystick
