#include <catch2/catch_all.hpp>
#include "ros_virtual_joystick/utils.hpp"

namespace ros_virtual_joystick {

TEST_CASE("Test the utils::normaliseValue() function") {
  GIVEN("A min max value of [0.0, 1.0)") {
    const float min_value = 0.0f;
    const float max_value = 1.0f;

    WHEN("The value to normalise is 0.0") {
      const float res = utils::normaliseValue(0.0f, min_value, max_value);
      REQUIRE(res == Catch::Approx(-1.0f));
    }

    WHEN("The value to normalise is 1.0") {
      const float res = utils::normaliseValue(1.0f, min_value, max_value);
      REQUIRE(res == Catch::Approx(1.0f));
    }

    WHEN("The value to normalise is 0.5") {
      const float res = utils::normaliseValue(0.5f, min_value, max_value);
      REQUIRE(res == Catch::Approx(0.0f));
    }

    WHEN("The value to normalise is 1.5") {
      const float res = utils::normaliseValue(1.5f, min_value, max_value);
      REQUIRE(res == Catch::Approx(1.0f));
    }

    WHEN("The value to normalise is -1.5") {
      const float res = utils::normaliseValue(-1.5f, min_value, max_value);
      REQUIRE(res == Catch::Approx(-1.0f));
    }
  }

  GIVEN("A min value greater than the max value") {
    const float min_value = 1.0;
    const float max_value = 0.0;

    WHEN("Calling the method") {
      THEN("The function call should thrown an exception") {
        REQUIRE_THROWS(utils::normaliseValue(0.0, min_value, max_value));
      }
    }
  }
}

TEST_CASE("Test the utils::normaliseCoordinate() function") {
  GIVEN("A min/max value of [0, 100]") {
    const float min_value = 0.0f;
    const float max_value = 100.0f;

    WHEN("Calling the function on a coordinate (0, 0)") {
      const QPointF coord(0.0f, 0.0f);
      const auto result = utils::normaliseCoorindate(coord, min_value, max_value);
      THEN("The normalised coordinate should be (-1.0, -1.0)") { REQUIRE(result == QPointF(-1.0f, -1.0f)); }
    }

    WHEN("Calling the function on a coordinate (50, 50)") {
      const QPointF coord(50.0f, 50.0f);
      const auto result = utils::normaliseCoorindate(coord, min_value, max_value);
      THEN("The normalised coordinate should be (0.0, 0.0)") { REQUIRE(result == QPointF(0.0f, 0.0f)); }
    }

    WHEN("Calling the function on a coordinate (100, 100)") {
      const QPointF coord(100.0f, 100.0f);
      const auto result = utils::normaliseCoorindate(coord, min_value, max_value);
      THEN("The normalised coordinate should be (1.0, 1.0)") { REQUIRE(result == QPointF(1.0f, 1.0f)); }
    }
  }
}

TEST_CASE("Test the utils::calculateAxisLockPosition() function") {
  GIVEN("A coordinate of (0.0, 0.0)") {
    const QPointF coord(0.0f, 0.0f);
    const auto result = utils::calculateAxisLockPosition(coord);
    const QPointF exp(0.0f, 0.0f);
    REQUIRE(result == exp);
  }

  GIVEN("A coordinate of (1, 5)") {
    const QPointF coord(1.0f, 5.0f);
    const auto result = utils::calculateAxisLockPosition(coord);
    const QPointF exp(0.0f, 5.0f);
    REQUIRE(result == exp);
  }

  GIVEN("A coordinate of (5, 1)") {
    const QPointF coord(5.0f, 1.0f);
    const auto result = utils::calculateAxisLockPosition(coord);
    const QPointF exp(5.0f, 0.0f);
    REQUIRE(result == exp);
  }
}

TEST_CASE("Test the utils::clampPositionToCircle() function") {
  GIVEN("A coordinate of (0.0, 0.0)") {
    const QPointF coord(0.0f, 0.0f);
    const auto result = utils::clampPositionToCircle(coord);
    const QPointF exp(0.0f, 0.0f);
    const float eps = 0.0001;
    REQUIRE_THAT(exp.x(), Catch::Matchers::WithinAbs(result.x(), eps));
    REQUIRE_THAT(exp.y(), Catch::Matchers::WithinAbs(result.y(), eps));
  }

  GIVEN("A coordinate of (0, -5)") {
    const QPointF coord(0.0f, -5.0f);
    const auto result = utils::clampPositionToCircle(coord);
    const QPointF exp(0.0f, -1.0f);
    const float eps = 0.0001;
    REQUIRE_THAT(exp.x(), Catch::Matchers::WithinAbs(result.x(), eps));
    REQUIRE_THAT(exp.y(), Catch::Matchers::WithinAbs(result.y(), eps));
  }

  GIVEN("A coordinate of (0, 5)") {
    const QPointF coord(0.0f, 5.0f);
    const auto result = utils::clampPositionToCircle(coord);
    const QPointF exp(0.0f, 1.0f);
    const float eps = 0.0001;
    REQUIRE_THAT(exp.x(), Catch::Matchers::WithinAbs(result.x(), eps));
    REQUIRE_THAT(exp.y(), Catch::Matchers::WithinAbs(result.y(), eps));
  }

  GIVEN("A coordinate of (-5, 0)") {
    const QPointF coord(-5.0f, 0.0f);
    const auto result = utils::clampPositionToCircle(coord);
    const QPointF exp(-1.0f, 0.0f);
    const float eps = 0.0001;
    REQUIRE_THAT(exp.x(), Catch::Matchers::WithinAbs(result.x(), eps));
    REQUIRE_THAT(exp.y(), Catch::Matchers::WithinAbs(result.y(), eps));
  }

  GIVEN("A coordinate of (5, 0)") {
    const QPointF coord(5.0f, 0.0f);
    const auto result = utils::clampPositionToCircle(coord);
    const QPointF exp(1.0f, 0.0f);
    const float eps = 0.0001;
    REQUIRE_THAT(exp.x(), Catch::Matchers::WithinAbs(result.x(), eps));
    REQUIRE_THAT(exp.y(), Catch::Matchers::WithinAbs(result.y(), eps));
  }
}

}  // namespace ros_virtual_joystick
