#include <gtest/gtest.h>

#include <limits>

#include "subsea_inspection/common/config_validation.hpp"

using subsea_inspection::RoiBoundingBox;
namespace config_validation = subsea_inspection::config_validation;

TEST(ConfigValidationTest, PositiveValuesMustBeFiniteAndGreaterThanZero) {
    EXPECT_TRUE(config_validation::is_finite_positive(0.1));
    EXPECT_FALSE(config_validation::is_finite_positive(0.0));
    EXPECT_FALSE(config_validation::is_finite_positive(-0.1));
    EXPECT_FALSE(config_validation::is_finite_positive(std::numeric_limits<double>::infinity()));
    EXPECT_FALSE(config_validation::is_finite_positive(std::numeric_limits<double>::quiet_NaN()));
}

TEST(ConfigValidationTest, NonnegativeValuesMustBeFinite) {
    EXPECT_TRUE(config_validation::is_finite_nonnegative(0.0));
    EXPECT_TRUE(config_validation::is_finite_nonnegative(1.0));
    EXPECT_FALSE(config_validation::is_finite_nonnegative(-0.1));
    EXPECT_FALSE(
        config_validation::is_finite_nonnegative(-std::numeric_limits<double>::infinity()));
    EXPECT_FALSE(
        config_validation::is_finite_nonnegative(std::numeric_limits<double>::quiet_NaN()));
}

TEST(ConfigValidationTest, RoiRequiresFiniteIncreasingBoundsOnEveryAxis) {
    EXPECT_TRUE(config_validation::is_valid_roi(RoiBoundingBox{}));

    RoiBoundingBox invalid_roi;
    invalid_roi.min_x = invalid_roi.max_x;
    EXPECT_FALSE(config_validation::is_valid_roi(invalid_roi));

    invalid_roi = RoiBoundingBox{};
    invalid_roi.min_y = invalid_roi.max_y;
    EXPECT_FALSE(config_validation::is_valid_roi(invalid_roi));

    invalid_roi = RoiBoundingBox{};
    invalid_roi.max_z = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(config_validation::is_valid_roi(invalid_roi));
}

TEST(ConfigValidationTest, PidGainsMustBeFiniteButMayBeNegative) {
    EXPECT_TRUE(config_validation::are_valid_pid_gains(-1.0, 0.0, 2.0));
    EXPECT_FALSE(
        config_validation::are_valid_pid_gains(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0));
    EXPECT_FALSE(
        config_validation::are_valid_pid_gains(0.0, std::numeric_limits<double>::infinity(), 0.0));
}

TEST(ConfigValidationTest, HeadingToleranceMustBePositiveAndAtMostHalfTurn) {
    EXPECT_FALSE(config_validation::is_valid_heading_tolerance(0.0));
    EXPECT_TRUE(config_validation::is_valid_heading_tolerance(20.0));
    EXPECT_TRUE(config_validation::is_valid_heading_tolerance(180.0));
    EXPECT_FALSE(config_validation::is_valid_heading_tolerance(180.1));
    EXPECT_FALSE(
        config_validation::is_valid_heading_tolerance(std::numeric_limits<double>::quiet_NaN()));
}

TEST(ConfigValidationTest, MissionWaypointRequiresFiniteCoordinatesAndValidRadiusAndDwell) {
    EXPECT_TRUE(config_validation::is_valid_mission_waypoint(1.0, 2.0, -3.0, 0.4, 0.0));
    EXPECT_FALSE(config_validation::is_valid_mission_waypoint(
        std::numeric_limits<double>::infinity(), 2.0, -3.0, 0.4, 0.0));
    EXPECT_FALSE(config_validation::is_valid_mission_waypoint(1.0, 2.0, -3.0, 0.0, 0.0));
    EXPECT_FALSE(config_validation::is_valid_mission_waypoint(1.0, 2.0, -3.0, 0.4, -0.1));
}

TEST(ConfigValidationTest, AvoidanceStrategyMustBeRecognized) {
    EXPECT_TRUE(config_validation::is_valid_avoidance_strategy("fly_over"));
    EXPECT_TRUE(config_validation::is_valid_avoidance_strategy("lateral_bypass"));
    EXPECT_FALSE(config_validation::is_valid_avoidance_strategy("flyover"));
}

TEST(ConfigValidationTest, InspectionSettingsMustHaveConsistentFiniteLimits) {
    EXPECT_TRUE(
        config_validation::is_valid_inspection_configuration(0.8, 0.1, 0.5, 1.2, 0.4, 30.0));
    EXPECT_FALSE(
        config_validation::is_valid_inspection_configuration(0.0, 0.1, 0.5, 1.2, 0.4, 30.0));
    EXPECT_FALSE(
        config_validation::is_valid_inspection_configuration(0.8, 0.0, 0.5, 1.2, 0.4, 30.0));
    EXPECT_FALSE(
        config_validation::is_valid_inspection_configuration(0.8, 0.1, 1.2, 0.5, 0.4, 30.0));
    EXPECT_FALSE(
        config_validation::is_valid_inspection_configuration(0.8, 0.1, 0.5, 1.2, -0.1, 30.0));
    EXPECT_FALSE(
        config_validation::is_valid_inspection_configuration(0.8, 0.1, 0.5, 1.2, 0.4, 180.1));
}
