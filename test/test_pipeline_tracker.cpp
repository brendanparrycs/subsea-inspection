#include <cmath>
#include <gtest/gtest.h>

#include "subsea_inspection/pipeline_tracker.hpp"

using namespace subsea_inspection;

/// @brief Verifies that an empty tracker safely returns default results without crashing.
TEST(PipelineTrackerTest, HandlesEmptyTracker) {
    PipelineTracker tracker;
    EXPECT_EQ(tracker.get_segment_count(), 0u);
    EXPECT_NEAR(tracker.get_total_pipeline_length_m(), 0.0, 1e-6);
    EXPECT_NEAR(tracker.get_coverage_percentage(), 0.0, 1e-6);

    const auto result = tracker.track_pose(PipePoint3D(0.0, 0.0, 0.0), 0.0);
    EXPECT_FALSE(result.is_inspected);
    EXPECT_EQ(result.active_segment_index, 0u);
}

/// @brief Verifies orthogonal projection, errors, and standoff along a straight segment.
TEST(PipelineTrackerTest, StraightSegmentTracking) {
    PipelineTracker tracker(0.10);
    // 10m straight segment along X from (0, 0, -2.8) to (10, 0, -2.8)
    tracker.add_vertex("v1", PipePoint3D(0.0, 0.0, -2.8));
    tracker.add_vertex("v2", PipePoint3D(10.0, 0.0, -2.8));

    EXPECT_EQ(tracker.get_segment_count(), 1u);
    EXPECT_NEAR(tracker.get_total_pipeline_length_m(), 10.0, 1e-6);

    // ROV hovering at (4.0, 0.15, -2.0) with yaw = 0 rad
    const auto result = tracker.track_pose(PipePoint3D(4.0, 0.15, -2.0), 0.0);

    EXPECT_EQ(result.active_segment_index, 0u);
    EXPECT_NEAR(result.projected_point.x, 4.0, 1e-6);
    EXPECT_NEAR(result.projected_point.y, 0.0, 1e-6);
    EXPECT_NEAR(result.projected_point.z, -2.8, 1e-6);
    EXPECT_NEAR(result.arc_length_on_segment_m, 4.0, 1e-6);
    EXPECT_NEAR(result.total_arc_length_m, 4.0, 1e-6);
    EXPECT_NEAR(result.cross_track_error_m, 0.15, 1e-6);
    EXPECT_NEAR(result.standoff_altitude_m, 0.8, 1e-6);  // -2.0 - (-2.8) = +0.8m
    EXPECT_NEAR(result.heading_error_deg, 0.0, 1e-6);
    EXPECT_TRUE(result.is_inspected);
}

/// @brief Verifies that projection clamps properly before the start and past the end of a segment.
TEST(PipelineTrackerTest, EndpointClamping) {
    PipelineTracker tracker(0.10);
    tracker.add_vertex("v1", PipePoint3D(2.0, 0.0, -2.8));
    tracker.add_vertex("v2", PipePoint3D(8.0, 0.0, -2.8));

    // Vehicle before segment start: X = 0.0
    const auto before = tracker.track_pose(PipePoint3D(0.0, 0.0, -2.0), 0.0);
    EXPECT_NEAR(before.projected_point.x, 2.0, 1e-6);
    EXPECT_NEAR(before.arc_length_on_segment_m, 0.0, 1e-6);

    // Vehicle past segment end: X = 12.0
    const auto after = tracker.track_pose(PipePoint3D(12.0, 0.0, -2.0), 0.0);
    EXPECT_NEAR(after.projected_point.x, 8.0, 1e-6);
    EXPECT_NEAR(after.arc_length_on_segment_m, 6.0, 1e-6);
}

/// @brief Verifies tracking and cumulative arc length across pipeline bends.
TEST(PipelineTrackerTest, MultiSegmentBendTracking) {
    PipelineTracker tracker(0.10);
    // Segment 1: (0, 0, -2.8) to (10, 0, -2.8) -> length = 10m, heading = 0 rad
    tracker.add_vertex("v1", PipePoint3D(0.0, 0.0, -2.8));
    tracker.add_vertex("v2", PipePoint3D(10.0, 0.0, -2.8));
    // Segment 2: (10, 0, -2.8) to (10, 10, -2.8) -> length = 10m, heading = +PI/2 rad
    tracker.add_vertex("v3", PipePoint3D(10.0, 10.0, -2.8));

    EXPECT_EQ(tracker.get_segment_count(), 2u);
    EXPECT_NEAR(tracker.get_total_pipeline_length_m(), 20.0, 1e-6);

    // Vehicle on Segment 2 at (10.05, 6.0, -2.0) with yaw = +PI/2
    const auto res = tracker.track_pose(PipePoint3D(10.05, 6.0, -2.0), M_PI / 2.0);

    EXPECT_EQ(res.active_segment_index, 1u);
    EXPECT_NEAR(res.projected_point.x, 10.0, 1e-6);
    EXPECT_NEAR(res.projected_point.y, 6.0, 1e-6);
    EXPECT_NEAR(res.arc_length_on_segment_m, 6.0, 1e-6);
    EXPECT_NEAR(res.total_arc_length_m, 16.0, 1e-6);  // 10m on seg1 + 6m on seg2
    EXPECT_NEAR(res.cross_track_error_m, 0.05, 1e-6);
    EXPECT_NEAR(res.heading_error_deg, 0.0, 1e-6);
    EXPECT_TRUE(res.is_inspected);
}

/// @brief Verifies that each quality criterion correctly gates inspection validity.
TEST(PipelineTrackerTest, InspectionCriteriaGating) {
    PipelineTracker tracker(0.10);
    tracker.add_vertex("v1", PipePoint3D(0.0, 0.0, -2.8));
    tracker.add_vertex("v2", PipePoint3D(10.0, 0.0, -2.8));

    InspectionCriteria criteria;
    criteria.min_standoff_m = 0.50;
    criteria.max_standoff_m = 1.20;
    criteria.max_cross_track_m = 0.40;
    criteria.max_heading_error_deg = 30.0;

    // 1. Ideal pass
    auto res = tracker.track_pose(PipePoint3D(5.0, 0.0, -2.0), 0.0, criteria);
    EXPECT_TRUE(res.is_standoff_valid);
    EXPECT_TRUE(res.is_cross_track_valid);
    EXPECT_TRUE(res.is_heading_valid);
    EXPECT_TRUE(res.is_inspected);

    // 2. High standoff (Fly-over obstacle at Z = -1.0 -> standoff = 1.8m > 1.2m)
    res = tracker.track_pose(PipePoint3D(5.0, 0.0, -1.0), 0.0, criteria);
    EXPECT_FALSE(res.is_standoff_valid);
    EXPECT_FALSE(res.is_inspected);

    // 3. Low standoff (Skimming pipe at Z = -2.5 -> standoff = 0.3m < 0.5m)
    res = tracker.track_pose(PipePoint3D(5.0, 0.0, -2.5), 0.0, criteria);
    EXPECT_FALSE(res.is_standoff_valid);
    EXPECT_FALSE(res.is_inspected);

    // 4. Excessive lateral cross-track drift (Y = 0.50m > 0.40m)
    res = tracker.track_pose(PipePoint3D(5.0, 0.50, -2.0), 0.0, criteria);
    EXPECT_FALSE(res.is_cross_track_valid);
    EXPECT_FALSE(res.is_inspected);

    // 5. Misaligned heading (45 deg > 30 deg)
    res = tracker.track_pose(PipePoint3D(5.0, 0.0, -2.0), 45.0 * M_PI / 180.0, criteria);
    EXPECT_FALSE(res.is_heading_valid);
    EXPECT_FALSE(res.is_inspected);
}

/// @brief Verifies that coverage accumulates monotonically as the vehicle traverses the pipeline.
TEST(PipelineTrackerTest, CoverageAccumulationAndReset) {
    PipelineTracker tracker(1.0);  // 1.0m resolution bins for easy counting
    tracker.add_vertex("v1", PipePoint3D(0.0, 0.0, -2.8));
    tracker.add_vertex("v2", PipePoint3D(10.0, 0.0, -2.8));  // 10 bins total

    EXPECT_NEAR(tracker.get_coverage_percentage(), 0.0, 1e-6);
    EXPECT_NEAR(tracker.get_inspected_length_m(), 0.0, 1e-6);

    // Visit first 5 meters with valid inspection
    for (double x = 0.5; x <= 4.5; x += 1.0) {
        tracker.track_pose(PipePoint3D(x, 0.0, -2.0), 0.0);
    }

    EXPECT_NEAR(tracker.get_coverage_percentage(), 50.0, 1e-6);
    EXPECT_NEAR(tracker.get_inspected_length_m(), 5.0, 1e-6);

    // Reset coverage
    tracker.reset_coverage();
    EXPECT_NEAR(tracker.get_coverage_percentage(), 0.0, 1e-6);
    EXPECT_NEAR(tracker.get_inspected_length_m(), 0.0, 1e-6);
}

/// @brief Verifies interoperability with Point3D from point_cloud_filter.hpp.
TEST(PipelineTrackerTest, Point3DInteroperability) {
    const Point3D p1(1.0, 2.0, 3.0);
    const PipePoint3D p2 = p1;

    EXPECT_DOUBLE_EQ(p2.x, 1.0);
    EXPECT_DOUBLE_EQ(p2.y, 2.0);
    EXPECT_DOUBLE_EQ(p2.z, 3.0);

    const Point3D p3 = p2;
    EXPECT_DOUBLE_EQ(p3.x, 1.0);
    EXPECT_DOUBLE_EQ(p3.y, 2.0);
    EXPECT_DOUBLE_EQ(p3.z, 3.0);
}
