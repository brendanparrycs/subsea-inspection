#include <gtest/gtest.h>

#include <cmath>

#include "subsea_inspection/pipeline_tracker.hpp"

using subsea_inspection::InspectionCriteria;
using subsea_inspection::PipelineTracker;
using subsea_inspection::PipePoint3D;
using subsea_inspection::Point3D;

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
    const PipePoint3D p2(p1);

    EXPECT_DOUBLE_EQ(p2.x, 1.0);
    EXPECT_DOUBLE_EQ(p2.y, 2.0);
    EXPECT_DOUBLE_EQ(p2.z, 3.0);

    const Point3D p3 = p2;
    EXPECT_DOUBLE_EQ(p3.x, 1.0);
    EXPECT_DOUBLE_EQ(p3.y, 2.0);
    EXPECT_DOUBLE_EQ(p3.z, 3.0);
}

/// @brief Verifies clustering of uninspected bins into distinct gaps with along-track coordinates.
TEST(PipelineTrackerTest, UninspectedGapsExtraction) {
    PipelineTracker tracker(0.10);  // 10cm bins
    // 10m straight segment
    tracker.add_vertex("seg_a", PipePoint3D(0.0, 0.0, -2.8));
    tracker.add_vertex("seg_b", PipePoint3D(10.0, 0.0, -2.8));

    // Initially with 0 coverage, entire 10m is one single gap
    auto initial_gaps = tracker.get_uninspected_gaps(0.15);
    ASSERT_EQ(initial_gaps.size(), 1u);
    EXPECT_NEAR(initial_gaps[0].start_s_m, 0.0, 1e-6);
    EXPECT_NEAR(initial_gaps[0].end_s_m, 10.0, 1e-6);
    EXPECT_NEAR(initial_gaps[0].length_m, 10.0, 1e-6);

    // Inspect: [0.0m .. 3.0m] (valid)
    for (double x = 0.05; x < 3.0; x += 0.08) {
        tracker.track_pose(PipePoint3D(x, 0.0, -2.0), 0.0);
    }
    // Fly-over obstacle: [3.0m .. 6.0m] at Z = -1.0 (invalid standoff = 1.8m > 1.2m)
    for (double x = 3.05; x < 6.0; x += 0.08) {
        tracker.track_pose(PipePoint3D(x, 0.0, -1.0), 0.0);
    }
    // Resume inspection: [6.0m .. 8.0m] (valid)
    for (double x = 6.05; x < 8.0; x += 0.08) {
        tracker.track_pose(PipePoint3D(x, 0.0, -2.0), 0.0);
    }
    // Leave [8.0m .. 10.0m] uninspected

    auto gaps = tracker.get_uninspected_gaps(0.15);
    ASSERT_EQ(gaps.size(), 2u);

    // Gap 1: obstacle fly-over region around 3.0m to 6.0m (length ~ 3.0m)
    EXPECT_EQ(gaps[0].gap_id, 1u);
    EXPECT_NEAR(gaps[0].start_s_m, 3.0, 0.15);
    EXPECT_NEAR(gaps[0].end_s_m, 6.0, 0.15);
    EXPECT_NEAR(gaps[0].length_m, 3.0, 0.20);

    // Gap 2: tail region from 8.0m to 10.0m (length ~ 2.0m)
    EXPECT_EQ(gaps[1].gap_id, 2u);
    EXPECT_NEAR(gaps[1].start_s_m, 8.0, 0.15);
    EXPECT_NEAR(gaps[1].end_s_m, 10.0, 0.15);
    EXPECT_NEAR(gaps[1].length_m, 2.0, 0.20);
}

/// @brief Verifies calculation of coverage metrics per individual pipeline segment.
TEST(PipelineTrackerTest, SegmentCoverageBreakdown) {
    PipelineTracker tracker(0.10);
    // Segment 1: 4m long
    tracker.add_vertex("pipe_sec_1", PipePoint3D(0.0, 0.0, -2.8));
    tracker.add_vertex("pipe_sec_2", PipePoint3D(4.0, 0.0, -2.8));
    // Segment 2: 6m long
    tracker.add_vertex("pipe_sec_3", PipePoint3D(10.0, 0.0, -2.8));

    EXPECT_EQ(tracker.get_segment_count(), 2u);

    // Fully inspect Segment 1 (0 to 4m)
    for (double x = 0.05; x < 4.0; x += 0.08) {
        tracker.track_pose(PipePoint3D(x, 0.0, -2.0), 0.0);
    }

    // Inspect half of Segment 2 (4m to 7m out of 4m to 10m)
    for (double x = 4.05; x < 7.0; x += 0.08) {
        tracker.track_pose(PipePoint3D(x, 0.0, -2.0), 0.0);
    }

    auto breakdown = tracker.get_segment_coverage_breakdown();
    ASSERT_EQ(breakdown.size(), 2u);

    // Segment 1: 100% coverage
    EXPECT_EQ(breakdown[0].index, 0u);
    EXPECT_EQ(breakdown[0].name, "pipe_sec_2");
    EXPECT_NEAR(breakdown[0].length_m, 4.0, 1e-6);
    EXPECT_NEAR(breakdown[0].inspected_length_m, 4.0, 0.10);
    EXPECT_NEAR(breakdown[0].coverage_percent, 100.0, 1.0);

    // Segment 2: ~50% coverage (3m of 6m)
    EXPECT_EQ(breakdown[1].index, 1u);
    EXPECT_EQ(breakdown[1].name, "pipe_sec_3");
    EXPECT_NEAR(breakdown[1].length_m, 6.0, 1e-6);
    EXPECT_NEAR(breakdown[1].inspected_length_m, 3.0, 0.15);
    EXPECT_NEAR(breakdown[1].coverage_percent, 50.0, 3.0);
}
