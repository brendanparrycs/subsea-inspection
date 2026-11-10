#ifndef SUBSEA_INSPECTION_PIPELINE_TRACKER_HPP_
#define SUBSEA_INSPECTION_PIPELINE_TRACKER_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "subsea_inspection/common/constants.hpp"
#include "subsea_inspection/geometry_utils.hpp"
#include "subsea_inspection/point_cloud_filter.hpp"

namespace subsea_inspection
{

/// Default distance resolution used to discretize pipeline inspection coverage.
inline constexpr double kDefaultPipelineBinResolutionM = 0.10;
inline constexpr double kDefaultPipelineRadiusM = 0.25;
inline constexpr double kDefaultMinimumGapLengthM = 0.15;

/// @brief Represents a 3D coordinate point or spatial vector.
struct PipePoint3D
{
  /// @brief X coordinate in meters.
  double x{0.0};

  /// @brief Y coordinate in meters.
  double y{0.0};

  /// @brief Z coordinate in meters.
  double z{0.0};

  PipePoint3D() = default;
  PipePoint3D(double px, double py, double pz)
  : x(px),
    y(py),
    z(pz)
  {}

  /// @brief Implicit converting constructor from Point3D for stack-wide interoperability.
  /// @param pt Spatial 3D point.
  PipePoint3D(const Point3D &pt)
  : x(pt.x),
    y(pt.y),
    z(pt.z)
  {}

  /// @brief Implicit conversion operator to Point3D.
  operator Point3D() const
  {
    return Point3D(x, y, z);
  }

  /// @brief Computes 3D Euclidean distance to another point.
  /// @param other Destination point.
  /// @return Spatial distance in meters.
  double distance_to(const PipePoint3D &other) const
  {
    double dx = other.x - x;
    double dy = other.y - y;
    double dz = other.z - z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  /// @brief Checks if point coordinates are valid finite numbers.
  /// @return True if x, y, and z are finite.
  bool is_valid() const
  {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
  }
};

/// @brief A linear cylindrical pipeline segment connecting two 3D vertices on the seabed.
struct PipelineSegment
{
  /// @brief Unique descriptive identifier for the segment.
  std::string name{};

  /// @brief Starting 3D vertex coordinates on pipe centerline.
  PipePoint3D start{};

  /// @brief Ending 3D vertex coordinates on pipe centerline.
  PipePoint3D end{};

  /// @brief Outer radius of the cylindrical pipeline in meters.
  double radius{kDefaultPipelineRadiusM};

  /// @brief 3D Euclidean length of the segment in meters.
  double length{0.0};

  /// @brief Tangent direction heading of the segment in radians [-PI, +PI].
  double heading_rad{0.0};

  PipelineSegment() = default;

  /// @brief Constructs a pipeline segment connecting start and end vertices.
  /// @param seg_name Unique descriptive identifier for the segment.
  /// @param p1 Starting 3D vertex coordinates on pipe centerline.
  /// @param p2 Ending 3D vertex coordinates on pipe centerline.
  /// @param r Outer radius of the pipe cylinder in meters.
  PipelineSegment(const std::string &seg_name, const PipePoint3D &p1, const PipePoint3D &p2,
                  double r = kDefaultPipelineRadiusM)
  : name(seg_name),
    start(p1),
    end(p2),
    radius(r)
  {
    double dx = end.x - start.x;
    double dy = end.y - start.y;
    double dz = end.z - start.z;
    length = std::sqrt(dx * dx + dy * dy + dz * dz);
    heading_rad = (length > 1e-4) ? std::atan2(dy, dx) : 0.0;
  }
};

/// @brief Sensor envelope criteria for qualifying valid subsea inspection coverage.
struct InspectionCriteria
{
  /// @brief Minimum allowable vertical standoff clearance above the pipe centerline in meters.
  double min_standoff_m{0.50};

  /// @brief Maximum allowable vertical standoff clearance above the pipe centerline in meters.
  double max_standoff_m{1.20};

  /// @brief Maximum allowable lateral horizontal deviation from the pipe centerline in meters.
  double max_cross_track_m{0.40};

  /// @brief Maximum allowable heading angular deviation relative to the pipe segment tangent in
  /// degrees.
  double max_heading_error_deg{30.0};
};

/// @brief Instantaneous tracking metrics and inspection qualification result.
struct TrackingResult
{
  /// @brief Index of the nearest active pipeline segment in the pipeline model.
  std::size_t active_segment_index{0};

  /// @brief Identifier of the nearest active pipeline segment.
  std::string active_segment_name{};

  /// @brief Orthogonal projection point on the active pipe centerline closest to the vehicle.
  PipePoint3D projected_point{};

  /// @brief Scalar arc-length distance along the active segment measured from segment start in
  /// meters.
  double arc_length_on_segment_m{0.0};

  /// @brief Cumulative arc-length distance along the entire pipeline measured from pipeline origin
  /// in meters.
  double total_arc_length_m{0.0};

  /// @brief Horizontal lateral cross-track deviation perpendicular to the pipe centerline in
  /// meters.
  double cross_track_error_m{0.0};

  /// @brief Vertical standoff clearance above the pipe centerline (ROV Z minus pipe Z) in meters.
  double standoff_altitude_m{0.0};

  /// @brief True 3D Euclidean spatial distance from the vehicle position to the pipe centerline in
  /// meters.
  double spatial_distance_m{0.0};

  /// @brief Absolute angular deviation between vehicle yaw heading and pipe tangent heading in
  /// degrees.
  double heading_error_deg{0.0};

  /// @brief True if vertical standoff altitude satisfies the inspection criteria envelope.
  bool is_standoff_valid{false};

  /// @brief True if lateral cross-track error is within the maximum allowable threshold.
  bool is_cross_track_valid{false};

  /// @brief True if vehicle yaw heading is aligned within the maximum allowable heading tolerance.
  bool is_heading_valid{false};

  /// @brief True if standoff, cross-track, and heading criteria are all simultaneously satisfied.
  bool is_inspected{false};
};

/// @brief Represents an uninspected gap section along the pipeline.
struct InspectionGap
{
  /// @brief Sequential gap index number (1-based).
  std::size_t gap_id{0};

  /// @brief Starting cumulative arc-length position along the pipeline in meters.
  double start_s_m{0.0};

  /// @brief Ending cumulative arc-length position along the pipeline in meters.
  double end_s_m{0.0};

  /// @brief Total spatial length of the uninspected gap in meters.
  double length_m{0.0};

  /// @brief Index of the pipeline segment containing or nearest to the gap start.
  std::size_t segment_index{0};

  /// @brief Descriptive identifier of the pipeline segment containing the gap start.
  std::string segment_name{};
};

/// @brief Inspection coverage analytics for an individual pipeline segment.
struct SegmentCoverage
{
  /// @brief Index of the pipeline segment (0-based).
  std::size_t index{0};

  /// @brief Descriptive identifier of the segment.
  std::string name{};

  /// @brief Total length of the segment in meters.
  double length_m{0.0};

  /// @brief Inspected length of the segment in meters.
  double inspected_length_m{0.0};

  /// @brief Coverage percentage for this segment [0.0, 100.0].
  double coverage_percent{0.0};
};

/// @brief Manages 3D pipeline geometry, orthogonal vehicle tracking, and inspection coverage.
class PipelineTracker
{
public:
  /// @brief Constructs a pipeline tracker with a specified spatial bin resolution.
  /// @param bin_resolution_m Discretization arc-length resolution in meters for coverage tracking.
  explicit PipelineTracker(double bin_resolution_m = kDefaultPipelineBinResolutionM)
  : bin_resolution_m_(bin_resolution_m),
    total_pipeline_length_(0.0)
  {}

  /// @brief Registers a new linear pipeline segment.
  /// @param segment Pipeline segment with start, end, and radius.
  void add_segment(const PipelineSegment &segment)
  {
    segments_.push_back(segment);
    total_pipeline_length_ += segment.length;
    rebuild_bins();
  }

  /// @brief Adds a new vertex, automatically connecting it to the previous vertex.
  /// @param vertex_name Unique identifier for the segment.
  /// @param pt 3D coordinates of the new pipeline vertex.
  /// @param radius Pipe radius in meters.
  void add_vertex(const std::string &vertex_name, const PipePoint3D &pt,
                  double radius = kDefaultPipelineRadiusM)
  {
    if (!vertices_.empty()) {
      PipelineSegment seg(vertex_name, vertices_.back(), pt, radius);
      add_segment(seg);
    }
    vertices_.push_back(pt);
  }

  /// @brief Clears all pipeline segments, vertices, and accumulated coverage data.
  void clear()
  {
    vertices_.clear();
    segments_.clear();
    total_pipeline_length_ = 0.0;
    bins_.clear();
  }

  /// @brief Resets all coverage inspection bins to uninspected.
  void reset_coverage()
  {
    std::fill(bins_.begin(), bins_.end(), false);
  }

  /// @brief Evaluates vehicle spatial pose against the pipeline geometry.
  /// @param rov_pos Current 3D position of the ROV in world coordinates.
  /// @param rov_yaw_rad Current heading of the ROV in radians [-PI, +PI].
  /// @param criteria Quality threshold envelope for inspection qualification.
  /// @return TrackingResult containing orthogonal projection, error metrics, and qualification
  /// flags.
  TrackingResult track_pose(const PipePoint3D &rov_pos, double rov_yaw_rad,
                            const InspectionCriteria &criteria = InspectionCriteria())
  {
    TrackingResult result;
    if (segments_.empty()) {
      return result;
    }

    double best_dist_sq = std::numeric_limits<double>::infinity();
    std::size_t best_idx = 0;
    PipePoint3D best_proj;
    double best_s = 0.0;
    double accumulated_len = 0.0;
    double best_accumulated_start = 0.0;

    for (std::size_t i = 0; i < segments_.size(); ++i) {
      const auto &seg = segments_[i];
      if (seg.length <= 1e-4) {
        accumulated_len += seg.length;
        continue;
      }

      double dx = seg.end.x - seg.start.x;
      double dy = seg.end.y - seg.start.y;
      double dz = seg.end.z - seg.start.z;

      // Unit tangent along segment
      double tx = dx / seg.length;
      double ty = dy / seg.length;
      double tz = dz / seg.length;

      // Vector from start vertex to ROV position
      double to_rov_x = rov_pos.x - seg.start.x;
      double to_rov_y = rov_pos.y - seg.start.y;
      double to_rov_z = rov_pos.z - seg.start.z;

      // Scalar dot product projection
      double s = to_rov_x * tx + to_rov_y * ty + to_rov_z * tz;
      double s_clamped = std::clamp(s, 0.0, seg.length);

      PipePoint3D proj(seg.start.x + s_clamped * tx, seg.start.y + s_clamped * ty,
                       seg.start.z + s_clamped * tz);

      double d_sq = (rov_pos.x - proj.x) * (rov_pos.x - proj.x) +
                    (rov_pos.y - proj.y) * (rov_pos.y - proj.y) +
                    (rov_pos.z - proj.z) * (rov_pos.z - proj.z);

      if (d_sq < best_dist_sq) {
        best_dist_sq = d_sq;
        best_idx = i;
        best_proj = proj;
        best_s = s_clamped;
        best_accumulated_start = accumulated_len;
      }

      accumulated_len += seg.length;
    }

    const auto &active_seg = segments_[best_idx];
    result.active_segment_index = best_idx;
    result.active_segment_name = active_seg.name;
    result.projected_point = best_proj;
    result.arc_length_on_segment_m = best_s;
    result.total_arc_length_m = best_accumulated_start + best_s;

    // Horizontal lateral cross-track error
    double dx_lat = rov_pos.x - best_proj.x;
    double dy_lat = rov_pos.y - best_proj.y;
    result.cross_track_error_m = std::hypot(dx_lat, dy_lat);

    // Vertical standoff altitude (ROV z - pipe z)
    result.standoff_altitude_m = rov_pos.z - best_proj.z;

    // 3D spatial Euclidean distance
    result.spatial_distance_m = std::sqrt(best_dist_sq);

    // Heading deviation wrapped to [-PI, +PI]
    double yaw_diff = rov_yaw_rad - active_seg.heading_rad;
    yaw_diff = geometry::wrap_angle(yaw_diff);
    result.heading_error_deg = std::abs(yaw_diff) * constants::kRadiansToDegrees;

    // Multi-criteria inspection qualification
    result.is_standoff_valid = (result.standoff_altitude_m >= criteria.min_standoff_m) &&
                               (result.standoff_altitude_m <= criteria.max_standoff_m);
    result.is_cross_track_valid = (result.cross_track_error_m <= criteria.max_cross_track_m);
    result.is_heading_valid = (result.heading_error_deg <= criteria.max_heading_error_deg);

    result.is_inspected =
      result.is_standoff_valid && result.is_cross_track_valid && result.is_heading_valid;

    // Mark discrete coverage bin if criteria are met
    if (result.is_inspected && !bins_.empty()) {
      std::size_t bin_idx = static_cast<std::size_t>(result.total_arc_length_m / bin_resolution_m_);
      if (bin_idx < bins_.size()) {
        bins_[bin_idx] = true;
      }
    }

    return result;
  }

  /// @brief Computes overall percentage of the pipeline successfully inspected.
  /// @return Percentage of pipeline length certified [0.0, 100.0].
  double get_coverage_percentage() const
  {
    if (bins_.empty())
      return 0.0;
    std::size_t inspected_count = 0;
    for (bool b : bins_) {
      if (b)
        inspected_count++;
    }
    return (static_cast<double>(inspected_count) / static_cast<double>(bins_.size())) * 100.0;
  }

  /// @brief Computes cumulative inspected pipeline length in meters.
  /// @return Length of inspected pipeline sections in meters.
  double get_inspected_length_m() const
  {
    if (bins_.empty() || total_pipeline_length_ <= 0.0)
      return 0.0;
    std::size_t inspected_count = 0;
    for (bool b : bins_) {
      if (b)
        inspected_count++;
    }
    double inspected = static_cast<double>(inspected_count) * bin_resolution_m_;
    return std::min(total_pipeline_length_, inspected);
  }

  /// @brief Gets total cumulative length of all registered pipeline segments.
  /// @return Total pipeline length in meters.
  double get_total_pipeline_length_m() const
  {
    return total_pipeline_length_;
  }

  /// @brief Gets the number of registered pipeline segments.
  /// @return Total segment count.
  std::size_t get_segment_count() const
  {
    return segments_.size();
  }

  /// @brief Gets the spatial discretization bin resolution.
  /// @return Resolution in meters.
  double get_bin_resolution_m() const
  {
    return bin_resolution_m_;
  }

  /// @brief Returns read-only reference to all registered pipeline segments.
  /// @return Vector of PipelineSegment structs.
  const std::vector<PipelineSegment> &get_segments() const
  {
    return segments_;
  }

  /// @brief Returns read-only reference to discretized coverage bins.
  /// @return Vector of booleans indicating inspection status per bin.
  const std::vector<bool> &get_coverage_bins() const
  {
    return bins_;
  }

  /// @brief Finds the segment index and name containing a given cumulative arc-length.
  /// @param s Cumulative arc-length in meters.
  /// @return Pair of segment index and segment descriptive name.
  std::pair<std::size_t, std::string> find_segment_for_arc_length(double s) const
  {
    if (segments_.empty()) {
      return {0, ""};
    }
    double accumulated = 0.0;
    for (std::size_t i = 0; i < segments_.size(); ++i) {
      accumulated += segments_[i].length;
      if (s <= accumulated || i == segments_.size() - 1) {
        return {i, segments_[i].name};
      }
    }
    return {segments_.size() - 1, segments_.back().name};
  }

  /// @brief Extracts continuous uninspected gaps along the pipeline from the discrete coverage
  /// bins.
  /// @param min_gap_length_m Minimum gap length in meters to qualify as a reported gap.
  /// @return Ordered list of InspectionGap structs.
  std::vector<InspectionGap> get_uninspected_gaps(
    double min_gap_length_m = kDefaultMinimumGapLengthM) const
  {
    std::vector<InspectionGap> gaps;
    if (bins_.empty() || total_pipeline_length_ <= 0.0) {
      return gaps;
    }

    std::size_t gap_count = 0;
    bool in_gap = false;
    std::size_t gap_start_bin = 0;

    for (std::size_t i = 0; i < bins_.size(); ++i) {
      if (!bins_[i]) {
        if (!in_gap) {
          in_gap = true;
          gap_start_bin = i;
        }
      } else {
        if (in_gap) {
          in_gap = false;
          double start_s = static_cast<double>(gap_start_bin) * bin_resolution_m_;
          double end_s = static_cast<double>(i) * bin_resolution_m_;
          end_s = std::min(end_s, total_pipeline_length_);
          double len = end_s - start_s;

          if (len >= min_gap_length_m) {
            gap_count++;
            InspectionGap gap;
            gap.gap_id = gap_count;
            gap.start_s_m = start_s;
            gap.end_s_m = end_s;
            gap.length_m = len;
            auto [seg_idx, seg_name] = find_segment_for_arc_length(start_s);
            gap.segment_index = seg_idx;
            gap.segment_name = seg_name;
            gaps.push_back(gap);
          }
        }
      }
    }

    // Handle trailing gap extending to end of pipe
    if (in_gap) {
      double start_s = static_cast<double>(gap_start_bin) * bin_resolution_m_;
      double end_s = total_pipeline_length_;
      double len = end_s - start_s;
      if (len >= min_gap_length_m) {
        gap_count++;
        InspectionGap gap;
        gap.gap_id = gap_count;
        gap.start_s_m = start_s;
        gap.end_s_m = end_s;
        gap.length_m = len;
        auto [seg_idx, seg_name] = find_segment_for_arc_length(start_s);
        gap.segment_index = seg_idx;
        gap.segment_name = seg_name;
        gaps.push_back(gap);
      }
    }

    return gaps;
  }

  /// @brief Computes coverage metrics broken down by individual pipeline segment.
  /// @return Ordered list of SegmentCoverage structs for each registered segment.
  std::vector<SegmentCoverage> get_segment_coverage_breakdown() const
  {
    std::vector<SegmentCoverage> breakdown;
    if (segments_.empty()) {
      return breakdown;
    }

    double accumulated_s = 0.0;
    for (std::size_t i = 0; i < segments_.size(); ++i) {
      const auto &seg = segments_[i];
      SegmentCoverage sc;
      sc.index = i;
      sc.name = seg.name;
      sc.length_m = seg.length;

      double seg_start_s = accumulated_s;
      double seg_end_s = accumulated_s + seg.length;

      if (seg.length > 1e-4 && !bins_.empty()) {
        std::size_t start_bin = static_cast<std::size_t>(seg_start_s / bin_resolution_m_);
        std::size_t end_bin = static_cast<std::size_t>(std::ceil(seg_end_s / bin_resolution_m_));
        start_bin = std::min(start_bin, bins_.size());
        end_bin = std::min(end_bin, bins_.size());

        std::size_t seg_bins_total = (end_bin > start_bin) ? (end_bin - start_bin) : 0;
        std::size_t seg_bins_inspected = 0;

        for (std::size_t b = start_bin; b < end_bin; ++b) {
          if (bins_[b]) {
            seg_bins_inspected++;
          }
        }

        if (seg_bins_total > 0) {
          sc.coverage_percent =
            (static_cast<double>(seg_bins_inspected) / static_cast<double>(seg_bins_total)) * 100.0;
          sc.inspected_length_m =
            std::min(seg.length, static_cast<double>(seg_bins_inspected) * bin_resolution_m_);
        }
      }

      breakdown.push_back(sc);
      accumulated_s += seg.length;
    }

    return breakdown;
  }

private:
  /// @brief Rebuilds the discrete coverage bin array when segments are added or modified.
  void rebuild_bins()
  {
    if (bin_resolution_m_ <= 0.0 || total_pipeline_length_ <= 0.0)
      return;
    std::size_t num_bins =
      static_cast<std::size_t>(std::ceil(total_pipeline_length_ / bin_resolution_m_));
    bins_.assign(num_bins, false);
  }

  /// @brief Spatial arc-length resolution in meters per discretized coverage cell.
  double bin_resolution_m_{0.10};

  /// @brief Total cumulative 3D length of all registered pipeline segments in meters.
  double total_pipeline_length_{0.0};

  /// @brief Ordered sequence of 3D vertices defining the pipeline path.
  std::vector<PipePoint3D> vertices_{};

  /// @brief Ordered array of linear cylindrical pipeline segments.
  std::vector<PipelineSegment> segments_{};

  /// @brief Discretized boolean grid representing inspection coverage per resolution cell.
  std::vector<bool> bins_{};
};

}  // namespace subsea_inspection

#endif  // SUBSEA_INSPECTION_PIPELINE_TRACKER_HPP_
