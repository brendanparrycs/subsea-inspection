#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "subsea_inspection/geometry_utils.hpp"
#include "subsea_inspection/pipeline_tracker.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "yaml-cpp/yaml.h"

using subsea_inspection::InspectionCriteria;
using subsea_inspection::PipelineSegment;
using subsea_inspection::PipelineTracker;
using subsea_inspection::PipePoint3D;
using subsea_inspection::TrackingResult;

namespace
{
constexpr double kDefaultStandoffHeightM = 0.80;
constexpr double kDefaultPipelineVertexDepthM = -2.0;
constexpr double kPipelineMarkerDiameterM = 0.15;
constexpr double kProjectionRayWidthM = 0.05;
}  // namespace

/// @brief ROS 2 node tracking BlueROV2 inspection pose relative to continuous 3D pipeline geometry,
/// evaluating multi-criteria survey standards, and publishing RViz visualization markers.
class InspectionTracker : public rclcpp::Node
{
public:
  /// @brief Initializes node, loads pipeline geometry from YAML, and sets up subscribers and
  /// publishers.
  InspectionTracker()
  : Node("inspection_tracker"),
    mission_file_(""),
    odometry_topic_("/model/bluerov2_heavy/odometry"),
    standoff_height_(kDefaultStandoffHeightM),
    bin_resolution_(subsea_inspection::kDefaultPipelineBinResolutionM)
  {
    const InspectionCriteria default_criteria{};

    // Declare and get parameters
    this->declare_parameter<std::string>("mission_file", mission_file_);
    this->declare_parameter<std::string>("odom_topic", odometry_topic_);
    this->declare_parameter<std::string>("mission_complete_topic", "/mission/complete");
    this->declare_parameter<std::string>("report_directory", "reports");
    this->declare_parameter<double>("standoff_height", standoff_height_);
    this->declare_parameter<double>("bin_resolution", bin_resolution_);
    this->declare_parameter<double>("min_standoff", default_criteria.min_standoff_m);
    this->declare_parameter<double>("max_standoff", default_criteria.max_standoff_m);
    this->declare_parameter<double>("max_cross_track", default_criteria.max_cross_track_m);
    this->declare_parameter<double>("max_heading_error_deg",
                                    default_criteria.max_heading_error_deg);

    mission_file_ = this->get_parameter("mission_file").as_string();
    odometry_topic_ = this->get_parameter("odom_topic").as_string();
    const auto mission_complete_topic = this->get_parameter("mission_complete_topic").as_string();
    report_directory_ = this->get_parameter("report_directory").as_string();
    standoff_height_ = this->get_parameter("standoff_height").as_double();
    bin_resolution_ = this->get_parameter("bin_resolution").as_double();

    criteria_.min_standoff_m = this->get_parameter("min_standoff").as_double();
    criteria_.max_standoff_m = this->get_parameter("max_standoff").as_double();
    criteria_.max_cross_track_m = this->get_parameter("max_cross_track").as_double();
    criteria_.max_heading_error_deg = this->get_parameter("max_heading_error_deg").as_double();

    // Initialize geometric tracker
    tracker_ = std::make_unique<PipelineTracker>(bin_resolution_);

    // Load pipeline geometry from mission YAML
    if (!mission_file_.empty()) {
      load_pipeline_from_yaml(mission_file_);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "No 'mission_file' parameter provided. Tracker model is empty.");
    }

    // Standard reliable QoS matching Gazebo bridge publisher
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);

    // Odometry subscription
    odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, qos_profile,
      std::bind(&InspectionTracker::odometry_callback, this, std::placeholders::_1));

    // Publishers for nearest pipe point and RViz visualization markers
    closest_point_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "/inspection/closest_pipe_point", 10);

    markers_publisher_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("/inspection/markers", 10);

    // Subscription for mission completion to trigger post-survey report
    mission_complete_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      mission_complete_topic, 10,
      std::bind(&InspectionTracker::mission_complete_callback, this, std::placeholders::_1));

    // Service to trigger report generation on demand
    report_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/inspection/generate_report", std::bind(&InspectionTracker::handle_generate_report, this,
                                               std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
                "InspectionTracker initialized. Criteria: Standoff [%.2f..%.2f]m | CrossTrack: "
                "<=%.2fm | Heading: <=%.1f deg",
                criteria_.min_standoff_m, criteria_.max_standoff_m, criteria_.max_cross_track_m,
                criteria_.max_heading_error_deg);
  }

private:
  /// @brief Parses pipeline mission YAML and reconstructs seafloor pipeline centerline segments.
  /// @param filepath Absolute path to mission YAML file.
  void load_pipeline_from_yaml(const std::string &filepath)
  {
    if (!std::filesystem::exists(filepath)) {
      RCLCPP_ERROR(this->get_logger(), "Mission file not found: %s", filepath.c_str());
      return;
    }

    try {
      YAML::Node root = YAML::LoadFile(filepath);
      if (!root["mission"] || !root["mission"]["waypoints"]) {
        RCLCPP_ERROR(this->get_logger(), "YAML missing 'mission/waypoints' sequence: %s",
                     filepath.c_str());
        return;
      }

      const auto waypoints = root["mission"]["waypoints"];
      tracker_->clear();

      for (std::size_t i = 0; i < waypoints.size(); ++i) {
        double wx = waypoints[i]["x"].as<double>(0.0);
        double wy = waypoints[i]["y"].as<double>(0.0);
        double wz = waypoints[i]["z"].as<double>(kDefaultPipelineVertexDepthM);
        std::string name = waypoints[i]["name"].as<std::string>("wp_" + std::to_string(i));

        // Reconstruct seabed pipe coordinates by subtracting inspection standoff altitude
        PipePoint3D pipe_vertex(wx, wy, wz - standoff_height_);
        tracker_->add_vertex(name, pipe_vertex);
      }

      RCLCPP_INFO(this->get_logger(), "Loaded %zu pipeline segments (total length: %.2fm) from %s",
                  tracker_->get_segment_count(), tracker_->get_total_pipeline_length_m(),
                  filepath.c_str());
    }
    catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error parsing YAML mission file: %s", e.what());
    }
  }

  /// @brief Primary odometry callback evaluating pose against pipeline geometry.
  /// @param msg Shared pointer to incoming nav_msgs/msg/Odometry message.
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pos_.x = msg->pose.pose.position.x;
    current_pos_.y = msg->pose.pose.position.y;
    current_pos_.z = msg->pose.pose.position.z;

    // Convert Quaternion to Euler yaw
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    m.getRPY(roll, pitch, yaw);
    current_yaw_ = yaw;

    // Perform orthogonal tracking projection and criteria qualification
    latest_result_ = tracker_->track_pose(current_pos_, current_yaw_, criteria_);

    // Publish closest point on pipe centerline
    geometry_msgs::msg::PointStamped proj_msg;
    proj_msg.header = msg->header;
    proj_msg.header.frame_id = "world";
    proj_msg.point.x = latest_result_.projected_point.x;
    proj_msg.point.y = latest_result_.projected_point.y;
    proj_msg.point.z = latest_result_.projected_point.z;
    closest_point_publisher_->publish(proj_msg);

    // Publish visualization markers for RViz
    publish_markers(msg->header);

    // Throttled 1 Hz telemetry log
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "[Inspection] Seg %zu/%zu ('%s') | e_cross: %.2fm (max %.2f) | Standoff: %.2fm [%.2f..%.2f] "
      "| "
      "YawErr: %.1f deg (max %.1f) | Inspected: %s | Coverage: %.1f%% (%.1f / %.1f m)",
      latest_result_.active_segment_index + 1, tracker_->get_segment_count(),
      latest_result_.active_segment_name.c_str(), latest_result_.cross_track_error_m,
      criteria_.max_cross_track_m, latest_result_.standoff_altitude_m, criteria_.min_standoff_m,
      criteria_.max_standoff_m, latest_result_.heading_error_deg, criteria_.max_heading_error_deg,
      (latest_result_.is_inspected ? "YES" : "NO"), tracker_->get_coverage_percentage(),
      tracker_->get_inspected_length_m(), tracker_->get_total_pipeline_length_m());
  }

  /// @brief Constructs and publishes RViz visualization markers for pipeline centerline and dynamic
  /// orthogonal ray.
  /// @param header Header carrying timestamp and world frame ID.
  void publish_markers(const std_msgs::msg::Header &header)
  {
    visualization_msgs::msg::MarkerArray marker_array;

    // 1. Pipeline Centerline (Dark Slate Gray Tube)
    visualization_msgs::msg::Marker pipe_marker;
    pipe_marker.header = header;
    pipe_marker.header.frame_id = "world";
    pipe_marker.ns = "pipeline_centerline";
    pipe_marker.id = 0;
    pipe_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    pipe_marker.action = visualization_msgs::msg::Marker::ADD;
    pipe_marker.scale.x = kPipelineMarkerDiameterM;
    pipe_marker.color.r = 0.35f;
    pipe_marker.color.g = 0.40f;
    pipe_marker.color.b = 0.45f;
    pipe_marker.color.a = 0.90f;

    const auto &segments = tracker_->get_segments();
    if (!segments.empty()) {
      geometry_msgs::msg::Point p_first;
      p_first.x = segments.front().start.x;
      p_first.y = segments.front().start.y;
      p_first.z = segments.front().start.z;
      pipe_marker.points.push_back(p_first);

      for (const auto &seg : segments) {
        geometry_msgs::msg::Point p_end;
        p_end.x = seg.end.x;
        p_end.y = seg.end.y;
        p_end.z = seg.end.z;
        pipe_marker.points.push_back(p_end);
      }
    }
    marker_array.markers.push_back(pipe_marker);

    // 2. Dynamic Orthogonal Projection Ray from ROV to Pipe Centerline
    visualization_msgs::msg::Marker ray_marker;
    ray_marker.header = header;
    ray_marker.header.frame_id = "world";
    ray_marker.ns = "orthogonal_projection_ray";
    ray_marker.id = 1;
    ray_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    ray_marker.action = visualization_msgs::msg::Marker::ADD;
    ray_marker.scale.x = kProjectionRayWidthM;

    // Green if within inspection qualification envelope; Red if criteria violated (e.g. fly-over
    // detour)
    if (latest_result_.is_inspected) {
      ray_marker.color.r = 0.10f;
      ray_marker.color.g = 0.90f;
      ray_marker.color.b = 0.20f;
      ray_marker.color.a = 0.90f;
    } else {
      ray_marker.color.r = 0.95f;
      ray_marker.color.g = 0.15f;
      ray_marker.color.b = 0.15f;
      ray_marker.color.a = 0.90f;
    }

    geometry_msgs::msg::Point rov_pt, proj_pt;
    rov_pt.x = current_pos_.x;
    rov_pt.y = current_pos_.y;
    rov_pt.z = current_pos_.z;
    proj_pt.x = latest_result_.projected_point.x;
    proj_pt.y = latest_result_.projected_point.y;
    proj_pt.z = latest_result_.projected_point.z;

    ray_marker.points.push_back(rov_pt);
    ray_marker.points.push_back(proj_pt);
    marker_array.markers.push_back(ray_marker);

    markers_publisher_->publish(marker_array);
  }

  /// @brief Callback when mission planner announces mission completion.
  /// @param msg Message containing completion boolean.
  void mission_complete_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data && !report_generated_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Mission completion signal received. Generating post-survey inspection report...");
      generate_survey_report();
    }
  }

  /// @brief Service handler to trigger post-survey report generation on demand.
  /// @param request Trigger request.
  /// @param response Response with success flag and filepath message.
  void handle_generate_report(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::string report_path;
    bool ok = generate_survey_report(&report_path);
    response->success = ok;
    response->message = ok ? ("Report generated at: " + report_path) : "Failed to generate report";
  }

  /// @brief Generates comprehensive YAML inspection report and prints executive summary to console.
  /// @param out_filepath Optional pointer to store the generated report's absolute file path.
  /// @return True if report was successfully written.
  bool generate_survey_report(std::string *out_filepath = nullptr)
  {
    if (!tracker_ || tracker_->get_segment_count() == 0) {
      RCLCPP_WARN(this->get_logger(), "Cannot generate report: tracker model is empty.");
      return false;
    }

    // Ensure reports directory exists
    std::filesystem::path dir_path(report_directory_);
    if (!std::filesystem::exists(dir_path)) {
      std::filesystem::create_directories(dir_path);
    }

    // Timestamp string
    auto now = std::chrono::system_clock::now();
    std::time_t time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now{};
    localtime_r(&time_t_now, &tm_now);

    char time_str[64];
    std::strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", &tm_now);

    std::string filename = "survey_report_" + std::string(time_str) + ".yaml";
    std::filesystem::path full_path = dir_path / filename;

    double total_len = tracker_->get_total_pipeline_length_m();
    double inspected_len = tracker_->get_inspected_length_m();
    double coverage_pct = tracker_->get_coverage_percentage();
    double uninspected_len = std::max(0.0, total_len - inspected_len);

    auto gaps = tracker_->get_uninspected_gaps();
    auto breakdown = tracker_->get_segment_coverage_breakdown();

    // Write YAML report file
    std::ofstream ofs(full_path);
    if (!ofs.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open report file for writing: %s",
                   full_path.string().c_str());
      return false;
    }

    ofs << "# Autonomous Subsea Inspection — Post-Survey Quality Report\n";
    ofs << "survey_report:\n";
    ofs << "  timestamp: \"" << std::string(time_str) << "\"\n";
    ofs << "  mission_file: \"" << mission_file_ << "\"\n";
    ofs << "  pipeline_metrics:\n";
    ofs << "    total_length_m: " << std::fixed << std::setprecision(2) << total_len << "\n";
    ofs << "    inspected_length_m: " << inspected_len << "\n";
    ofs << "    uninspected_length_m: " << uninspected_len << "\n";
    ofs << "    certified_coverage_percent: " << std::setprecision(1) << coverage_pct << "\n";
    ofs << "    bin_resolution_m: " << std::setprecision(2) << bin_resolution_ << "\n";
    ofs << "    total_segments: " << tracker_->get_segment_count() << "\n";
    ofs << "\n";
    ofs << "  inspection_criteria:\n";
    ofs << "    min_standoff_m: " << criteria_.min_standoff_m << "\n";
    ofs << "    max_standoff_m: " << criteria_.max_standoff_m << "\n";
    ofs << "    max_cross_track_m: " << criteria_.max_cross_track_m << "\n";
    ofs << "    max_heading_error_deg: " << criteria_.max_heading_error_deg << "\n";
    ofs << "\n";
    ofs << "  uninspected_gaps:\n";
    if (gaps.empty()) {
      ofs << "    []\n";
    } else {
      for (const auto &gap : gaps) {
        ofs << "    - gap_id: " << gap.gap_id << "\n";
        ofs << "      start_s_m: " << gap.start_s_m << "\n";
        ofs << "      end_s_m: " << gap.end_s_m << "\n";
        ofs << "      length_m: " << gap.length_m << "\n";
        ofs << "      segment_index: " << gap.segment_index + 1 << "\n";
        ofs << "      segment_name: \"" << gap.segment_name << "\"\n";
      }
    }
    ofs << "\n";
    ofs << "  segment_breakdown:\n";
    for (const auto &sc : breakdown) {
      ofs << "    - index: " << sc.index + 1 << "\n";
      ofs << "      name: \"" << sc.name << "\"\n";
      ofs << "      length_m: " << sc.length_m << "\n";
      ofs << "      inspected_length_m: " << sc.inspected_length_m << "\n";
      ofs << "      coverage_percent: " << sc.coverage_percent << "\n";
    }
    ofs.close();

    // Print formatted executive summary report to console
    RCLCPP_INFO(this->get_logger(),
                "======================================================================");
    RCLCPP_INFO(this->get_logger(),
                "            AUTONOMOUS SUBSEA PIPELINE SURVEY REPORT                 ");
    RCLCPP_INFO(this->get_logger(),
                "======================================================================");
    RCLCPP_INFO(this->get_logger(), " Total Pipeline Length:       %6.2f m (%zu segments)",
                total_len, tracker_->get_segment_count());
    RCLCPP_INFO(this->get_logger(), " Certified Inspected Length:  %6.2f m", inspected_len);
    RCLCPP_INFO(this->get_logger(), " Uninspected Gap Length:      %6.2f m", uninspected_len);
    RCLCPP_INFO(this->get_logger(), " Overall Certified Coverage:  %6.1f %%", coverage_pct);
    RCLCPP_INFO(this->get_logger(), " Discretization Resolution:   %6.2f m", bin_resolution_);
    RCLCPP_INFO(this->get_logger(),
                "----------------------------------------------------------------------");
    RCLCPP_INFO(this->get_logger(), " UNINSPECTED GAP REGISTRY (%zu Gaps >= %.2f m):", gaps.size(),
                subsea_inspection::kDefaultMinimumGapLengthM);
    if (gaps.empty()) {
      RCLCPP_INFO(this->get_logger(), "  None! Full 100%% contiguous coverage certified.");
    } else {
      for (const auto &g : gaps) {
        RCLCPP_INFO(this->get_logger(),
                    "  [Gap %zu] s: %5.2fm -> %5.2fm | Len: %5.2fm | Seg %zu ('%s')", g.gap_id,
                    g.start_s_m, g.end_s_m, g.length_m, g.segment_index + 1,
                    g.segment_name.c_str());
      }
    }
    RCLCPP_INFO(this->get_logger(),
                "----------------------------------------------------------------------");
    RCLCPP_INFO(this->get_logger(), " SEGMENT COVERAGE BREAKDOWN:");
    for (const auto &sc : breakdown) {
      RCLCPP_INFO(this->get_logger(), "  #%-2zu '%-18s': %5.2fm | Inspected: %5.2fm | %5.1f %%",
                  sc.index + 1, sc.name.c_str(), sc.length_m, sc.inspected_length_m,
                  sc.coverage_percent);
    }
    RCLCPP_INFO(this->get_logger(),
                "======================================================================");
    RCLCPP_INFO(this->get_logger(), " Detailed Survey Report Saved: %s",
                full_path.string().c_str());
    RCLCPP_INFO(this->get_logger(),
                "======================================================================");

    report_generated_ = true;
    if (out_filepath) {
      *out_filepath = std::filesystem::absolute(full_path).string();
    }
    return true;
  }

  /// @brief Target directory for generated survey reports.
  std::string report_directory_{"reports"};

  /// @brief Flag indicating whether a post-survey report has already been exported.
  bool report_generated_{false};

  /// @brief Subscription handle for mission completion notifications.
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_complete_subscription_;

  /// @brief Service server handle for on-demand report generation.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr report_service_;

  /// @brief Path to mission YAML file containing pipeline route definition.
  std::string mission_file_;

  /// @brief Input topic name for vehicle odometry.
  std::string odometry_topic_;

  /// @brief Nominal inspection altitude offset above the pipe in meters.
  double standoff_height_;

  /// @brief Discretization resolution in meters for coverage binning.
  double bin_resolution_;

  /// @brief Current 3D position of the ROV in world coordinates.
  PipePoint3D current_pos_{};

  /// @brief Current vehicle yaw heading in radians.
  double current_yaw_{0.0};

  /// @brief Threshold envelope for qualifying valid inspection coverage.
  InspectionCriteria criteria_{};

  /// @brief Geometric pipeline tracking and coverage engine.
  std::unique_ptr<PipelineTracker> tracker_{nullptr};

  /// @brief Latest computed tracking and qualification result.
  TrackingResult latest_result_{};

  /// @brief Subscription handle for incoming Odometry messages.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;

  /// @brief Publisher handle for the closest orthogonal point on the pipe centerline.
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr closest_point_publisher_;

  /// @brief Publisher handle for RViz visualization markers.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_publisher_;
};

/// @brief Main entry point: initializes rclcpp, spins the InspectionTracker node, and cleanly shuts
/// down.
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<InspectionTracker>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
