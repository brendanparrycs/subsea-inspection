#include <algorithm>
#include <cmath>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "yaml-cpp/yaml.h"

#include "subsea_inspection/pipeline_tracker.hpp"
#include "subsea_inspection/geometry_utils.hpp"

using subsea_inspection::PipelineTracker;
using subsea_inspection::PipePoint3D;
using subsea_inspection::InspectionCriteria;
using subsea_inspection::TrackingResult;
using subsea_inspection::PipelineSegment;

/// @brief ROS 2 node tracking BlueROV2 inspection pose relative to continuous 3D pipeline geometry,
/// evaluating multi-criteria survey standards, and publishing RViz visualization markers.
class InspectionTracker: public rclcpp::Node {
public:
    /// @brief Initializes node, loads pipeline geometry from YAML, and sets up subscribers and publishers.
    InspectionTracker(): 
        Node("inspection_tracker"),
        mission_file_(""),
        odometry_topic_("/model/bluerov2_heavy/odometry"),
        standoff_height_(0.80),
        bin_resolution_(0.10),
        has_odometry_(false)
    {
        // Declare and get parameters
        this->declare_parameter<std::string>("mission_file", mission_file_);
        this->declare_parameter<std::string>("odometry_topic", odometry_topic_);
        this->declare_parameter<double>("standoff_height", standoff_height_);
        this->declare_parameter<double>("bin_resolution", bin_resolution_);
        this->declare_parameter<double>("min_standoff", 0.50);
        this->declare_parameter<double>("max_standoff", 1.20);
        this->declare_parameter<double>("max_cross_track", 0.40);
        this->declare_parameter<double>("max_heading_error_deg", 30.0);

        mission_file_ = this->get_parameter("mission_file").as_string();
        odometry_topic_ = this->get_parameter("odometry_topic").as_string();
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
            RCLCPP_WARN(this->get_logger(), "No 'mission_file' parameter provided. Tracker model is empty.");
        }

        // Standard reliable QoS matching Gazebo bridge publisher
        rclcpp::QoS qos_profile(10);
        qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
        qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);

        // Odometry subscription
        odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odometry_topic_,
            qos_profile,
            std::bind(&InspectionTracker::odometry_callback, this, std::placeholders::_1)
        );

        // Publishers for nearest pipe point and RViz visualization markers
        closest_point_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/inspection/closest_pipe_point",
            10
        );

        markers_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/inspection/markers",
            10
        );

        RCLCPP_INFO(
            this->get_logger(),
            "InspectionTracker initialized. Criteria: Standoff [%.2f..%.2f]m | CrossTrack: <=%.2fm | Heading: <=%.1f deg",
            criteria_.min_standoff_m,
            criteria_.max_standoff_m,
            criteria_.max_cross_track_m,
            criteria_.max_heading_error_deg
        );
    }

private:
    /// @brief Parses pipeline mission YAML and reconstructs seafloor pipeline centerline segments.
    /// @param filepath Absolute path to mission YAML file.
    void load_pipeline_from_yaml(const std::string &filepath) {
        if (!std::filesystem::exists(filepath)) {
            RCLCPP_ERROR(this->get_logger(), "Mission file not found: %s", filepath.c_str());
            return;
        }

        try {
            YAML::Node root = YAML::LoadFile(filepath);
            if (!root["mission"] || !root["mission"]["waypoints"]) {
                RCLCPP_ERROR(this->get_logger(), "YAML missing 'mission/waypoints' sequence: %s", filepath.c_str());
                return;
            }

            const auto waypoints = root["mission"]["waypoints"];
            tracker_->clear();

            for (size_t i = 0; i < waypoints.size(); ++i) {
                double wx = waypoints[i]["x"].as<double>(0.0);
                double wy = waypoints[i]["y"].as<double>(0.0);
                double wz = waypoints[i]["z"].as<double>(-2.0);
                std::string name = waypoints[i]["name"].as<std::string>("wp_" + std::to_string(i));

                // Reconstruct seabed pipe coordinates by subtracting inspection standoff altitude
                PipePoint3D pipe_vertex(wx, wy, wz - standoff_height_);
                tracker_->add_vertex(name, pipe_vertex, 0.25);
            }

            RCLCPP_INFO(
                this->get_logger(),
                "Loaded %zu pipeline segments (total length: %.2fm) from %s",
                tracker_->get_segment_count(),
                tracker_->get_total_pipeline_length_m(),
                filepath.c_str()
            );
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error parsing YAML mission file: %s", e.what());
        }
    }

    /// @brief Primary odometry callback evaluating pose against pipeline geometry.
    /// @param msg Shared pointer to incoming nav_msgs/msg/Odometry message.
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pos_.x = msg->pose.pose.position.x;
        current_pos_.y = msg->pose.pose.position.y;
        current_pos_.z = msg->pose.pose.position.z;

        // Convert Quaternion to Euler yaw
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        m.getRPY(roll, pitch, yaw);
        current_yaw_ = yaw;

        has_odometry_ = true;

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
            this->get_logger(),
            *this->get_clock(),
            1000,
            "[Inspection] Seg %zu/%zu ('%s') | e_cross: %.2fm (max %.2f) | Standoff: %.2fm [%.2f..%.2f] | "
            "YawErr: %.1f deg (max %.1f) | Inspected: %s | Coverage: %.1f%% (%.1f / %.1f m)",
            latest_result_.active_segment_index + 1,
            tracker_->get_segment_count(),
            latest_result_.active_segment_name.c_str(),
            latest_result_.cross_track_error_m,
            criteria_.max_cross_track_m,
            latest_result_.standoff_altitude_m,
            criteria_.min_standoff_m,
            criteria_.max_standoff_m,
            latest_result_.heading_error_deg,
            criteria_.max_heading_error_deg,
            (latest_result_.is_inspected ? "YES" : "NO"),
            tracker_->get_coverage_percentage(),
            tracker_->get_inspected_length_m(),
            tracker_->get_total_pipeline_length_m()
        );
    }

    /// @brief Constructs and publishes RViz visualization markers for pipeline centerline and dynamic orthogonal ray.
    /// @param header Header carrying timestamp and world frame ID.
    void publish_markers(const std_msgs::msg::Header &header) {
        visualization_msgs::msg::MarkerArray marker_array;

        // 1. Pipeline Centerline (Dark Slate Gray Tube)
        visualization_msgs::msg::Marker pipe_marker;
        pipe_marker.header = header;
        pipe_marker.header.frame_id = "world";
        pipe_marker.ns = "pipeline_centerline";
        pipe_marker.id = 0;
        pipe_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        pipe_marker.action = visualization_msgs::msg::Marker::ADD;
        pipe_marker.scale.x = 0.15; // 15cm thickness
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
        ray_marker.scale.x = 0.05; // 5cm line width

        // Green if within inspection qualification envelope; Red if criteria violated (e.g. fly-over detour)
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

    /// @brief Path to mission YAML file containing pipeline route definition.
    std::string mission_file_;

    /// @brief Input topic name for vehicle odometry.
    std::string odometry_topic_;

    /// @brief Nominal inspection altitude offset above the pipe in meters.
    double standoff_height_;

    /// @brief Discretization resolution in meters for coverage binning.
    double bin_resolution_;

    /// @brief Flag indicating whether at least one odometry sample has been received.
    bool has_odometry_;

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

/// @brief Main entry point: initializes rclcpp, spins the InspectionTracker node, and cleanly shuts down.
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<InspectionTracker>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
