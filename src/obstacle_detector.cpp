#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "subsea_inspection/point_cloud_filter.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using subsea_inspection::filter_corridor;
using subsea_inspection::FilterResult;
using subsea_inspection::Point3D;
using subsea_inspection::RoiBoundingBox;

namespace
{
constexpr double kObstacleMarkerDiameterM = 0.25;
}  // namespace

/// @brief ROS 2 node subscribe to live 3D depth point clouds, filtering for a forward
/// safety corridor, and publishing obstacle alerts and RViz visualization markers.
class ObstacleDetector : public rclcpp::Node
{
public:
  ObstacleDetector()
  : Node("obstacle_detector"),
    point_cloud_topic_("/model/bluerov2_heavy/depth_camera/points"),
    detection_threshold_(subsea_inspection::kDefaultDetectionThresholdM),
    closest_point_topic_("/obstacle/closest_point")
  {
    const RoiBoundingBox default_roi{};

    // Declare and get parameters
    this->declare_parameter<std::string>("point_cloud_topic", point_cloud_topic_);
    this->declare_parameter<double>("detection_threshold", detection_threshold_);
    this->declare_parameter<double>("min_x", default_roi.min_x);
    this->declare_parameter<double>("max_x", default_roi.max_x);
    this->declare_parameter<double>("min_y", default_roi.min_y);
    this->declare_parameter<double>("max_y", default_roi.max_y);
    this->declare_parameter<double>("min_z", default_roi.min_z);
    this->declare_parameter<double>("max_z", default_roi.max_z);
    this->declare_parameter<std::string>("closest_point_topic", closest_point_topic_);

    point_cloud_topic_ = this->get_parameter("point_cloud_topic").as_string();
    detection_threshold_ = this->get_parameter("detection_threshold").as_double();
    roi_.min_x = this->get_parameter("min_x").as_double();
    roi_.max_x = this->get_parameter("max_x").as_double();
    roi_.min_y = this->get_parameter("min_y").as_double();
    roi_.max_y = this->get_parameter("max_y").as_double();
    roi_.min_z = this->get_parameter("min_z").as_double();
    roi_.max_z = this->get_parameter("max_z").as_double();
    closest_point_topic_ = this->get_parameter("closest_point_topic").as_string();

    // Standard sensor data QoS profile matching Gazebo bridge publisher
    rclcpp::QoS qos_profile(5);
    qos_profile.best_effort();
    qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);

    // Publishers for neareset obstacle position and RViz visualization markers
    closest_point_publisher_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>(closest_point_topic_, 10);

    markers_publisher_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("/obstacle/markers", 10);

    // Subscribe to depth camera point cloud
    point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      point_cloud_topic_, qos_profile,
      std::bind(&ObstacleDetector::point_cloud_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "ObstacleDetector initialized. Subscribed to %s. Corridor X: [%.2f, %.2f], Y: "
                "[%.2f, %.2f], Z: [%.2f, %.2f]",
                point_cloud_topic_.c_str(), roi_.min_x, roi_.max_x, roi_.min_y, roi_.max_y,
                roi_.min_z, roi_.max_z);
  }

private:
  /// @brief Callback for incoming PointCloud2 messages. Unpacks points, filters corridor, and
  /// publishes alerts.
  /// @param msg Shared pointer to incoming sensor_msgs/msg/PointCloud2 message.
  void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::vector<Point3D> points;
    points.reserve(msg->width * msg->height);

    // Fast zero-copy iterator over packed FLOAT32 coordinates
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      points.emplace_back(*iter_x, *iter_y, *iter_z);
    }

    // Run safety corridor filter
    FilterResult result = filter_corridor(points, roi_, detection_threshold_);

    // Publish closest obstacle point
    if (result.obstacle_detected) {
      geometry_msgs::msg::PointStamped closest_msg;
      closest_msg.header = msg->header;
      closest_msg.point.x = result.closest_point.x;
      closest_msg.point.y = result.closest_point.y;
      closest_msg.point.z = result.closest_point.z;
      closest_point_publisher_->publish(closest_msg);

      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "[Obstacle Alert] Distance: %.2fm | Closest: [X: %.2f, Y: %.2f, Z: "
                           "%.2f]m | Corridor points: %zu",
                           result.min_distance, result.closest_point.x, result.closest_point.y,
                           result.closest_point.z, result.corridor_points.size());
    } else {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "[Corridor Clear] Total cloud points: %zu | Nearest candidate: %.2fm",
                           points.size(), result.min_distance);
    }

    // Publish visualization markers for RViz
    publish_markers(msg->header, result);
  }

  /// @brief Constructs and publishes RViz visualization markers for the corridor bounding box and
  /// closest obstacle.
  /// @param header Header from sensor point cloud carrying timestamp and frame_id.
  /// @param result Filtering outcome with obstacle coordinates and detection flag.
  void publish_markers(const std_msgs::msg::Header &header, const FilterResult &result)
  {
    visualization_msgs::msg::MarkerArray marker_array;

    // 1. Safety corridor 3D bounding box marker
    visualization_msgs::msg::Marker box_marker;
    box_marker.header = header;
    box_marker.ns = "safety_corridor";
    box_marker.id = 0;
    box_marker.type = visualization_msgs::msg::Marker::CUBE;
    box_marker.action = visualization_msgs::msg::Marker::ADD;

    box_marker.pose.position.x = (roi_.min_x + roi_.max_x) / 2.0;
    box_marker.pose.position.y = (roi_.min_y + roi_.max_y) / 2.0;
    box_marker.pose.position.z = (roi_.min_z + roi_.max_z) / 2.0;
    box_marker.pose.orientation.w = 1.0;

    box_marker.scale.x = roi_.max_x - roi_.min_x;
    box_marker.scale.y = roi_.max_y - roi_.min_y;
    box_marker.scale.z = roi_.max_z - roi_.min_z;

    if (result.obstacle_detected) {
      box_marker.color.r = 1.0f;
      box_marker.color.g = 0.0f;
      box_marker.color.b = 0.0f;
      box_marker.color.a = 0.35f;  // Red alert fill
    } else {
      box_marker.color.r = 0.0f;
      box_marker.color.g = 1.0f;
      box_marker.color.b = 0.0f;
      box_marker.color.a = 0.15f;  // Green clear fill
    }

    marker_array.markers.push_back(box_marker);

    // 2. Closeset obstacle sphere marker
    visualization_msgs::msg::Marker obstacle_marker;
    obstacle_marker.header = header;
    obstacle_marker.ns = "closest_obstacle";
    obstacle_marker.id = 1;
    obstacle_marker.type = visualization_msgs::msg::Marker::SPHERE;

    if (result.obstacle_detected) {
      obstacle_marker.action = visualization_msgs::msg::Marker::ADD;
      obstacle_marker.pose.position.x = result.closest_point.x;
      obstacle_marker.pose.position.y = result.closest_point.y;
      obstacle_marker.pose.position.z = result.closest_point.z;
      obstacle_marker.pose.orientation.w = 1.0;

      obstacle_marker.scale.x = kObstacleMarkerDiameterM;
      obstacle_marker.scale.y = kObstacleMarkerDiameterM;
      obstacle_marker.scale.z = kObstacleMarkerDiameterM;

      obstacle_marker.color.r = 1.0f;
      obstacle_marker.color.g = 0.2f;
      obstacle_marker.color.b = 0.0f;
      obstacle_marker.color.a = 0.9f;
    } else {
      obstacle_marker.action = visualization_msgs::msg::Marker::DELETE;
    }

    marker_array.markers.push_back(obstacle_marker);
    markers_publisher_->publish(marker_array);
  }

  /// @brief Input topic name for depth point clouds.
  std::string point_cloud_topic_;

  /// @brief Proximity threshold in meters to trigger an obstacle detection.
  double detection_threshold_;

  /// @brief Topic used to publish the nearest obstacle point in the safety corridor.
  std::string closest_point_topic_;

  /// @brief 3D safety corridor bounding box in the vehicle coordinate frame.
  RoiBoundingBox roi_;

  /// @brief Subscription handle for incoming PointCloud2 messages.
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;

  /// @brief Publisher handle for the closest obstacle position.
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr closest_point_publisher_;

  /// @brief Publisher handle for RViz corridor and obstacle visualization markers.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_publisher_;
};

/// @brief Main entry point: initializes rclcpp, spins the ObstacleDetector node, and cleanly shuts
/// down.
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ObstacleDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
