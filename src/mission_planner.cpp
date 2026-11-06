#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "yaml-cpp/yaml.h"

#include "subsea_inspection/action/navigate_to_waypoint.hpp"

using namespace std::chrono_literals;
using NavigateToWaypoint = subsea_inspection::action::NavigateToWaypoint;
using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToWaypoint>;

/// @brief Represents a single 3D inspection waypoint along the pipeline route.
struct Waypoint {
    std::string name;
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double acceptance_radius{0.5};
    double dwell_sec{0.0};
};

/// @brief Possible states of the mission sequencer state machine.
enum class MissionState {
    IDLE,
    SENDING_GOAL,
    NAVIGATING,
    DWELLING,
    MISSION_COMPLETE,
    ABORTED
};

/// @brief High-level mission planner and sequencer for autonomous pipeline inspection.
class MissionPlanner: public rclcpp::Node {
public:
    /// @brief Initializes node, declares parameters, loads YAML mission, and starts action client.
    MissionPlanner(): 
        Node("mission_planner"), 
        state_(MissionState::IDLE), 
        default_acceptance_radius_(0.5), 
        current_waypoint_index_(0), 
        total_waypoints_completed_(0) 
    {
        // Declare and get parameters
        this->declare_parameter<std::string>("mission_file", "");
        mission_file_ = this->get_parameter("mission_file").as_string();

        if (mission_file_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'mission_file' is empty! Cannot proceed.");
            state_ = MissionState::ABORTED;
            return;
        }

        // Load and parse mission YAML
        if (!load_mission_file(mission_file_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load mission file: %s", mission_file_.c_str());
            state_ = MissionState::ABORTED;
            return;
        }

        // Create action client for /navigate_to_waypoint
        action_client_ = rclcpp_action::create_client<NavigateToWaypoint>(
            this,
            "navigate_to_waypoint"
        );

        // 100ms (10Hz) sequencer loop timer
        timer_ = this->create_wall_timer(
            100ms,
            std::bind(&MissionPlanner::sequencer_loop, this)
        );

        mission_start_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "MissionPlanner initialized. Waiting for action server...");
    }

private:
    /// @brief Parses YAML mission definition into waypoint structs.
    /// @param filepath Absolute or relative path to mission YAML file.
    /// @return True if parsing succeeded and waypoints exist; false otherwise.
    bool load_mission_file(const std::string &filepath) {
        if (!std::filesystem::exists(filepath)) {
            RCLCPP_ERROR(this->get_logger(), "Mission file does not exist: %s", filepath.c_str());
            return false;
        }

        try {
            YAML::Node root = YAML::LoadFile(filepath);
            if (!root["mission"]) {
                RCLCPP_ERROR(this->get_logger(), "YAML root missing 'mission' key.");
                return false;
            }

            YAML::Node mission_node = root["mission"];
            mission_name_ = mission_node["name"].as<std::string>("unnamed_mission");
            default_acceptance_radius_ = mission_node["default_acceptance_radius"].as<double>(0.5);

            YAML::Node waypoints_node = mission_node["waypoints"];
            if (!waypoints_node || !waypoints_node.IsSequence()) {
                RCLCPP_ERROR(this->get_logger(), "'waypoints' key is missing or not a sequence.");
                return false;
            }

            waypoints_.clear();
            for (std::size_t i = 0; i < waypoints_node.size(); ++i) {
                YAML::Node wp = waypoints_node[i];
                Waypoint waypoint;
                waypoint.name = wp["name"].as<std::string>("wp_" + std::to_string(i + 1));
                waypoint.x = wp["x"].as<double>(0.0);
                waypoint.y = wp["y"].as<double>(0.0);
                waypoint.z = wp["z"].as<double>(-2.0);
                waypoint.acceptance_radius = wp["acceptance_radius"].as<double>(default_acceptance_radius_);
                waypoint.dwell_sec = wp["dwell_sec"].as<double>(0.0);

                waypoints_.push_back(waypoint);
            }

            RCLCPP_INFO(
                this->get_logger(),
                "Loaded mission '%s' with %zu waypoints from %s.",
                mission_name_.c_str(),
                waypoints_.size(),
                filepath.c_str()
            );

            return !waypoints_.empty();
        } catch (const YAML::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "YAML parsing error: %s", e.what());
            return false;
        }
    }

    /// @brief 100ms (10Hz) state machine loop managing mission execution lifecycle.
    void sequencer_loop() {
        switch (state_) {
            case MissionState::IDLE: {
                // Wait for action server
                if (!action_client_->wait_for_action_server(0s)) {
                    RCLCPP_INFO_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        2000,
                        "Waiting for /navigate_to_waypoint action server..."
                    );

                    return;
                }

                RCLCPP_INFO(this->get_logger(), "Action server detected. Starting mission execution!");
                mission_start_time_ = this->now();
                state_ = MissionState::SENDING_GOAL;
                break;
            }

            case MissionState::SENDING_GOAL: {
                if (current_waypoint_index_ >= waypoints_.size()) {
                    state_ = MissionState::MISSION_COMPLETE;
                    return;
                }

                send_current_waypoint_goal();
                state_ = MissionState::NAVIGATING;
                break;
            }

            case MissionState::NAVIGATING: {
                break;
            }

            case MissionState::DWELLING: {
                double elapsed_dwell = (this->now() - dwell_start_time_).seconds();
                const auto &wp = waypoints_[current_waypoint_index_];

                if (elapsed_dwell >= wp.dwell_sec) {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Dwell complete at '%s'. Advancing to next waypoint.",
                        wp.name.c_str()
                    );

                    current_waypoint_index_++;
                    state_ = MissionState::SENDING_GOAL;
                } else {
                    RCLCPP_INFO_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        1000,
                        "[Dwell] Paused at '%s' (%.1f / %.1f s)",
                        wp.name.c_str(),
                        elapsed_dwell,
                        wp.dwell_sec
                    );
                }

                break;
            }

            case MissionState::MISSION_COMPLETE: {
                print_mission_report();
                timer_->cancel();
                break;
            }

            case MissionState::ABORTED: {
                RCLCPP_ERROR(this->get_logger(), "Mission aborted due to error.");
                timer_->cancel();
                break;
            }
        }
    }

    /// @brief Sends the active waypoint goal to the action server.
    void send_current_waypoint_goal() {
        const auto &wp = waypoints_[current_waypoint_index_];

        auto goal_msg = NavigateToWaypoint::Goal();
        goal_msg.target_x = wp.x;
        goal_msg.target_y = wp.y;
        goal_msg.target_z = wp.z;
        goal_msg.acceptance_radius = wp.acceptance_radius;

        RCLCPP_INFO(
            this->get_logger(),
            "\n>>> [Waypoint %zu/%zu] Dispatching '%s' -> Target: [X: %.2f, Y: %.2f, Z: %.2f, Radius: %.2f]",
            current_waypoint_index_ + 1,
            waypoints_.size(),
            wp.name.c_str(),
            wp.x,
            wp.y,
            wp.z,
            wp.acceptance_radius
        );

        auto send_goal_options = rclcpp_action::Client<NavigateToWaypoint>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&MissionPlanner::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&MissionPlanner::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&MissionPlanner::result_callback, this, std::placeholders::_1);
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    /// @brief Callback when action server accepts or rejects goal.
    /// @param goal_handle Shared pointer to client goal handle.
    void goal_response_callback(const GoalHandleNavigate::SharedPtr &goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by action server!");
            state_ = MissionState::ABORTED;
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by waypoint navigator. In transit...");
        }
    }

    /// @brief Periodic feedback callback during transit.
    /// @param feedback Shared pointer to incoming NavigateToWaypoint feedback.
    void feedback_callback(
        GoalHandleNavigate::SharedPtr,
        const std::shared_ptr<const NavigateToWaypoint::Feedback> feedback
    ) {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1500,
            "[Progress] Distance remaining: %.2f m | Heading error: %.1f deg",
            feedback->distance_remaining,
            feedback->heading_error_deg
        );
    }

    /// @brief Terminal result callback when waypoint is reached or fails.
    /// @param result Wrapped action result from action server.
    void result_callback(const GoalHandleNavigate::WrappedResult &result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED: {
                total_waypoints_completed_++;
                const auto &wp = waypoints_[current_waypoint_index_];

                RCLCPP_INFO(
                    this->get_logger(),
                    "=== Reached '%s' in %.2f s (Final error: %.2f m) ===",
                    wp.name.c_str(),
                    result.result->elapsed_time,
                    result.result->final_distance
                );

                if (wp.dwell_sec > 0.0) {
                    dwell_start_time_ = this->now();
                    state_ = MissionState::DWELLING;
                } else {
                    current_waypoint_index_++;
                    state_ = MissionState::SENDING_GOAL;
                }

                break;
            }

            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Waypoint goal was aborted by server!");
                state_ = MissionState::ABORTED;
                break;

            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Waypoint goal was canceled!");
                state_ = MissionState::ABORTED;
                break;

            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown action result code.");
                state_ = MissionState::ABORTED;
                break;
        }
    }

    /// @brief Prints formatted mission performance summary report.
    void print_mission_report() {
        double total_duration = (this->now() - mission_start_time_).seconds();
        RCLCPP_INFO(this->get_logger(), "==================================================");
        RCLCPP_INFO(this->get_logger(), "          SUBSEA MISSION COMPLETE REPORT          ");
        RCLCPP_INFO(this->get_logger(), "==================================================");
        RCLCPP_INFO(this->get_logger(), " Mission Name:        %s", mission_name_.c_str());
        RCLCPP_INFO(this->get_logger(), " Status:              SUCCESS");
        RCLCPP_INFO(this->get_logger(), " Waypoints Visited:   %zu / %zu", total_waypoints_completed_, waypoints_.size());
        RCLCPP_INFO(this->get_logger(), " Total Mission Time:  %.2f seconds", total_duration);
        RCLCPP_INFO(this->get_logger(), "==================================================");
    }

    /// @brief Path to the YAML mission configuration file.
    std::string mission_file_;

    /// @brief Current state of the sequencer state machine.
    MissionState state_;

    /// @brief Default acceptance radius for waypoints if not specified per waypoint.
    double default_acceptance_radius_;

    /// @brief Index of the currently active waypoint in the waypoints sequence.
    std::size_t current_waypoint_index_;

    /// @brief Counter tracking the total number of successfully completed waypoints.
    std::size_t total_waypoints_completed_;

    /// @brief Name identifier of the mission loaded from YAML.
    std::string mission_name_;

    /// @brief Ordered list of waypoints loaded from the mission file.
    std::vector<Waypoint> waypoints_;

    /// @brief Timestamp when mission execution began.
    rclcpp::Time mission_start_time_;

    /// @brief Timestamp when current dwell period began.
    rclcpp::Time dwell_start_time_;

    /// @brief Action client handle for sending waypoint goals.
    rclcpp_action::Client<NavigateToWaypoint>::SharedPtr action_client_;

    /// @brief 100ms (10Hz) periodic loop timer for the sequencer state machine.
    rclcpp::TimerBase::SharedPtr timer_;
};

/// @brief Main entry point: initializes rclcpp, spins the MissionPlanner node, and cleanly shuts down.
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MissionPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
