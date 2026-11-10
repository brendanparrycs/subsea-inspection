#ifndef SUBSEA_INSPECTION__PID_CONTROLLER_HPP_
#define SUBSEA_INSPECTION__PID_CONTROLLER_HPP_

#include <algorithm>
#include <cmath>

namespace subsea_inspection {

/// @brief Generic, ROS-independent Proportional-Integral-Derivative (PID) controller with
/// anti-windup and output clamping.
class PidController {
  public:
    /// @brief Constructs a PID controller with specified gains and output limits.
    /// @param kp Proportional gain.
    /// @param ki Integral gain.
    /// @param kd Derivative gain.
    /// @param max_output Maximum allowable control output magnitude.
    /// @param integral_limit Maximum allowable accumulated integral magnitude (anti-windup).
    PidController(double kp = 0.0, double ki = 0.0, double kd = 0.0, double max_output = 1.0,
                  double integral_limit = 1.0)
    : kp_(kp),
      ki_(ki),
      kd_(kd),
      max_output_(std::abs(max_output)),
      integral_limit_(std::abs(integral_limit)),
      prev_error_(0.0),
      integral_(0.0),
      has_run_(false) {}

    /// @brief Computes control effort based on current tracking error and elapsed time.
    /// @param error Difference between desired setpoint and current measurement (target - current).
    /// @param dt Elapsed time in seconds since previous computation.
    /// @return Clamped control command within [-max_output, max_output].
    double compute(double error, double dt) {
        if (dt <= 0.0)
            return 0.0;

        double p_term = kp_ * error;
        integral_ += error * dt;
        integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);

        double i_term = ki_ * integral_;
        double d_term = 0.0;

        if (has_run_) {
            d_term = kd_ * (error - prev_error_) / dt;
        } else {
            has_run_ = true;
        }

        prev_error_ = error;
        double output = p_term + i_term + d_term;
        return std::clamp(output, -max_output_, max_output_);
    }

    /// @brief Resets accumulated integral and previous error states (e.g., when switching
    /// waypoints).
    void reset() {
        integral_ = 0.0;
        prev_error_ = 0.0;
        has_run_ = false;
    }

    /// @brief Updates controller gains.
    /// @param kp Proportional gain.
    /// @param ki Integral gain.
    /// @param kd Derivative gain.
    void set_gains(double kp, double ki, double kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    /// @brief Sets maximum output command limit.
    /// @param max_output Maximum allowable magnitude.
    void set_output_limit(double max_output) {
        max_output_ = std::abs(max_output);
    }

    /// @brief Sets anti-windup integral accumulation limit.
    /// @param integral_limit Maximum allowable integral magnitude.
    void set_integral_limit(double integral_limit) {
        integral_limit_ = std::abs(integral_limit);
    }

  private:
    /// @brief Proportional gain.
    double kp_;

    /// @brief Integral gain.
    double ki_;

    /// @brief Derivative gain.
    double kd_;

    /// @brief Absolute maximum output limit.
    double max_output_;

    /// @brief Absolute maximum integral accumulation limit (anti-windup).
    double integral_limit_;

    /// @brief Error value from previous update cycle.
    double prev_error_;

    /// @brief Accumulated integral error sum.
    double integral_;

    /// @brief Flag indicating whether compute() has been called at least once.
    bool has_run_;
};

}  // namespace subsea_inspection

#endif  // SUBSEA_INSPECTION__PID_CONTROLLER_HPP_
