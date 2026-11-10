#ifndef SUBSEA_INSPECTION__COMMON__INPUT_WATCHDOG_HPP_
#define SUBSEA_INSPECTION__COMMON__INPUT_WATCHDOG_HPP_

#include <chrono>

namespace subsea_inspection::input_watchdog {

/// @brief Reports whether input has been absent for at least the configured timeout.
/// @param last_input Steady-clock time when input was last received.
/// @param now Current steady-clock time.
/// @param timeout Maximum permitted input age.
/// @return True if the input age meets or exceeds the timeout.
inline bool is_stale(std::chrono::steady_clock::time_point last_input,
                     std::chrono::steady_clock::time_point now,
                     std::chrono::duration<double> timeout) {
    return now - last_input >= timeout;
}

}  // namespace subsea_inspection::input_watchdog

#endif  // SUBSEA_INSPECTION__COMMON__INPUT_WATCHDOG_HPP_
