#ifndef SUBSEA_INSPECTION__COMMON__CONSTANTS_HPP_
#define SUBSEA_INSPECTION__COMMON__CONSTANTS_HPP_

namespace subsea_inspection {
namespace constants {

/// @brief Mathematical constant pi, used for angle conversions.
inline constexpr double kPi = 3.14159265358979323846;

/// @brief Multiplier for converting radians to degrees.
inline constexpr double kRadiansToDegrees = 180.0 / kPi;

/// @brief Multiplier for converting degrees to radians.
inline constexpr double kDegreesToRadians = kPi / 180.0;

}  // namespace constants
}  // namespace subsea_inspection

#endif  // SUBSEA_INSPECTION__COMMON__CONSTANTS_HPP_
