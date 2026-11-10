#ifndef SUBSEA_INSPECTION_COMMON_CONSTANTS_HPP_
#define SUBSEA_INSPECTION_COMMON_CONSTANTS_HPP_

namespace subsea_inspection
{
namespace constants
{

/// Mathematical constant pi, used for angle conversions.
inline constexpr double kPi = 3.14159265358979323846;

/// Multiplier for converting radians to degrees.
inline constexpr double kRadiansToDegrees = 180.0 / kPi;

/// Multiplier for converting degrees to radians.
inline constexpr double kDegreesToRadians = kPi / 180.0;

}  // namespace constants
}  // namespace subsea_inspection

#endif  // SUBSEA_INSPECTION_COMMON_CONSTANTS_HPP_
