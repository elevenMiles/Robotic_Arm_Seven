#ifndef CONFIG_H
#define CONFIG_H

namespace Config
{
  static constexpr float REST_ANGLE      {90.0f}; // [degrees]
  static constexpr int   MAX_SERVO_ANGLE {180};   // [degrees]
  static constexpr int   MIN_SERVO_ANGLE {0};     // [degrees]
  static constexpr float ANGULAR_SPEED   {45.0f}; // [degrees / second]
}

#endif