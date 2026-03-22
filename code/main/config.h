#ifndef CONFIG_H
#define CONFIG_H

namespace Config
{
  static constexpr float REST_ANGLE            {90.0f}; // [degrees]
  static constexpr int   MAX_SERVO_ANGLE       {180};   // [degrees]
  static constexpr int   MIN_SERVO_ANGLE       {0};     // [degrees]
  static constexpr float MIN_ANGULAR_SPEED     {30.0f}; // [degrees / second]
  static constexpr float MAX_ANGULAR_SPEED     {90.0f}; // [degrees / second]
  static constexpr float DEFAULT_ANGULAR_SPEED {60.0f}; // [degrees / second]
  static constexpr float PAUSE_TIME            {0.5f};  // [seconds]
  static constexpr float MAX_SAFETY_MARGIN     {1.0f};  // [cm]
  static constexpr float MIN_SAFETY_MARGIN     {3.0f};  // [cm]
  static constexpr float DEFAULT_MOVE_AMOUNT   {5.0f};  // [cm]
  static constexpr float MIN_X                 {1.0f};   // [cm]
  static constexpr float MIN_Z                 {-6.0f};   // [cm]
  static constexpr int   MAX_VECTOR_ARR_SIZE   {25};
  static constexpr int   EEPROM_MEMORY_ADDRESS {0};
}

#endif