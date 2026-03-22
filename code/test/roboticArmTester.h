#ifndef ROBOTIC_ARM_TESTER
#define ROBOTIC_ARM_TESTER

#include "config.h"
#include "servoConfig.h"

enum TestIndex
{
  Base     = 0,
  Shoulder = 1,
  Elbow    = 2,
  Gripper  = 3
};

class RoboticArmTester
{
private:
  ServoConfig servoArray[4];
  ServoConfig* servo_ptr;

  float current_angle {Config::REST_ANGLE};
  float target_angle  {Config::REST_ANGLE};

  float angular_speed {Config::ANGULAR_SPEED};

public:
  RoboticArmTester(ServoConfig base, ServoConfig shoulder, ServoConfig elbow, ServoConfig gripper);
  void begin();
  void test(TestIndex index);
  void setAngle(float angle);
  void update(float delta_time);
};

#endif