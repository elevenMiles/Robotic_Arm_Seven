#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

#include "config.h"
#include <Servo.h>

enum ServoRot
{
  Normal,
  Reverse
};

class ServoConfig
{
private:
  Servo servoObj;
  const int pin {};
  int min_angle {Config::MIN_SERVO_ANGLE};
  int max_angle {Config::MAX_SERVO_ANGLE};
  ServoRot rot  {ServoRot::Normal};

public:
  ServoConfig(int pin, ServoRot rot = ServoRot::Normal, int min_angle = Config::MIN_SERVO_ANGLE, int max_angle = Config::MAX_SERVO_ANGLE);
  void begin(int angle = Config::REST_ANGLE);
  void write(int angle);
  int getMinAngle() const;
  int getMaxAngle() const;
  int getPin() const;
};

#endif