#include "config.h"
#include "servoConfig.h"
#include <Servo.h>
#include <Arduino.h>

ServoConfig::ServoConfig(int _pin, ServoRot _rot, int _minAngle, int _maxAngle)
  : pin {_pin},
    rot {_rot},
    min_angle {_minAngle},
    max_angle {_maxAngle}
{
}

void ServoConfig::begin(int angle)
{
  // Tell the Servo library what the angle should be 
  // BEFORE the signal pin actually starts sending pulses.
  servoObj.write(angle); // Or your specific REST_ANGLE
  servoObj.attach(pin);
}

void ServoConfig::write(int angle)
{
  int safe_angle {constrain(angle, min_angle, max_angle)};
  switch (rot)
  {
  case ServoRot::Normal:
    servoObj.write(constrain(safe_angle, 0, 180));
    return;
  case ServoRot::Reverse:
    servoObj.write(constrain(180 - safe_angle, 0, 180)); 
    return;
  }
}

int ServoConfig::getMinAngle() const
{
  return min_angle;
}

int ServoConfig::getMaxAngle() const
{
  return max_angle;
}

int ServoConfig::getPin() const
{
  return pin;
}