#include "config.h"
#include "servoConfig.h"
#include "roboticArmTester.h"
#include <HardwareSerial.h>
#include <Arduino.h>

RoboticArmTester::RoboticArmTester(ServoConfig base, ServoConfig shoulder, ServoConfig elbow, ServoConfig gripper)
  : servoArray {base, shoulder, elbow, gripper}
{
}

void RoboticArmTester::begin()
{
  for (int i {0}; i < 4; ++i)
  {
    servoArray[i].begin();
  }

  Serial.println(F("Enter one of the following characters to test corresponding servo:"));
  Serial.println(F("B -> base servo"));
  Serial.println(F("S -> shoulder servo"));
  Serial.println(F("E -> elbow servo"));
  Serial.println(F("G -> gripper servo")); Serial.println();
}

void RoboticArmTester::test(TestIndex index)
{
  servo_ptr = &servoArray[index];

  current_angle = Config::REST_ANGLE;
  target_angle = Config::REST_ANGLE;

  Serial.println();

  switch (index)
  {
  case TestIndex::Base:     Serial.println(F("Base servo test started."));     break;
  case TestIndex::Shoulder: Serial.println(F("Shoulder servo test started.")); break;
  case TestIndex::Elbow:    Serial.println(F("Elbow servo test started."));    break;
  case TestIndex::Gripper:  Serial.println(F("Gripper servo test started."));  break;
  }

  Serial.print(F("Please connect the servo to digital pin ")); Serial.println(servo_ptr->getPin());
  Serial.print(F("Default servo angle: ")); Serial.print(current_angle); Serial.println(F(" degrees."));
  Serial.print(F("Enter angle values between (")); Serial.print(servo_ptr->getMinAngle()); Serial.print(F(" - ")); 
  Serial.print(servo_ptr->getMaxAngle()); Serial.println(F(") degrees to move the servo. Or enter a character (B, S, E, G) to test another servo."));
}

void RoboticArmTester::setAngle(float angle)
{
  target_angle = constrain(angle, servo_ptr->getMinAngle(), servo_ptr->getMaxAngle());
  Serial.print(F("Servo moving to angle: ")); Serial.print(target_angle); Serial.println(F(" degrees."));
}

void RoboticArmTester::update(float delta_time)
{
  float max_delta    {angular_speed * delta_time};
  float displacement {target_angle - current_angle};
  float distance     {abs(displacement)};

  if (distance != 0.0f || distance > max_delta)
  {
    float direction {displacement / distance};

    current_angle += direction * max_delta;
    current_angle = constrain(current_angle, servo_ptr->getMinAngle(), servo_ptr->getMaxAngle());
  }

  servo_ptr->write(current_angle);
}