/*
 * Robotic Arm Assembly & Test Sketch
 * Description: This sketch facilitates the assembly process by allowing 
 * independent testing of each servo motor. 
 * Usage:
 * 1. Connect the servo to its designated digital pin.
 * 2. On startup, the servo will automatically center to 90°.
 * 3. Open the Serial Monitor at 115200 baud rate.
 * 4. Enter the character (B, S, G, E) of the servo you wish to control.
 * 5. Enter a numeric value (0-180) to set a new target angle.
 */

constexpr int BASE_SERVO_PIN     {6};
constexpr int SHOULDER_SERVO_PIN {9};
constexpr int ELBOW_SERVO_PIN    {10};
constexpr int GRIPPER_SERVO_PIN  {11};

#include "roboticArmTester.h"

RoboticArmTester tester
{
  ServoConfig(BASE_SERVO_PIN,     Normal),
  ServoConfig(SHOULDER_SERVO_PIN, Normal),
  ServoConfig(ELBOW_SERVO_PIN,    Reverse, 20, 180),
  ServoConfig(GRIPPER_SERVO_PIN,  Reverse, 0, 100)
};

void setup() 
{
  Serial.begin(115200);
  tester.begin();
}

void loop() 
{
  // Calculating elapsed time between two loop function calls
  static unsigned long previous_time_micros {};
  unsigned long current_time_micros {micros()};
  float delta_time {(current_time_micros - previous_time_micros) / 1000000.0f}; // Converting microseconds to seconds by dividing 1000000.
  previous_time_micros = current_time_micros;
  // ------------------------------------------------------------------------------------------------------------

  if (Serial.available() > 0)
  {
    String input {Serial.readStringUntil('\n')};
    input.trim();
    input.toUpperCase();

    switch (input[0])
    {
    case 'B': tester.test(TestIndex::Base);     break;
    case 'S': tester.test(TestIndex::Shoulder); break;
    case 'E': tester.test(TestIndex::Elbow);    break;
    case 'G': tester.test(TestIndex::Gripper);  break;
    default: tester.setAngle(input.toFloat());  break;
    }
  }

  tester.update(delta_time);
}