#ifndef ROBOTIC_ARM_H
#define ROBOTIC_ARM_H

#include "config.h"
#include "fixedVector.h"
#include "servoConfig.h"
#include "vector3.h"
#include <Arduino.h>

enum Gripper
{
  open, 
  close
};

struct Waypoint
{
  Vector3 position;
  Gripper gripper;
};

class RoboticArm
{
private:
  FixedVector<Config::MAX_VECTOR_ARR_SIZE, Waypoint> waypointList;

  const float length_shoulder_elbow {}; // [cm]
  const float length_elbow_gripper  {}; // [cm]

  ServoConfig base_servo;
  ServoConfig shoulder_servo;
  ServoConfig elbow_servo;
  ServoConfig gripper_servo;

  float min_distance {}; // [cm]
  float max_distance {}; // [cm]

  float current_base_angle     {Config::REST_ANGLE}; // [degrees]
  float current_shoulder_angle {Config::REST_ANGLE}; // [degrees]
  float current_elbow_angle    {Config::REST_ANGLE}; // [degrees]
  float current_gripper_angle  {Config::REST_ANGLE}; // [degrees]

  float target_base_angle      {Config::REST_ANGLE}; // [degrees]
  float target_shoulder_angle  {Config::REST_ANGLE}; // [degrees]
  float target_elbow_angle     {Config::REST_ANGLE}; // [degrees]
  float target_gripper_angle   {Config::REST_ANGLE}; // [degrees]

  float calib_base_angle     {}; // [degrees]
  float calib_shoulder_angle {}; // [degrees]
  float calib_elbow_angle    {}; // [degrees]

  float gripper_open_angle  {}; // [degrees]
  float gripper_close_angle {}; // [degrees]

  float angular_speed     {Config::DEFAULT_ANGULAR_SPEED}; // [degrees / second]
  float min_angular_speed {Config::MIN_ANGULAR_SPEED};     // [degrees / second]
  float max_angular_speed {Config::MAX_ANGULAR_SPEED};     // [degrees / second]

  Gripper gripperState;
  bool handshake       {false};
  bool isListCreated   {false};

  // WAYPOINT LIST MOVEMENT VARIABLES
  bool list_move     {false};
  float timer        {0.0f};
  bool set_timer     {true};
  int counter        {};
  int waypoint_index {};
  
public:
  RoboticArm(
    float length_shoulder_elbow, 
    float length_elbow_gripper, 
    ServoConfig base_servo, 
    ServoConfig shoudler_servo, 
    ServoConfig elbow_servo,
    ServoConfig gripper_servo
  );

  void begin();
  void update(float delta_time);
  void command(char* cmd);

private:
  void calibrationCommand(char* cmd);
  void calibrateAndSave();
  void resetAllCalibration();
  void handshakeCommand(char* cmd);
  void speedCommand(char* cmd);
  void automaticCommand(char* cmd);
  void manualCommand(char* cmd);
  void gripperCommand(char* cmd);
  void resetArm();
  void saveGripper(char* cmd);
  void setGripper(Gripper state);
  void moveCommand(char* cmd);
  void listCommand(char* cmd);
  void startMoveOnWaypoints();
  void endMoveOnWaypoints();
  void addStartPointToList();
  void addWaypointToList();
  void addEndPointToList();
  void inverseKinematics(Vector3 target);
  Vector3 forwardKinematics();
  void saveEEPROM();
  void getEEPROM();
};

#endif