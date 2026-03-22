#include "HardwareSerial.h"
#include "WString.h"
#include "config.h"
#include "fixedVector.h"
#include "vector3.h"
#include "servoConfig.h"
#include "roboticArm.h"
#include <math.h>
#include <EEPROM.h>
#include <Arduino.h>

struct Calibration
{
  float baseAngle;
  float shoulderAngle;
  float elbowAngle;
  float gripperOpenAngle;
  float gripperCloseAngle;
};

RoboticArm::RoboticArm(
    float _lengthShoulderElbow, 
    float _lengthElbowGripper, 
    ServoConfig _baseServo, 
    ServoConfig _shoulderServo,
    ServoConfig _elbowServo,
    ServoConfig _gripperServo
  )
  : length_shoulder_elbow {_lengthShoulderElbow}, 
    length_elbow_gripper {_lengthElbowGripper},

    base_servo {_baseServo},
    shoulder_servo {_shoulderServo},
    elbow_servo {_elbowServo},
    gripper_servo {_gripperServo}
{
}

void RoboticArm::begin()
{
  getEEPROM();

  base_servo.begin(static_cast<int>(Config::REST_ANGLE + calib_base_angle));
  shoulder_servo.begin(static_cast<int>(Config::REST_ANGLE + calib_shoulder_angle));
  elbow_servo.begin(static_cast<int>(Config::REST_ANGLE + calib_elbow_angle));
  
  gripper_servo.begin(static_cast<int>(gripper_open_angle));

  min_distance = abs(length_shoulder_elbow - length_elbow_gripper) + Config::MIN_SAFETY_MARGIN;
  max_distance = length_shoulder_elbow + length_elbow_gripper      - Config::MAX_SAFETY_MARGIN;
  target_gripper_angle = gripper_open_angle;

  Serial.println(F("Robotic arm initialized.")); 
  Serial.println(F("If this is the first run, please use -R command to reset all calibration angles."));
}

void RoboticArm::update(float delta_time)
{
  float max_delta {angular_speed * delta_time};

  // ------------------------------------------------------------------------------
  // GRIPPER UPDATE
  float distance {abs(target_gripper_angle - current_gripper_angle)};

  if (max_delta > distance)
  {
    current_gripper_angle = target_gripper_angle;
  }
  else
  {
    float direction {(target_gripper_angle - current_gripper_angle) / distance};
    current_gripper_angle += direction * max_delta;
  }

  if (abs(current_gripper_angle - gripper_close_angle) < max_delta)
  {
    gripperState = close;
  }
  else if (abs(current_gripper_angle - gripper_open_angle) < max_delta)
  {
    gripperState = open;
  }

  // ------------------------------------------------------------------------------
  // ARM UPDATE
  Vector3 calibration_angle_vector {calib_base_angle, calib_shoulder_angle, calib_elbow_angle};

  Vector3 current_angle_vector {current_base_angle, current_shoulder_angle, current_elbow_angle};
  Vector3 target_angle_vector  {target_base_angle, target_shoulder_angle, target_elbow_angle};

  Vector3 movement = Vector3::moveTowards(current_angle_vector, target_angle_vector + calibration_angle_vector, max_delta);

  current_base_angle = movement.x;
  current_shoulder_angle = movement.y;
  current_elbow_angle = movement.z;

  // ------------------------------------------------------------------------------
  if (list_move)
  {
    if (set_timer)
    {
      set_timer = false;
      if (counter == 0)
      {
        inverseKinematics(waypointList[waypoint_index].position);
        Vector3 current_angle_vector {current_base_angle, current_shoulder_angle, current_elbow_angle};
        Vector3 target_angle_vector  {target_base_angle, target_shoulder_angle, target_elbow_angle};
        float dist {Vector3::distance(current_angle_vector, target_angle_vector)};
        timer = dist / angular_speed + Config::PAUSE_TIME;
      }
      if (counter == 1)
      {
        setGripper(waypointList[waypoint_index].gripper);
        float dist {abs(current_gripper_angle - target_gripper_angle)};
        timer = dist / angular_speed + Config::PAUSE_TIME;
      }

      counter += 1;
      if (counter > 1)
      {
        counter = 0;
        waypoint_index += 1;
      }
    }

    timer -= delta_time;
    if (timer <= 0.0f)
    {
      set_timer = true;
      if (waypoint_index >= waypointList.len())
      {
        endMoveOnWaypoints();
      }
    }
  }
  // ------------------------------------------------------------------------------
  // WRITE TO SERVOS
  base_servo.write(static_cast<int>(current_base_angle));
  shoulder_servo.write(static_cast<int>(current_shoulder_angle));
  elbow_servo.write(static_cast<int>(current_elbow_angle));
  gripper_servo.write(static_cast<int>(current_gripper_angle));
  // ------------------------------------------------------------------------------
  
  // HANDSHAKE
  if (handshake)
    Serial.println(F("K"));
}

void RoboticArm::command(char* cmd)
{
  // 0. No command accepted during waypoint list movement.
  if (list_move) return;

  // 1. Skip any leading whitespace or newlines
  while (*cmd == ' ' || *cmd == '\n' || *cmd == '\r' || *cmd == '\t') 
  {
    cmd++;
  }

  // 2. Check if the string is empty after trimming
  if (cmd[0] == '\0') 
  {
    return; 
  }

  if (handshake)
  {
    switch(cmd[0])
    {
    case '-':
    case '+': return;
    }
  }

  // 3. Process the command
  switch(cmd[0])
  {
    case '-': calibrationCommand(cmd); return;
    case 'H': handshakeCommand(cmd);   return;
    case 'S': speedCommand(cmd);       return;
    case 'A': automaticCommand(cmd);   return;
    case 'M': manualCommand(cmd);      return;
    case 'G': gripperCommand(cmd);     return;
    case 'O': resetArm();              return;
    case 'U':
    case 'D':
    case 'L':
    case 'R':
    case 'F':
    case 'B': moveCommand(cmd);        return; 
    case '+': listCommand(cmd);        return;
    case 'T': Serial.print(F("End effector position: ")); forwardKinematics().println(); return;
  }
  
  Serial.print(F("INVALID COMMAND: "));
  Serial.println(cmd[0]); // Print the character that failed for debugging
}

void RoboticArm::calibrationCommand(char* cmd)
{
  switch (cmd[1])
  {
  case 'C': calibrateAndSave();    return;
  case 'R': resetAllCalibration(); return;
  case 'G': saveGripper(cmd);      return;
  }
}

void RoboticArm::calibrateAndSave()
{
  calib_base_angle = current_base_angle - Config::REST_ANGLE;
  calib_shoulder_angle = current_shoulder_angle - Config::REST_ANGLE;
  calib_elbow_angle = current_elbow_angle - Config::REST_ANGLE;

  saveEEPROM();

  // Move robotic arm to it's rest position after calibration.
  target_base_angle = Config::REST_ANGLE;
  target_shoulder_angle = Config::REST_ANGLE;
  target_elbow_angle = Config::REST_ANGLE;

  Serial.println(F("CALIBRATED AND SAVED."));
}

void RoboticArm::resetAllCalibration()
{
  calib_base_angle = 0.0f;
  calib_shoulder_angle = 0.0f;
  calib_elbow_angle = 0.0f;
  gripper_open_angle = gripper_servo.getMaxAngle();
  gripper_close_angle = gripper_servo.getMinAngle();
  
  saveEEPROM();

  Serial.println(F("ALL CALIBRATION VALUES RESET AND SAVED."));
}

void  RoboticArm::handshakeCommand(char* cmd)
{
  switch (cmd[1]) 
  {
  case 'A': handshake = true; return;
  case 'D': handshake = false; return;
  }
}

void RoboticArm::speedCommand(char* cmd)
{
  // 1. Find the position of the opening parenthesis '('
  char* startPtr {strchr(cmd, '(')};
  
  if (startPtr != NULL) 
  {
    // Move pointer one step forward to point at the actual number
    startPtr++; 

    // 2. Convert the text starting at startPtr directly to a float
    // atof stops automatically when it hits a non-numeric character like ')'
    float speed {atof(startPtr)};

    // 3. Constrain the speed
    angular_speed = constrain(speed, min_angular_speed, max_angular_speed);
    
    if (!handshake)
    {
      Serial.print(F("New Speed: ")); Serial.print(angular_speed); Serial.print(" degrees/second ");
      Serial.print(F("(Min: ")); Serial.print(Config::MIN_ANGULAR_SPEED); Serial.print(F(" degrees/second, Max: ")); 
      Serial.print(Config::MAX_ANGULAR_SPEED); Serial.println(F(" degrees/second)."));
    }
  }
}

void RoboticArm::automaticCommand(char* cmd)
{
  // 1. Point to the start of the coordinates inside '('
  char* dataStart {strchr(cmd, '(')};
  
  if (dataStart != NULL)
  {
    dataStart++; // Move past the '(' to the first number

    // 2. Tokenize the buffer into X, Y, and Z strings
    char* xStr {strtok(dataStart, " ")};
    char* yStr {strtok(NULL, " ")};
    char* zStr {strtok(NULL, " )")}; // Stops at space or closing ')'

    // 3. Convert and pass to Inverse Kinematics function
    if (xStr != NULL && yStr != NULL && zStr != NULL)
    {
      Vector3 target;
      target.x = atof(xStr);
      target.y = atof(yStr);
      target.z = atof(zStr);

      inverseKinematics(target);
    }
  }
}

void RoboticArm::manualCommand(char* cmd)
{
  // 1. Find the starting point after the '('
  char* dataStart {strchr(cmd, '(')};
  
  if (dataStart != NULL)
  {
    dataStart++; // Move past the '('

    // 2. Tokenize the string
    // strtok(dataStart, " ") finds the first space and puts a \0 there
    char* baseStr     {strtok(dataStart, " ")}; 
    
    // Passing NULL tells strtok to keep going from where it left off
    char* shoulderStr {strtok(NULL, " ")};
    
    // For the last one, we look for ')' to terminate the string correctly
    char* elbowStr    {strtok(NULL, " )")}; 

    // 3. Convert and assign if all tokens were found
    if (baseStr != NULL && shoulderStr != NULL && elbowStr != NULL)
    {
      target_base_angle     = atof(baseStr);
      target_shoulder_angle = atof(shoulderStr);
      target_elbow_angle    = atof(elbowStr);

      if (!handshake)
      {
        Serial.print(F("Moving to angles: "));
        Vector3(target_base_angle, target_shoulder_angle, target_elbow_angle).println();
      }
    }
  }
}

void RoboticArm::gripperCommand(char* cmd)
{
  // 1. Handle the prefix logic ('O', 'C')
  switch(cmd[1])
  {
    case 'O': setGripper(open);  return;
    case 'C': setGripper(close); return;
  }

  // 2. Handle the "Angle" logic if it wasn't a shortcut command
  char* startPtr {strchr(cmd, '(')};
  
  if (startPtr != NULL)
  {
    startPtr++; // Move past the '('
    
    // atof reads until it hits ')' or a non-numeric char
    float angle {atof(startPtr)}; 

    target_gripper_angle = constrain(angle, 
                                     gripper_servo.getMinAngle(), 
                                     gripper_servo.getMaxAngle());
  }
}

void RoboticArm::setGripper(Gripper state)
{
  switch (state) 
  {
  case open:
    target_gripper_angle = gripper_open_angle;
    return;
  case close:
    target_gripper_angle = gripper_close_angle;
    return;
  }
}

void RoboticArm::resetArm()
{
  target_base_angle = Config::REST_ANGLE;
  target_shoulder_angle = Config::REST_ANGLE;
  target_elbow_angle = Config::REST_ANGLE;
}

void RoboticArm::saveGripper(char* cmd)
{
  // 1. Find the start of the angle value inside the parentheses
  char* startPtr {strchr(cmd, '(')};
  
  if (startPtr != NULL)
  {
    startPtr++; // Move past the '('

    // 2. Convert to float and constrain immediately
    float angle {atof(startPtr)};
    angle = constrain(angle, gripper_servo.getMinAngle(), gripper_servo.getMaxAngle());

    // 3. Use the character at index 2 to decide which limit to save
    // Example command: "-GO(120)" -> index 2 is 'O'
    switch (cmd[2]) 
    {
      case 'O':
        if (angle < gripper_close_angle)
        {
          Serial.println(F("Gripper open angle can't be less than gripper close angle."));
          return;
        }
        gripper_open_angle = angle;
        Serial.print(F("Open gripper angle is set to: ")); Serial.print(gripper_open_angle); Serial.println(F(" degrees. And saved."));
        break;
      case 'C':
        if (angle > gripper_open_angle)
        {
          Serial.println(F("Gripper close angle can't be grater than gripper open angle."));
          return;
        }
        gripper_close_angle = angle;
        Serial.print(F("Close gripper angle is set to: ")); Serial.print(gripper_close_angle); Serial.println(F(" degrees. And saved."));
        break;
      default:
        return; // Invalid command format, exit without saving
    }

    // 4. Commit to permanent memory
    saveEEPROM();
  }
}

void RoboticArm::moveCommand(char* cmd)
{
  Vector3 dir;
  switch (cmd[0])
  {
    case 'U': dir = Vector3::up();       break;
    case 'D': dir = Vector3::down();     break;
    case 'L': dir = Vector3::left();     break;
    case 'R': dir = Vector3::right();    break;
    case 'F': dir = Vector3::forward();  break;
    case 'B': dir = Vector3::backward(); break;
    default:  dir = Vector3::zero();     break;
  }

  float distance {Config::DEFAULT_MOVE_AMOUNT};

  // Check if there is a specific distance in the command, e.g., "U(10)"
  if (cmd[1] == '(')
  {
    char* startPtr = strchr(cmd, '(');
    if (startPtr != NULL)
    {
      // Point to the first digit and convert
      distance = atof(startPtr + 1); 
    }
  }

  // Calculate new position using Forward + Direction * Distance, then IK back to angles
  inverseKinematics(forwardKinematics() + dir * distance);
}

void RoboticArm::listCommand(char* cmd)
{
  switch (cmd[1])
  {
  case 'S': addStartPointToList();  return;
  case 'W': addWaypointToList();    return;
  case 'E': addEndPointToList();    return;
  case 'M': startMoveOnWaypoints(); return;
  }
}

void RoboticArm::startMoveOnWaypoints()
{
  if (!isListCreated && waypointList.len() >= 1)
  {
    Serial.println(F("Started to move on waypoints."));
    list_move = true;
    set_timer = true;
  }
  else 
    Serial.println(F("List is not created or empty."));
}

void RoboticArm::endMoveOnWaypoints()
{
  waypoint_index = 0;
  list_move = false;
  Serial.println(F("Ended to move on waypoints."));
}

void RoboticArm::addStartPointToList()
{
  waypointList.clear();
  Waypoint startPoint {forwardKinematics(), gripperState};
  waypointList.add(startPoint);
  isListCreated = true;
  
  Serial.print(waypointList.len());
  Serial.print(F(" - New list created and "));
  startPoint.position.print();
  Serial.print(F(" added as first point. Gripper saved as "));
  Serial.println(startPoint.gripper == open ? F("open.") : F("close."));
}

void RoboticArm::addWaypointToList()
{
  if (isListCreated)
  {
    if (waypointList.len() >= Config::MAX_VECTOR_ARR_SIZE - 1)
    {
      addEndPointToList();
      return;
    }

    Waypoint waypoint {forwardKinematics(), gripperState};
    waypointList.add(waypoint);

    Serial.print(waypointList.len());
    Serial.print(F(" - Waypoint "));
    waypoint.position.print();
    Serial.print(F(" added to the list. Gripper saved as "));
    Serial.println(waypoint.gripper == open ? F("open.") : F("close."));
  }
  else 
    Serial.println(F("You cannot add a waypoint before adding a start point."));
}

void RoboticArm::addEndPointToList()
{
  if (isListCreated)
  {
    Waypoint endPoint {forwardKinematics(), gripperState};
    waypointList.add(endPoint);
    isListCreated = false;

    Serial.print(waypointList.len());
    Serial.print(F(" - Current list closed and "));
    endPoint.position.print();
    Serial.print(F(" added as end point. Gripper saved as "));
    Serial.println(endPoint.gripper == open ? F("open.") : F("close."));
  }
  else
    Serial.println(F("You cannot add a end point before adding a start point."));
}

void RoboticArm::inverseKinematics(Vector3 target)
{
  if (target.x < Config::MIN_X) target.x = Config::MIN_X;
  if (target.z < Config::MIN_Z) target.z = Config::MIN_Z;

  float xyDistance {sqrtf(target.x * target.x + target.y * target.y)};
  float length_shoulder_target {sqrtf(xyDistance * xyDistance + target.z * target.z)};
  length_shoulder_target = constrain(length_shoulder_target, min_distance, max_distance);

  float lengthA {length_elbow_gripper};
  float lengthB {length_shoulder_elbow};
  float lengthC {length_shoulder_target};

  float cosBeta   {(lengthA * lengthA + lengthB * lengthB - lengthC * lengthC) / (2.0f * lengthA * lengthB)};
  float betaAngle {acosf(constrain(cosBeta, -1.0f, 1.0f))};

  float cosX   {(lengthB * lengthB + lengthC * lengthC - lengthA * lengthA) / (2.0f * lengthB * lengthC)};
  float xAngle {acosf(constrain(cosX, -1.0f, 1.0f))};
  float yAngle {atan2f(target.z, (target.x > 0.0f ? xyDistance : -xyDistance))};

  float alphaAngle {xAngle + yAngle};

  float tetaAngle {atan2f(-target.y, target.x) + PI / 2.0f};
  if (tetaAngle < 0.0f) tetaAngle += PI * 2.0f;

  if (target.x < 0.0f)
    tetaAngle -= PI;

  target_base_angle     = tetaAngle  * RAD_TO_DEG;
  target_shoulder_angle = alphaAngle * RAD_TO_DEG;
  target_elbow_angle    = betaAngle  * RAD_TO_DEG;
}

Vector3 RoboticArm::forwardKinematics()
{
  // Convert degrees to radians
  float b {(current_base_angle - calib_base_angle)         * DEG_TO_RAD};
  float s {(current_shoulder_angle - calib_shoulder_angle) * DEG_TO_RAD};
  float e {(current_elbow_angle - calib_elbow_angle)       * DEG_TO_RAD};

  float L1 {length_shoulder_elbow};
  float L2 {length_elbow_gripper};

  float horizontal_dist {cosf(s) * L1 + cosf(e + s - PI) * L2};
  float vertical_dist   {sinf(s) * L1 + sinf(e + s - PI) * L2};

  Vector3 endEffector 
  {
    sinf(b) * horizontal_dist,
    cosf(b) * horizontal_dist,
    vertical_dist
  };

  return endEffector;
}

void RoboticArm::saveEEPROM()
{
  Calibration calibWrite;
  calibWrite.baseAngle = calib_base_angle;
  calibWrite.shoulderAngle = calib_shoulder_angle;
  calibWrite.elbowAngle = calib_elbow_angle;
  calibWrite.gripperOpenAngle = gripper_open_angle;
  calibWrite.gripperCloseAngle = gripper_close_angle;

  EEPROM.put(Config::EEPROM_MEMORY_ADDRESS, calibWrite);
}

void RoboticArm::getEEPROM()
{
  Calibration calibRecover;
  EEPROM.get(Config::EEPROM_MEMORY_ADDRESS, calibRecover);
  calib_base_angle     = calibRecover.baseAngle;
  calib_shoulder_angle = calibRecover.shoulderAngle;
  calib_elbow_angle    = calibRecover.elbowAngle;
  gripper_open_angle   = calibRecover.gripperOpenAngle;
  gripper_close_angle  = calibRecover.gripperCloseAngle;

  calib_base_angle =     isnan(calib_base_angle)     ? 0.0f : calib_base_angle;
  calib_shoulder_angle = isnan(calib_shoulder_angle) ? 0.0f : calib_shoulder_angle;
  calib_elbow_angle =    isnan(calib_elbow_angle)    ? 0.0f : calib_elbow_angle;
  gripper_open_angle =   isnan(gripper_open_angle)   ? 0.0f : gripper_open_angle;
  gripper_close_angle =  isnan(gripper_close_angle)  ? 0.0f : gripper_close_angle;
}