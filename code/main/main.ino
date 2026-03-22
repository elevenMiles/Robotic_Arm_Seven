/* ROBOTIC ARM (Seven) - Author: Selçuk Yüksel
 * 
 * A 3-DOF robotic arm controlled via serial communication. Supports manual joint control, 
 * inverse kinematics, gripper control, waypoint list programming, and external software integration via handshake protocol.
 * -----------------------------------------------------------------------------------------------------------
 * ABOUT CALIBRATION: Due to mounting errors, servo joints may not point in the correct directions. 
 * Calibration corrects this offset and saves it to EEPROM so it persists across power cycles.
 * 
 * TO CALIBRATE: Manually enter angle values so that each joint should look exactly at it's 90 degree position
 * (REST POSITION OF THE ARM). After being ok with the joint positions enter calibration command -C 
 * to the serial monitor and hit enter. 
 * 
 * GRIPPER CALIBRATION:
 * To calibrate gripper use G(ANGLE) command to find a good open and close gripper angles (usually 45 degrees for open 0 degrees for close).
 * Then use -GO(ANGLE) command to save the open gripper angle and -GC(ANGLE) command to save the close gripper angle.
 * Before using GO / GC, you should define the open and close angles:
 * -GO(45) -- set and save the open angle
 * -GC(0)  -- set and save the close angle
 *
 * CALIBRATION RESET:
 * To reset all calibration values including gripper values enter -R command. This will set all the values to zero.
 * ------------------------------------------------------------------------------------------------------------
 * ABOUT AUTOMATIC CONTROL (Inverse Kinematics): This command takes 3-D space coordinates in centimeters. 
 * Moves the end effector (gripper) to the entered coordinates.
 * ------------------------------------------------------------------------------------------------------------
 * ABOUT COORDINATE SYSTEM
 * The shoulder servo's pivot point is the origin (0, 0, 0). All coordinates are in centimeters.
 * | Axis  |  Direction   |
 * |  ---  |     ---      |
 * |   X   |   Forward    | 
 * |   Y   | Left / Right |
 * |   Z   |  Up / Down   |
 * ------------------------------------------------------------------------------------------------------------
 * ABOUT HANDSHAKE 
 * If you want to control the robotic arm from another software or application. You can activate
 * handshake with (HA) command. This command prints "K" to the serial port after every update cycle. 
 * So you can read serial monitor from the other application or software and only send command if "K" is received.
 * 
 * While handshake is active, CALIBRATION (-C) and WAYPOINT LIST (+S, +W, +E, +M) commands are disabled.
 * These commands only usable while handshake is deactivate. Use (HD) command to deactivate handshake.  
 * ------------------------------------------------------------------------------------------------------------
 * ABOUT WAYPOINTS LIST
 * Waypoints let you record a sequence of positions and replay them in a loop. The workflow is always: Start → Waypoints → End → Move.
 * The list can hold up to 25 waypoints. All serial commands are blocked while the arm is executing a waypoint list. 
 * ------------------------------------------------------------------------------------------------------------
 * COMMANDS
 * 
 * CALIBRATION COMMANDS 
 * Calibrate and save -------------------------------------------------> -C
 * Reset calibration angles (0, 0, 0) and gripper angles and save -----> -R
 * Set gripper open angle and save ------------------------------------> -GO(ANGLE)
 * Set gripper close angle and save -----------------------------------> -GC(ANGLE)
 *
 * MOVEMENT COMMANDS
 * Set angular speed [degrees / second] -------------------------------> S(SPEED)
 *
 * Automatic control (Inverse Kinematics) -----------------------------> A(X Y Z)
 * Manual control -----------------------------------------------------> M(BASE_ANGLE SHOULDER_ANGLE ELBOW_ANGLE)
 * Set current gripper angle ------------------------------------------> G(ANGLE)
 * Open gripper -------------------------------------------------------> GO
 * Close gripper ------------------------------------------------------> GC
 * 
 * Move x cm up -------------------------------------------------------> U(x)
 * Move x cm down -----------------------------------------------------> D(x)
 * Move x cm left -----------------------------------------------------> L(x)
 * Move x cm right ----------------------------------------------------> R(x)
 * Move x cm forward --------------------------------------------------> F(x)
 * Move x cm backward -------------------------------------------------> B(x)
 *
 * Move 5 cm up -------------------------------------------------------> U
 * Move 5 cm down -----------------------------------------------------> D
 * Move 5 cm left -----------------------------------------------------> L
 * Move 5 cm right ----------------------------------------------------> R
 * Move 5 cm forward --------------------------------------------------> F
 * Move 5 cm backward -------------------------------------------------> B
 * 
 * Reset arm position -------------------------------------------------> O
 *
 * WAYPOINT LIST COMMANDS
 * Create a new list and save the current position as the start point -> +S
 * Save the current position and gripper state as a waypoint ----------> +W
 * Save the current position as the end point and close the list ------> +E
 * Begin looping through the waypoint list ----------------------------> +M
 *  
 * HANDSHAKE COMMANDS
 * Handshake active ---------------------------------------------------> HA
 * Handshake deactivated ----------------------------------------------> HD
 *
 */
// Adjusted to size of robotic arm. Don't change these values unless you make any modifications.
constexpr float LENGTH_SHOULDER_ELBOW {12.0f}; // [cm];
constexpr float LENGTH_ELBOW_GRIPPER  {12.0f}; // [cm];
// ------------------------------------------------------------------------------------------------------------
constexpr int BASE_SERVO_PIN     {6};
constexpr int SHOULDER_SERVO_PIN {9};
constexpr int ELBOW_SERVO_PIN    {10};
constexpr int GRIPPER_SERVO_PIN  {11};

// ------------------------------------------------------------------------------------------------------------
constexpr int COMMAND_BUFFER_SIZE {64};
// ------------------------------------------------------------------------------------------------------------

#include "servoConfig.h"
#include "roboticArm.h"

char command_buffer[COMMAND_BUFFER_SIZE];

RoboticArm roboticArm 
{
  LENGTH_SHOULDER_ELBOW,
  LENGTH_ELBOW_GRIPPER,
  ServoConfig(BASE_SERVO_PIN,     ServoRot::Normal),
  ServoConfig(SHOULDER_SERVO_PIN, ServoRot::Normal, 0, 110),
  ServoConfig(ELBOW_SERVO_PIN,    ServoRot::Reverse, 20, 180),
  ServoConfig(GRIPPER_SERVO_PIN,  ServoRot::Reverse, 0, 100)
};

void setup() 
{
  Serial.begin(115200);
  roboticArm.begin();
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
    uint8_t len {static_cast<uint8_t>(Serial.readBytesUntil('\n', command_buffer, COMMAND_BUFFER_SIZE - 1))};
    command_buffer[len] = '\0';

    for (uint8_t i {0}; i < len; ++i)
    {
      command_buffer[i] = toUpperCase(command_buffer[i]);
    }

    roboticArm.command(command_buffer);
  }

  roboticArm.update(delta_time);
}