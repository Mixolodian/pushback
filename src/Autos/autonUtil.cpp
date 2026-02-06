/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       autonUtil.cpp                                             */
/*    Description:  PID configuration and utility functions for autonomous    */
/*                                                                            */
/*    CONTENTS:                                                               */
/*    1. PID Constants Configuration                                          */
/*    2. Utility Functions (delayedCall)                                      */
/*    3. Drive to Wall Functions                                              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "robot-config.h"
#include "Autos/autonUtil.h"

/*============================================================================*/
/*                                                                            */
/*                    SECTION 1: PID CONSTANTS CONFIGURATION                  */
/*                                                                            */
/*============================================================================*/

/**
 * @brief Sets default PID constants for autonomous movement
 *
 * PID FORMAT: (maxVoltage, kP, kI, kD, startI)
 *   - maxVoltage: Maximum motor voltage (0-12)
 *   - kP: Proportional gain
 *   - kI: Integral gain
 *   - kD: Derivative gain
 *   - startI: Error threshold to start integral accumulation
 *
 * EXIT CONDITIONS FORMAT: (settle_error, settle_time, timeout)
 *   - settle_error: Error threshold to consider "settled"
 *   - settle_time: Time (ms) to stay within settle_error
 *   - timeout: Maximum time (ms) before giving up
 */
void default_constants() {
  // Movement PID constants
  chassis.set_drive_constants(8, 3.5, 0, 19, 0);    // Forward/backward
  chassis.set_heading_constants(7, .4, 0, 1, 0);     // Heading correction
  chassis.set_turn_constants(7, .5, 0, 4, 15);       // Point turns
  chassis.set_swing_constants(6, .3, 0, 2, 15);      // Swing turns

  // Exit conditions
  chassis.set_drive_exit_conditions(1.5, 250, 3400);
  chassis.set_turn_exit_conditions(3, 200, 2000);
  chassis.set_swing_exit_conditions(3, 100, 3000);
}

/**
 * @brief Sets PID constants optimized for odometry-based movements
 *
 * Odometry movements (drive_to_point, etc.) generally work better with:
 *   - Lower max voltage for smoother control
 *   - Larger settle error tolerance
 */
void odom_constants() {
  default_constants();
  chassis.heading_max_voltage = 12;
  chassis.drive_max_voltage = 8;
  chassis.drive_settle_error = 1;
  chassis.boomerang_lead = .5;
  chassis.drive_min_voltage = 0;
}

/*============================================================================*/
/*                                                                            */
/*                      SECTION 2: UTILITY FUNCTIONS                          */
/*                                                                            */
/*============================================================================*/

/**
 * @brief Delayed function call structure and task
 * Allows calling a function after a delay while autonomous continues
 */
struct DelayedCall {
  void (*func)();
  int delayMs;
};

int delayedCallTask(void* params) {
  DelayedCall* call = (DelayedCall*)params;
  vex::this_thread::sleep_for(call->delayMs);
  call->func();
  delete call;
  return 0;
}

/**
 * @brief Schedules a function to be called after a delay (non-blocking)
 * @param func The function to call (must be void with no parameters)
 * @param delayMs Delay in milliseconds before calling the function
 *
 * Usage: delayedCall(myFunction, 500);  // Calls myFunction after 500ms
 * The autonomous routine continues immediately without waiting.
 */
void delayedCall(void (*func)(), int delayMs) {
  DelayedCall* call = new DelayedCall{func, delayMs};
  vex::task t(delayedCallTask, call);
}

/*============================================================================*/
/*                                                                            */
/*                    SECTION 3: DRIVE TO WALL FUNCTIONS                      */
/*                                                                            */
/*============================================================================*/

/**
 * @brief Drive while maintaining heading using inertial sensor
 *
 * Uses the inertial sensor to keep the robot driving straight by correcting
 * any drift from the starting heading.
 *
 * @param target_distance If relative_to_robot=true: distance to travel in inches
 *                        If relative_to_robot=false: target distance from wall in mm
 * @param drive_max_voltage Maximum drive voltage (0-12)
 * @param heading_kp Proportional gain for heading correction
 * @param settle_error Error threshold to consider settled (inches or mm depending on mode)
 * @param timeout Maximum time in ms before giving up
 * @param relative_to_robot If true, drive target_distance inches using encoders (relative to robot)
 *                          If false, drive until DistanceFront reads target_distance mm from wall
 */
void drive_to_wall(float target_distance, float drive_max_voltage,
                   float heading_kp, float settle_error, float timeout,
                   bool relative_to_robot) {

  // PID constants for forward drive
  float drive_kp = relative_to_robot ? 4 : 0.05;  // Lower kp for distance sensor (mm vs inches)
  float drive_ki = 0.0;
  float drive_kd = relative_to_robot ? 17.0 : 0.5;

  // Record starting heading from inertial sensor
  float start_heading = Inertial.heading(degrees);

  // Store starting position (only used if relative_to_robot)
  float start_left = chassis.get_left_position_in();
  float start_right = chassis.get_right_position_in();

  // PID variables for drive
  float drive_error = target_distance;
  float drive_prev_error = target_distance;
  float drive_integral = 0;
  float drive_derivative = 0;

  // Heading correction variables
  float heading_prev_error = 0;

  // Timing
  float time_spent = 0;
  float settle_time = 0;
  float settle_threshold = 20; // ms to stay settled

  while (time_spent < timeout) {
    // Calculate drive error based on mode
    if (relative_to_robot) {
      // Use encoders - error is target minus distance traveled
      float left_traveled = chassis.get_left_position_in() - start_left;
      float right_traveled = chassis.get_right_position_in() - start_right;
      float avg_traveled = (left_traveled + right_traveled) / 2.0;
      drive_error = target_distance - avg_traveled;
    } else {
      // Use front distance sensor - error is current distance minus target
      float current_distance = DistanceFront.objectDistance(mm);
      drive_error = current_distance - target_distance;
    }

    drive_integral += drive_error;
    drive_derivative = drive_error - drive_prev_error;
    drive_prev_error = drive_error;

    float drive_output = (drive_kp * drive_error) +
                         (drive_ki * drive_integral) +
                         (drive_kd * drive_derivative);

    // Clamp drive output
    if (drive_output > drive_max_voltage) drive_output = drive_max_voltage;
    if (drive_output < -drive_max_voltage) drive_output = -drive_max_voltage;

    // Heading correction using inertial sensor (keeps robot driving straight)
    float current_heading = Inertial.heading(degrees);
    float heading_error = start_heading - current_heading;

    // Handle wraparound (e.g., 359 to 1 degrees)
    if (heading_error > 180) heading_error -= 360;
    if (heading_error < -180) heading_error += 360;

    float heading_derivative = heading_error - heading_prev_error;
    heading_prev_error = heading_error;

    // Heading correction (positive error = drifted left, turn right)
    float heading_correction = (heading_kp * heading_error) + (0.1 * heading_derivative);

    // Clamp heading correction
    float max_correction = 3.0;
    if (heading_correction > max_correction) heading_correction = max_correction;
    if (heading_correction < -max_correction) heading_correction = -max_correction;

    // Apply voltages with heading correction
    float left_voltage = drive_output + heading_correction;
    float right_voltage = drive_output - heading_correction;

    chassis.drive_with_voltage(left_voltage, right_voltage);

    // Check if settled
    if (fabs(drive_error) < 5) {
      settle_time += 10;
      if (settle_time >= settle_threshold) {
        break;
      }
    } else {
      settle_time = 0;
    }

    time_spent += 10;
    task::sleep(10);
  }

  // Stop motors
  chassis.drive_with_voltage(0, 0);
}

/**
 * @brief Simplified drive_to_wall with just distance parameter (encoder mode)
 */
void drive_to_wall(float target_distance) {
  drive_to_wall(target_distance, 6, 0.5, 1.0, 3000, true);
}
