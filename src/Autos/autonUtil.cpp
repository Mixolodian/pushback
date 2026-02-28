// autonUtil.cpp - PID configuration and utility functions for autonomous

#include "vex.h"
#include "robot-config.h"
#include "Autos/autonUtil.h"

// PID Constants Configuration
// PID format: (maxVoltage, kP, kI, kD, startI)
// Exit conditions format: (settle_error, settle_time, timeout)
void default_constants() {
  chassis.set_drive_constants(8.5, 3.3, 0, 20, 0);
  chassis.set_heading_constants(8, .4, 0, 1, 0);
  chassis.set_turn_constants(8, .5, 0, 4, 15);
  chassis.set_swing_constants(6, .3, 0, 2, 15);

  chassis.set_drive_exit_conditions(1, 250, 2500);
  chassis.set_turn_exit_conditions(1.5, 150, 1100);
  chassis.set_swing_exit_conditions(3, 100, 3000);
}

// Odometry-optimized constants
void odom_constants() {
  default_constants();
  chassis.heading_max_voltage = 12;
  chassis.drive_max_voltage = 8;
  chassis.drive_settle_error = 1;
  chassis.boomerang_lead = .5;
  chassis.drive_min_voltage = 0;
}

// Delayed function call utility
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

// Schedules func to be called after delayMs (non-blocking)
void delayedCall(void (*func)(), int delayMs) {
  DelayedCall* call = new DelayedCall{func, delayMs};
  vex::task t(delayedCallTask, call);
}

// Drive to wall - drives until distance sensor reads target_distance (mm)
// Maintains heading using inertial sensor
void drive_to_wall(float target_distance, float drive_max_voltage,
                   float heading_kp, float settle_error, float timeout) {

  float drive_kp = 0.05;
  float drive_ki = 0.0;
  float drive_kd = 0.5;

  float start_heading = Inertial.heading(degrees);

  float drive_error = target_distance;
  float drive_prev_error = target_distance;
  float drive_integral = 0;
  float drive_derivative = 0;
  float heading_prev_error = 0;

  float time_spent = 0;
  float settle_time = 0;
  float settle_threshold = 20;

  while (time_spent < timeout) {
    float current_distance = DistanceFront.objectDistance(mm);
    drive_error = current_distance - target_distance;

    drive_integral += drive_error;
    drive_derivative = drive_error - drive_prev_error;
    drive_prev_error = drive_error;

    float drive_output = (drive_kp * drive_error) +
                         (drive_ki * drive_integral) +
                         (drive_kd * drive_derivative);

    if (drive_output > drive_max_voltage) drive_output = drive_max_voltage;
    if (drive_output < -drive_max_voltage) drive_output = -drive_max_voltage;

    float current_heading = Inertial.heading(degrees);
    float heading_error = start_heading - current_heading;
    if (heading_error > 180) heading_error -= 360;
    if (heading_error < -180) heading_error += 360;

    float heading_derivative = heading_error - heading_prev_error;
    heading_prev_error = heading_error;

    float heading_correction = (heading_kp * heading_error) + (0.1 * heading_derivative);
    float max_correction = 3.0;
    if (heading_correction > max_correction) heading_correction = max_correction;
    if (heading_correction < -max_correction) heading_correction = -max_correction;

    float left_voltage = drive_output + heading_correction;
    float right_voltage = drive_output - heading_correction;
    chassis.drive_with_voltage(left_voltage, right_voltage);

    if (fabs(drive_error) < settle_error) {
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

  chassis.drive_with_voltage(0, 0);
}

// Simplified drive_to_wall with defaults
void drive_to_wall(float target_distance) {
  drive_to_wall(target_distance, 7.5, 0.8, 8.0, 2000);
}

// Toggle functions
bool wingState = false;
bool descoreState = false;
bool MidDescoreState = false;

void toggleWings() {
  wingState = !wingState;
  Matchloader.set(wingState);
}

void toggleDescore() {
  descoreState = !descoreState;
  Descore.set(descoreState);
}

void toggleMidDescore() {
  MidDescoreState = !MidDescoreState;
  if (MidDescoreState) {
    Hood.set(false);
  }
  MidDescore.set(MidDescoreState);
}

void toggleMid(){
  Wings.set(true);
}

// Snapshot and correct Y position using distance sensor drift compensation.
// First call: records current odometry Y and front distance reading.
// Second call: computes how far the robot actually moved (via distance sensor delta),
// converts mm -> inches, and re-sets Y accordingly. Resets state after second call.
static bool _snapTaken = false;
static float _snapY    = 0;
static float _snapDist = 0;

void snapCorrectY() {
  if (!_snapTaken) {
    // --- Snapshot ---
    _snapY    = chassis.get_Y_position();
    _snapDist = DistanceFront.objectDistance(mm);
    _snapTaken = true;
  } else {
    // --- Correct ---
    float currentDist       = DistanceFront.objectDistance(mm);
    // Positive when robot moved toward the wall (distance shrank)
    float distDeltaInches   = (_snapDist - currentDist) / 25.4f;
    float correctedY        = _snapY + distDeltaInches;
    chassis.set_coordinates(chassis.get_X_position(), correctedY, Inertial.heading(degrees));
    _snapTaken = false; // reset for next pair of calls
  }
}
