// autonUtil.cpp - PID configuration and utility functions for autonomous

#include "vex.h"
#include "robot-config.h"
#include "Autos/autonUtil.h"

// PID Constants Configuration
// PID format: (maxVoltage, kP, kI, kD, startI)
// Exit conditions format: (settle_error, settle_time, timeout)
void default_constants() {
  chassis.set_drive_constants(8, 3.2, 0, 20, 0);
  chassis.set_heading_constants(8, .4, 0, 1, 0);
  chassis.set_turn_constants(8, .5, 0, 4, 15);
  chassis.set_swing_constants(6, .3, 0, 2, 15);

  chassis.set_drive_exit_conditions(1.5, 250, 3900);
  chassis.set_turn_exit_conditions(2, 200, 2000);
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

// Drive to wall - maintains heading using inertial sensor
// relative_to_robot=true: drive target_distance inches via encoders
// relative_to_robot=false: drive until DistanceFront reads target_distance mm
void drive_to_wall(float target_distance, float drive_max_voltage,
                   float heading_kp, float settle_error, float timeout,
                   bool relative_to_robot) {

  float drive_kp = relative_to_robot ? 4 : 0.05;
  float drive_ki = 0.0;
  float drive_kd = relative_to_robot ? 17.0 : 0.5;

  float start_heading = Inertial.heading(degrees);
  float start_left = chassis.get_left_position_in();
  float start_right = chassis.get_right_position_in();

  float drive_error = target_distance;
  float drive_prev_error = target_distance;
  float drive_integral = 0;
  float drive_derivative = 0;
  float heading_prev_error = 0;

  float time_spent = 0;
  float settle_time = 0;
  float settle_threshold = 20;

  while (time_spent < timeout) {
    if (relative_to_robot) {
      float left_traveled = chassis.get_left_position_in() - start_left;
      float right_traveled = chassis.get_right_position_in() - start_right;
      float avg_traveled = (left_traveled + right_traveled) / 2.0;
      drive_error = target_distance - avg_traveled;
    } else {
      float current_distance = DistanceFront.objectDistance(mm);
      drive_error = current_distance - target_distance;
    }

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

  chassis.drive_with_voltage(0, 0);
}

// Simplified drive_to_wall (encoder mode with defaults)
void drive_to_wall(float target_distance) {
  drive_to_wall(target_distance, 6, 0.5, 1.0, 3000, true);
}
