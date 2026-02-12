#pragma once
#include "JAR-Template/drive.h"
#include "robot-config.h"
#include "vex.h"

extern Drive chassis;

// PID Constants
void default_constants();
void odom_constants();

// Utility Functions
void delayedCall(void (*func)(), int delayMs);

// Drive to wall - maintains heading using inertial sensor
// relative_to_robot: true = encoder-based inches, false = distance sensor mm
void drive_to_wall(float target_distance, float drive_max_voltage,
                   float heading_kp, float settle_error, float timeout,
                   bool relative_to_robot);
void drive_to_wall(float target_distance);
