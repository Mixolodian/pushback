#pragma once
#include "Drivetrain/drive.h"
#include "robot-config.h"
#include "vex.h"

extern Drive chassis;

// PID Constants
void default_constants();
void odom_constants();

// Utility Functions
void delayedCall(void (*func)(), int delayMs);

// Drive to wall - drives until distance sensor reads target_distance (mm)
// Maintains heading using inertial sensor
void drive_to_wall(float target_distance, float drive_max_voltage,
                   float heading_kp, float settle_error, float timeout);
void drive_to_wall(float target_distance);

// Toggle functions
void toggleWings();
void toggleDescore();
void toggleMidDescore();
void toggleMid();

// Y position drift correction using distance sensor.
// Call once before a movement to snapshot, call again after to apply correction.
void snapCorrectY();
