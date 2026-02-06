#pragma once
#include "JAR-Template/drive.h"
#include "robot-config.h"
#include "vex.h"

extern Drive chassis;

/*----------------------------------------------------------------------------*/
/*                        PID CONSTANTS CONFIGURATION                         */
/*----------------------------------------------------------------------------*/

void default_constants();
void odom_constants();

/*----------------------------------------------------------------------------*/
/*                           UTILITY FUNCTIONS                                */
/*----------------------------------------------------------------------------*/

// Delayed function call utility - calls func after delayMs without blocking
void delayedCall(void (*func)(), int delayMs);

/*----------------------------------------------------------------------------*/
/*                        DRIVE TO WALL FUNCTIONS                             */
/*----------------------------------------------------------------------------*/

// Drive while maintaining heading using inertial sensor
// relative_to_robot: true  = drive target_distance inches using encoders (relative to robot)
//                    false = drive until DistanceFront reads target_distance mm from wall
void drive_to_wall(float target_distance, float drive_max_voltage,
                   float heading_kp, float settle_error, float timeout,
                   bool relative_to_robot);
void drive_to_wall(float target_distance);
