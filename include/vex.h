/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       vex.h                                                     */
/*    Author:       Vex Robotics                                              */
/*    Created:      1 Feb 2019                                                */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//

#pragma once
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <algorithm>

#include "v5.h"
#include "v5_vcs.h"

#include "robot-config.h"
#include "Drivetrain/odom.h"
#include "Drivetrain/drive.h"
#include "Drivetrain/util.h"
#include "Drivetrain/PID.h"
#include "Autos/autons.h"
#include "Autos/autons2.h"
#include "Autos/autonUtil.h"
#include "subsystems.h"
#include "images.h"

#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)