/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Description:  Competition template with JAR-Template chassis config     */
/*                                                                            */
/*    CONTENTS:                                                               */
/*    1. Includes & Globals                                                   */
/*    2. Chassis Configuration                                                */
/*    3. Pre-Autonomous (Auton Selection)                                     */
/*    4. Autonomous                                                           */
/*    5. User Control (Driver)                                                */
/*    6. Main Entry Point                                                     */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "pid-tuning.h"

using namespace vex;
competition Competition;

/*============================================================================*/
/*                                                                            */
/*                      SECTION 1: CHASSIS CONFIGURATION                      */
/*                                                                            */
/*    Configure your drivetrain settings here. This uses JAR-Template's      */
/*    Drive class for motion control and odometry.                            */
/*                                                                            */
/*============================================================================*/

Drive chassis(

  /*--------------------------------------------------------------------------*/
  /*                         DRIVE TYPE SELECTION                             */
  /*--------------------------------------------------------------------------*/
  /*  Available options:                                                      */
  /*    ZERO_TRACKER_NO_ODOM     - No tracking wheels, no odometry            */
  /*    ZERO_TRACKER_ODOM        - No tracking wheels, IMU-based odometry     */
  /*    TANK_ONE_FORWARD_ENCODER - One forward encoder                        */
  /*    TANK_ONE_FORWARD_ROTATION- One forward rotation sensor                */
  /*    TANK_ONE_SIDEWAYS_ENCODER- One sideways encoder                       */
  /*    TANK_ONE_SIDEWAYS_ROTATION-One sideways rotation sensor               */
  /*    TANK_TWO_ENCODER         - Two encoders                               */
  /*    TANK_TWO_ROTATION        - Two rotation sensors                       */
  /*    HOLONOMIC_TWO_ENCODER    - Holonomic with two encoders                */
  /*    HOLONOMIC_TWO_ROTATION   - Holonomic with two rotation sensors        */
  /*--------------------------------------------------------------------------*/
  TANK_ONE_FORWARD_ROTATION,

  /*--------------------------------------------------------------------------*/
  /*                           DRIVE MOTORS                                   */
  /*--------------------------------------------------------------------------*/
  // Left Motors
  motor_group(LeftMotor1, LeftMotor2, LeftMotor3),

  // Right Motors
  motor_group(RightMotor1, RightMotor2, RightMotor3),

  /*--------------------------------------------------------------------------*/
  /*                         INERTIAL SENSOR PORT                             */
  /*--------------------------------------------------------------------------*/
  PORT7,

  /*--------------------------------------------------------------------------*/
  /*                          WHEEL PARAMETERS                                */
  /*--------------------------------------------------------------------------*/
  // Wheel diameter in inches (4" omnis are closer to 4.125")
  3.25,

  // External gear ratio (motor teeth / wheel teeth)
  // Direct drive = 1.0, 84:60 = 1.4
  0.75,

  // Gyro scale factor (adjust if 360° turn doesn't read 360°)
  358.5,

  /*--------------------------------------------------------------------------*/
  /*                    POSITION TRACKING (ADVANCED)                          */
  /*    Only modify if using tracking wheels for odometry                     */
  /*--------------------------------------------------------------------------*/

  // Holonomic motor ports (leave as-is for tank drive)
  // LF:        RF:
  PORT1,      -PORT2,
  // LB:        RB:
  PORT3,      -PORT4,

  // Forward tracker port
  PORT5,

  // Forward tracker diameter (negative reverses direction)
  -2,

  // Forward tracker center distance (positive = right of center)
  -0.8,

  // Sideways tracker port
  PORT8,

  // Sideways tracker diameter (negative reverses direction)
  2,

  // Sideways tracker center distance (positive = behind center)
  -1
);

/*============================================================================*/
/*                                                                            */
/*                     SECTION 2: AUTON SELECTION STATE                       */
/*                                                                            */
/*============================================================================*/

int current_auton_selection = 2;  // Default auton (0-indexed)
bool auto_started = false;        // Flag to exit pre-auton loop

/*============================================================================*/
/*                                                                            */
/*                     SECTION 3: PRE-AUTONOMOUS ROUTINE                      */
/*                                                                            */
/*    Runs before the match starts. Displays auton selector on brain screen.  */
/*    Tap the screen to cycle through available autonomous routines.          */
/*                                                                            */
/*============================================================================*/

// Task to print robot debug info to terminal (2 lines, overwrites in place)
int printDebugTask() {
  while (true) {
    printf("\rX:%.2f Y:%.2f H:%.2f | DF:%.0f DB:%.0f DS:%.0f      "
           "\n\rDL:%.0f/%.0f/%.0f DR:%.0f/%.0f/%.0f I:%.0f B:%.0f C      \033[A",
           chassis.get_X_position(),
           chassis.get_Y_position(),
           chassis.get_absolute_heading(),
           DistanceFront.objectDistance(mm),
           DistanceBack.objectDistance(mm),
           DistanceSide.objectDistance(mm),
           LeftMotor1.temperature(celsius),
           LeftMotor2.temperature(celsius),
           LeftMotor3.temperature(celsius),
           RightMotor1.temperature(celsius),
           RightMotor2.temperature(celsius),
           RightMotor3.temperature(celsius),
           Intake.temperature(celsius),
           BackRoller.temperature(celsius));
    fflush(stdout);
    task::sleep(100);
  }
  return 0;
}

void pre_auton() {
  vexcodeInit();
  default_constants();

  // Start debug print task (only instance of terminal output)
  task debugTask(printDebugTask);

  // Auton selection loop - runs until match starts
  while (!auto_started) {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    switch (current_auton_selection) {
      case 0:
        Brain.Screen.print("Skills");
        break;
      case 1:
        Brain.Screen.print("High Side Auto");
        break;
      case 2:
        Brain.Screen.print("Low Side Auto");
        break;
      case 3:
        Brain.Screen.print("Solo AWP");
        break;
      case 4:
        Brain.Screen.print("Test/Tuning");
        break;
    }

    if (Brain.Screen.pressing()) {
      while (Brain.Screen.pressing()) {}
      current_auton_selection++;
      if (current_auton_selection > 4) {
        current_auton_selection = 0;
      }
    }

    task::sleep(10);
  }
}

/*============================================================================*/
/*                                                                            */
/*                       SECTION 4: AUTONOMOUS ROUTINE                        */
/*                                                                            */
/*    Executes the selected autonomous program.                               */
/*    Modify the switch cases to call your auton functions from autons.cpp    */
/*                                                                            */
/*============================================================================*/

void autonomous(void) {
  current_auton_selection = 1;  // Override for testing (remove in competition)
  auto_started = true;
  GamePhase = AUTO;

  switch (current_auton_selection) {
    case 0:
      SKILLS();
      break;
    case 1:
      high_side_auto();
      break;
    case 2:
      low_side_auto();
      break;
    case 3:
      turn_test();
      break;
    case 4:
      full_test();
      break;
  }
}

/*============================================================================*/
/*                                                                            */
/*                      SECTION 5: USER CONTROL (DRIVER)                      */
/*                                                                            */
/*    Main driver control loop. Handles:                                      */
/*    - Drivetrain control (arcade style)                                     */
/*    - Intake/roller control via triggers                                    */
/*    - Pneumatic state management                                            */
/*                                                                            */
/*    CONTROL SCHEME:                                                         */
/*    R1 - Outtake (reverse intake + rollers)                                 */
/*    R2 - Intake (forward intake + retract pneumatics)                       */
/*    L1 - Mid goal (deploy wings)                                            */
/*    B  - Intake + wings                                                     */
/*    L2 - Scoring (100% speed + deploy hood only)                            */
/*                                                                            */
/*============================================================================*/

void usercontrol(void) {
  chassis.set_coordinates(0, 0, 0);
  GamePhase = OPCONTROL;
  subsystems::init();

  if (enablePidTuning) {
    updateControllerDisplay();
  }

  /*--------------------------------------------------------------------------*/
  /*                         MAIN CONTROL LOOP                                */
  /*--------------------------------------------------------------------------*/
  while (1) {
    int intakePct = 0;
    int hoodPct = 0;

    /*------------------------------------------------------------------------*/
    /*                       PID TUNING CONTROLS                              */
    /*------------------------------------------------------------------------*/
    if (enablePidTuning) {
      handlePidTuningControls();
    }

    /*------------------------------------------------------------------------*/
    /*                       INTAKE/ROLLER CONTROL                            */
    /*------------------------------------------------------------------------*/
    if (Controller1.ButtonR1.pressing()) {
      intakePct = -100;
      hoodPct = -100;
    } else if (Controller1.ButtonR2.pressing()) {
      intakePct = 100;
      hoodPct = 100;
      Hood.set(false);
      Wings.set(false);
    } else if (Controller1.ButtonL1.pressing()) {
      intakePct = 100;
      hoodPct = 100;
      Hood.set(false);
      Wings.set(true);
    } else if (Controller1.ButtonB.pressing()) {
      intakePct = 100;
      hoodPct = 100;
      Wings.set(true);
    } else if (Controller1.ButtonL2.pressing()) {
      intakePct = 100;
      hoodPct = 100;
      Hood.set(true);
      Wings.set(false);
    }

    Intake.spin(fwd, intakePct, pct);
    BackRoller.spin(fwd, hoodPct, pct);

    /*------------------------------------------------------------------------*/
    /*                        DRIVETRAIN CONTROL                              */
    /*------------------------------------------------------------------------*/
    chassis.control_arcade();

    wait(20, msec);
  }
}

/*============================================================================*/
/*                                                                            */
/*                       SECTION 6: MAIN ENTRY POINT                          */
/*                                                                            */
/*============================================================================*/

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
