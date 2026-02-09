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

// Task to print robot coordinates to terminal
int printOdomTask() {
  while (true) {
    printf("\rX: %.2f  Y: %.2f  Heading: %.2f      ",
           chassis.get_X_position(),
           chassis.get_Y_position(),
           chassis.get_absolute_heading());
    fflush(stdout);
    task::sleep(100);
  }
  return 0;
}

void pre_auton() {
  // Initialize robot hardware
  vexcodeInit();
  default_constants();

  // Start task to print coordinates to terminal
  task odomPrintTask(printOdomTask);

  // Auton selection loop - runs until match starts
  while (!auto_started) {

    // Display auton name based on current selection
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

    // Handle screen tap to cycle autons
    if (Brain.Screen.pressing()) {
      while (Brain.Screen.pressing()) {}  // Wait for release
      current_auton_selection++;
      if (current_auton_selection > 4) {
        current_auton_selection = 0;  // Wrap around
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
  current_auton_selection =1;  // Override for testing (remove in competition)
  auto_started = true;
  GamePhase = AUTO;

  switch (current_auton_selection) {
    case 0:  // Skills
      SKILLS();
      break;

    case 1:  // High Side Auto
      high_side_auto();
      break;

    case 2:  // Low Side Auto
      low_side_auto();
      break;

    case 3:  // Solo AWP
      turn_test();
      break;

    case 4:  // Test/Tuning
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
/*    L1 - Mid goal (60% speed + deploy hood/wings)                           */
/*    L2 - Scoring (100% speed + deploy hood only)                            */
/*                                                                            */
/*============================================================================*/

void usercontrol(void) {
  chassis.set_coordinates(0, 0, 0);
  GamePhase = OPCONTROL;
  subsystems::init();
  vexcodeInit();

  // Initialize PID display on controller
  if (enablePidTuning) {
    updateControllerDisplay();
  }

  // Piston state - stays retracted once triggered
  bool pistonRetracted = false;

  /*--------------------------------------------------------------------------*/
  /*                    DISTANCE TRACKING (OPTIONAL)                          */
  /*    Uncomment to enable 3-sensor position tracking                        */
  /*--------------------------------------------------------------------------*/
  /*
  distanceTracker = new DistanceTracking(3000.0, 3000.0, 3000.0);
  distanceTracker->setSensorOffsets(0.0, 6.0, -6.0, 0.0, 6.0, 0.0);
  distanceTracker->setFieldDimensions(144.0, 144.0);
  vex::task trackingTask(distanceTrackingTask);
  */

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
      // OUTTAKE - Reverse both systems
      intakePct = -100;
      hoodPct = -100;
    }
    else if (Controller1.ButtonR2.pressing()) {
      // INTAKE - Forward + retract pneumatics
      intakePct = 100;
      hoodPct = 100;
      Hood.set(false);
      Wings.set(false);
    }
    else if (Controller1.ButtonL1.pressing()) {
      // MID GOAL
      intakePct = 100;
      hoodPct = 100;
      Hood.set(false);
      Wings.set(true);
      }
       else if(Controller1.ButtonB.pressing()) {
      intakePct = 100;
      hoodPct = 100;
      Wings.set(true);
    }
    
  
    else if (Controller1.ButtonL2.pressing()) {
      // SCORING - Full speed + hood only
      intakePct = 100;
      hoodPct = 100;
      Hood.set(true);
      Wings.set(false);
    }

    // Apply motor speeds
    Intake.spin(fwd, intakePct, pct);
    BackRoller.spin(fwd, hoodPct, pct);

    /*------------------------------------------------------------------------*/
    /*                        DRIVETRAIN CONTROL                              */
    /*------------------------------------------------------------------------*/
    chassis.control_arcade();
    
    /*------------------------------------------------------------------------*/
    /*                          DEBUG OUTPUT                                  */
    /*------------------------------------------------------------------------*/
    // Handle debug menu page switching and draw
    

    // Print to terminal
    
    // Prevent CPU hogging
    wait(20, msec);
  }
}

/*============================================================================*/
/*                                                                            */
/*                       SECTION 6: MAIN ENTRY POINT                          */
/*                                                                            */
/*============================================================================*/

int main() {
  // Register competition callbacks
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run pre-autonomous (auton selector)
  pre_auton();

  // Keep program alive
  while (true) {
    wait(100, msec);
  }
}
