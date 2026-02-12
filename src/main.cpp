// main.cpp - Competition template with JAR-Template chassis config

#include "vex.h"
#include "pid-tuning.h"

using namespace vex;
competition Competition;

// Chassis Configuration
Drive chassis(
  TANK_ONE_FORWARD_ROTATION,

  // Left Motors
  motor_group(LeftMotor1, LeftMotor2, LeftMotor3),

  // Right Motors
  motor_group(RightMotor1, RightMotor2, RightMotor3),

  // Inertial sensor port
  PORT7,

  // Wheel diameter, gear ratio, gyro scale
  3.25,
  0.75,
  358.5,

  // Holonomic motor ports (leave as-is for tank drive)
  PORT1,      -PORT2,
  PORT3,      -PORT4,

  // Forward tracker: port, diameter, center distance
  PORT5,
  -2,
  -0.8,

  // Sideways tracker: port, diameter, center distance
  PORT8,
  2,
  -1
);

// Auton Selection State
int current_auton_selection = 2;
bool auto_started = false;

// Pre-Autonomous
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
  Inertial.calibrate();

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

// Autonomous
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

// User Control - R1:Outtake R2:Intake L1:Mid goal B:Intake+wings L2:Scoring
void usercontrol(void) {
  chassis.set_coordinates(0, 0, 0);
  GamePhase = OPCONTROL;
  subsystems::init();

  if (enablePidTuning) {
    updateControllerDisplay();
  }

  while (1) {
    int intakePct = 0;
    int hoodPct = 0;

    // PID tuning controls
    if (enablePidTuning) {
      handlePidTuningControls();
    }

    // Intake/roller control
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

    // Drivetrain
    chassis.control_arcade();

    wait(20, msec);
  }
}

// Main Entry Point
int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
