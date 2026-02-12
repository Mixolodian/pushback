#include "vex.h"
#include "pid-tuning.h"

// Master toggle for PID tuning controls on the controller
bool enablePidTuning = false;

// Tuning mode: 0 = PID constants, 1 = Exit conditions
int tuningMode = 0;

// PID constant names for display
const char* pidConstantNames[] = {"kP", "kI", "kD", "maxV"};
int selectedPidConstant = 0;  // 0=kP, 1=kI, 2=kD, 3=maxVolt

// Exit condition names for display
const char* exitConditionNames[] = {"sErr", "sTim", "tOut"};
int selectedExitCondition = 0;  // 0=settleError, 1=settleTime, 2=timeout

// Current drive PID values (editable)
float tuning_kP = 1.6;
float tuning_kI = 0.0;
float tuning_kD = 12.0;
float tuning_maxVolt = 13.0;

// Current exit condition values (editable)
float tuning_settleError = 1.0;
float tuning_settleTime = 200.0;
float tuning_timeout = 3000.0;

// Increment amount for adjustments
float pidIncrement = 0.1;
float exitIncrement = 10.0;  // For time values in ms

// Test drive state (alternates between forward and backward)
bool driveForward = true;

// Update controller display with current PID or exit condition values
void updateControllerDisplay() {
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);

  if (tuningMode == 0) {
    // PID mode display
    Controller1.Screen.print("P%.1f I%.2f D%.1f V%.0f", tuning_kP, tuning_kI, tuning_kD, tuning_maxVolt);
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("[%s] X+ A- L:sw U:go", pidConstantNames[selectedPidConstant]);
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("DOWN: Exit Cond");
  } else {
    // Exit conditions mode display
    Controller1.Screen.print("Err%.1f Tm%.0f TO%.0f", tuning_settleError, tuning_settleTime, tuning_timeout);
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("[%s] X+ A- L:sw U:go", exitConditionNames[selectedExitCondition]);
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("DOWN: PID Mode");
  }
}

// Test drive function - resets position to 0,0,0 and drives 10 units forward then back
void pidTestDrive() {
  // Apply the tuned PID constants
  chassis.set_drive_constants(tuning_maxVolt, tuning_kP, tuning_kI, tuning_kD, 0);

  // Apply the tuned exit conditions
  chassis.set_drive_exit_conditions(tuning_settleError, tuning_settleTime, tuning_timeout);

  // Reset position to 0, 0, 0
  chassis.set_coordinates(0, 0, 0);

  if (driveForward) {
    // Drive to point 5 units ahead
    chassis.drive_to_point(0, 10);
  } else {
    // Drive to point 5 units behind
    chassis.drive_to_point(0, -10);
  }

  // Toggle direction for next call
  driveForward = !driveForward;
}

// Handle PID tuning button inputs
void handlePidTuningControls() {
  // Down button - toggle between PID and Exit Conditions mode
  if (Controller1.ButtonDown.pressing()) {
    while (Controller1.ButtonDown.pressing()) {}  // Wait for release
    tuningMode = (tuningMode + 1) % 2;
    updateControllerDisplay();
  }

  // Left arrow - cycle through constants
  if (Controller1.ButtonLeft.pressing()) {
    while (Controller1.ButtonLeft.pressing()) {}  // Wait for release
    if (tuningMode == 0) {
      selectedPidConstant = (selectedPidConstant + 1) % 4;  // 4 PID options now
    } else {
      selectedExitCondition = (selectedExitCondition + 1) % 3;
    }
    updateControllerDisplay();
  }

  // X button - increase selected constant
  if (Controller1.ButtonX.pressing()) {
    while (Controller1.ButtonX.pressing()) {}  // Wait for release
    if (tuningMode == 0) {
      switch (selectedPidConstant) {
        case 0: tuning_kP += pidIncrement; break;
        case 1: tuning_kI += pidIncrement; break;
        case 2: tuning_kD += pidIncrement; break;
        case 3: tuning_maxVolt += 0.5; break;
      }
    } else {
      switch (selectedExitCondition) {
        case 0: tuning_settleError += 0.1; break;
        case 1: tuning_settleTime += exitIncrement; break;
        case 2: tuning_timeout += 100.0; break;
      }
    }
    updateControllerDisplay();
  }

  // A button - decrease selected constant
  if (Controller1.ButtonA.pressing()) {
    while (Controller1.ButtonA.pressing()) {}  // Wait for release
    if (tuningMode == 0) {
      switch (selectedPidConstant) {
        case 0: tuning_kP = fmax(0, tuning_kP - pidIncrement); break;
        case 1: tuning_kI = fmax(0, tuning_kI - pidIncrement); break;
        case 2: tuning_kD = fmax(0, tuning_kD - pidIncrement); break;
        case 3: tuning_maxVolt = fmax(1, tuning_maxVolt - 0.5); break;
      }
    } else {
      switch (selectedExitCondition) {
        case 0: tuning_settleError = fmax(0.1, tuning_settleError - 0.1); break;
        case 1: tuning_settleTime = fmax(10, tuning_settleTime - exitIncrement); break;
        case 2: tuning_timeout = fmax(100, tuning_timeout - 100.0); break;
      }
    }
    updateControllerDisplay();
  }

  // Up button - run test drive
  if (Controller1.ButtonUp.pressing()) {
    while (Controller1.ButtonUp.pressing()) {}  // Wait for release
    pidTestDrive();
    updateControllerDisplay();
  }
}
