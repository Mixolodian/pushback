#pragma once
#include "Drivetrain/drive.h"

extern Drive chassis;

// Master toggle for PID tuning controls on the controller
extern bool enablePidTuning;

// Tuning mode: 0 = PID constants, 1 = Exit conditions
extern int tuningMode;

// PID constant names for display
extern const char* pidConstantNames[];
extern int selectedPidConstant;

// Exit condition names for display
extern const char* exitConditionNames[];
extern int selectedExitCondition;

// Current drive PID values (editable)
extern float tuning_kP;
extern float tuning_kI;
extern float tuning_kD;
extern float tuning_maxVolt;

// Current exit condition values (editable)
extern float tuning_settleError;
extern float tuning_settleTime;
extern float tuning_timeout;

// Increment amount for adjustments
extern float pidIncrement;
extern float exitIncrement;

// Test drive state (alternates between forward and backward)
extern bool driveForward;

// Update controller display with current values
void updateControllerDisplay();

// Test drive function - drives 10 inches forward or backward
void pidTestDrive();

// Handle PID tuning button inputs (call this in usercontrol loop)
void handlePidTuningControls();
