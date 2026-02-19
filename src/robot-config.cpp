// robot-config.cpp - Hardware definitions
// PORT MAP: 1-RightMotor1, 2-RightMotor2, 3-RightMotor3, 6-LeftMotor2,
//           7-Inertial, 9-LeftMotor3, 10-LeftMotor1, 11-Intake,
//           16-TopRoller, 17-DistanceSide, 18-DistanceFront, 19-DistanceBack,
//           20-BackRoller | 3-Wire: A-Hood, B-Matchloader, C-Wings, D-Descore

#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// Core Components
brain Brain;
controller Controller1 = controller(primary);

// Drivetrain Motors - motor(port, gearRatio, reversed)
motor LeftMotor1 = motor(PORT10, ratio18_1, true);
motor LeftMotor2 = motor(PORT6, ratio18_1, true);
motor LeftMotor3 = motor(PORT9, ratio18_1, true);

motor RightMotor1 = motor(PORT1, ratio18_1, false);
motor RightMotor2 = motor(PORT2, ratio18_1, false);
motor RightMotor3 = motor(PORT3, ratio18_1, false);

// Subsystem Motors
motor Intake = motor(PORT11, ratio6_1, true);
motor BackRoller = motor(PORT20, ratio6_1, false);
motor TopRoller = motor(PORT16, false);

motor_group IntakeMotors = motor_group(Intake, BackRoller);

// Pneumatics
digital_out Hood = digital_out(Brain.ThreeWirePort.A);
digital_out Matchloader = digital_out(Brain.ThreeWirePort.B);
digital_out Wings = digital_out(Brain.ThreeWirePort.C);
digital_out Descore = digital_out(Brain.ThreeWirePort.D);
digital_out MidDescore = digital_out(Brain.ThreeWirePort.E);

// Sensors
distance DistanceBack = distance(PORT18);
distance DistanceFront = distance(PORT19);
distance DistanceSide = distance(PORT17);

inertial Inertial = inertial(PORT7);

// Initialization
void vexcodeInit(void) {
}
