// robot-config.h - Hardware declarations for all robot devices

#pragma once
using namespace vex;

// Core Components
extern brain Brain;
extern controller Controller1;

// Drivetrain Motors
extern motor LeftMotor1;
extern motor LeftMotor2;
extern motor LeftMotor3;

extern motor RightMotor1;
extern motor RightMotor2;
extern motor RightMotor3;

// Subsystem Motors
extern motor Intake;
extern motor TopRoller;
extern motor BackRoller;
extern motor_group IntakeMotors;

// Pneumatics
extern digital_out Hood;
extern digital_out Matchloader;
extern digital_out Wings;
extern digital_out Descore;

// Sensors
extern distance DistanceBack;
extern distance DistanceFront;
extern distance DistanceSide;
extern inertial Inertial;

// Initialization
void vexcodeInit(void);
