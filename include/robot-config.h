/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       robot-config.h                                            */
/*    Description:  Hardware declarations for all robot devices               */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#pragma once
using namespace vex;

/*----------------------------------------------------------------------------*/
/*                            CORE COMPONENTS                                 */
/*----------------------------------------------------------------------------*/

extern brain Brain;
extern controller Controller1;

/*----------------------------------------------------------------------------*/
/*                            DRIVETRAIN MOTORS                               */
/*----------------------------------------------------------------------------*/

// Left side motors (3-motor drive)
extern motor LeftMotor1;
extern motor LeftMotor2;
extern motor LeftMotor3;

// Right side motors (3-motor drive)
extern motor RightMotor1;
extern motor RightMotor2;
extern motor RightMotor3;

/*----------------------------------------------------------------------------*/
/*                           SUBSYSTEM MOTORS                                 */
/*----------------------------------------------------------------------------*/

extern motor Intake;
extern motor TopRoller;
extern motor BackRoller;
extern motor_group IntakeMotors;

/*----------------------------------------------------------------------------*/
/*                             PNEUMATICS                                     */
/*----------------------------------------------------------------------------*/

extern digital_out Hood;
extern digital_out Matchloader;
extern digital_out Wings;
extern digital_out Descore;

/*----------------------------------------------------------------------------*/
/*                              SENSORS                                       */
/*----------------------------------------------------------------------------*/

// Distance sensors
extern distance DistanceBack;
extern distance DistanceFront;
extern distance DistanceSide;

// Inertial sensor for heading
extern inertial Inertial;

/*----------------------------------------------------------------------------*/
/*                           INITIALIZATION                                   */
/*----------------------------------------------------------------------------*/

void vexcodeInit(void);
