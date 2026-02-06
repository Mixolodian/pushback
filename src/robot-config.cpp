/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       robot-config.cpp                                          */
/*    Description:  Hardware definitions for all robot devices                */
/*                                                                            */
/*    PORT MAP:                                                               */
/*    ---------                                                               */
/*    PORT1  - RightMotor1          PORT11 - Intake                           */
/*    PORT2  - RightMotor2          PORT16 - TopRoller                        */
/*    PORT3  - RightMotor3          PORT17 - DistanceSide                     */
/*    PORT6  - LeftMotor2           PORT18 - DistanceFront                    */
/*    PORT7  - Inertial             PORT19 - DistanceBack                     */
/*    PORT9  - LeftMotor3           PORT20 - BackRoller                       */
/*    PORT10 - LeftMotor1                                                     */
/*                                                                            */
/*    3-WIRE PORTS:                                                           */
/*    A - Hood          B - Matchloader                                       */
/*    C - Wings         D - Descore                                           */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

/*----------------------------------------------------------------------------*/
/*                            CORE COMPONENTS                                 */
/*----------------------------------------------------------------------------*/

brain Brain;
controller Controller1 = controller(primary);

/*----------------------------------------------------------------------------*/
/*                            DRIVETRAIN MOTORS                               */
/*    Motor constructor: motor(port, gearRatio, reversed)                     */
/*    - ratio18_1 = 200 RPM (green cartridge)                                 */
/*    - ratio6_1  = 600 RPM (blue cartridge)                                  */
/*----------------------------------------------------------------------------*/

// Left side motors - ALL REVERSED for correct drive direction
motor LeftMotor1 = motor(PORT10, ratio18_1, true);
motor LeftMotor2 = motor(PORT6, ratio18_1, true);
motor LeftMotor3 = motor(PORT9, ratio18_1, true);

// Right side motors - NOT REVERSED
motor RightMotor1 = motor(PORT1, ratio18_1, false);
motor RightMotor2 = motor(PORT2, ratio18_1, false);
motor RightMotor3 = motor(PORT3, ratio18_1, false);

/*----------------------------------------------------------------------------*/
/*                           SUBSYSTEM MOTORS                                 */
/*----------------------------------------------------------------------------*/

motor Intake = motor(PORT11, ratio6_1, true);      // 600 RPM, reversed
motor BackRoller = motor(PORT20, ratio6_1, false); // 600 RPM
motor TopRoller = motor(PORT16, false);            // Default cartridge

motor_group IntakeMotors = motor_group(Intake, BackRoller);

/*----------------------------------------------------------------------------*/
/*                             PNEUMATICS                                     */
/*    Connected via 3-wire ports on the Brain                                 */
/*----------------------------------------------------------------------------*/

digital_out Hood = digital_out(Brain.ThreeWirePort.A);
digital_out Matchloader = digital_out(Brain.ThreeWirePort.B);
digital_out Wings = digital_out(Brain.ThreeWirePort.C);
digital_out Descore = digital_out(Brain.ThreeWirePort.D);

/*----------------------------------------------------------------------------*/
/*                              SENSORS                                       */
/*----------------------------------------------------------------------------*/

// Distance sensors
distance DistanceBack = distance(PORT19);
distance DistanceFront = distance(PORT18);
distance DistanceSide = distance(PORT17);

// Inertial sensor for heading (shared with chassis on PORT7)
inertial Inertial = inertial(PORT7);

/*----------------------------------------------------------------------------*/
/*                           INITIALIZATION                                   */
/*----------------------------------------------------------------------------*/

void vexcodeInit(void) {
  // Reserved for future initialization
}
