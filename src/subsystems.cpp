#include "vex.h"
#include "robot-config.h"
#include "Autos/autons.h"

/*============================================================================*/
/*                                                                            */
/*                           SUBSYSTEM GLOBALS                                */
/*                                                                            */
/*============================================================================*/

const int AUTO = 1;
const int OPCONTROL = 2;

int GamePhase = AUTO; //Sets the default phase to autonomous
//defining what the subsystems do under different conditions
extern controller Controller1;
motor_group DriveLeft = motor_group(LeftMotor1,LeftMotor2,LeftMotor3);
motor_group DriveRight = motor_group(RightMotor1,RightMotor2,RightMotor3);

extern digital_out Hood;
extern digital_out Wings;
extern digital_out Descore;
extern digital_out Matchloader;

extern motor Intake;
extern motor BackRoller;
extern motor TopRoller;

// -------- Pneumatic States --------
bool hoodState = false;
bool wingState = false;
bool descoreState = false;

// ========== PNEUMATICS ==========

void subsystems::toggleWings() {
  wingState = !wingState;
  Matchloader.set(wingState);
}


void subsystems::toggleDescore() {
  descoreState = !descoreState;
  Descore.set(descoreState);
}


// ========== INITIALIZATION ==========

void subsystems::init() {
  // Attach button events ONCE
  Controller1.ButtonRight.pressed(toggleWings);
  Controller1.ButtonY.pressed(toggleDescore);
}
