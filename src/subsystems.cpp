#include "vex.h"
#include "robot-config.h"

/*============================================================================*/
/*                                                                            */
/*                           SUBSYSTEM GLOBALS                                */
/*                                                                            */
/*============================================================================*/

const int AUTO = 1;
const int OPCONTROL = 2;
int GamePhase = AUTO;

// Pneumatic states
bool wingState = false;
bool descoreState = false;

/*============================================================================*/
/*                                                                            */
/*                             PNEUMATICS                                     */
/*                                                                            */
/*============================================================================*/

void subsystems::toggleWings() {
  wingState = !wingState;
  Matchloader.set(wingState);
}

void subsystems::toggleDescore() {
  descoreState = !descoreState;
  Descore.set(descoreState);
}

/*============================================================================*/
/*                                                                            */
/*                            INITIALIZATION                                  */
/*                                                                            */
/*============================================================================*/

void subsystems::init() {
  Controller1.ButtonRight.pressed(toggleWings);
  Controller1.ButtonY.pressed(toggleDescore);
}
