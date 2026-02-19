#include "vex.h"
#include "robot-config.h"
#include "Autos/autonUtil.h"

// Subsystem Globals
const int AUTO = 1;
const int OPCONTROL = 2;
int GamePhase = AUTO;

// Initialization
void subsystems::init() {
  Controller1.ButtonRight.pressed(toggleWings);
  Controller1.ButtonY.pressed(toggleDescore);
  Controller1.ButtonB.pressed(toggleMidDescore);
}
