#include "vex.h"
#include "robot-config.h"

// Subsystem Globals
const int AUTO = 1;
const int OPCONTROL = 2;
int GamePhase = AUTO;

bool wingState = false;
bool descoreState = false;

// Pneumatics
void subsystems::toggleWings() {
  wingState = !wingState;
  Matchloader.set(wingState);
}

void subsystems::toggleDescore() {
  descoreState = !descoreState;
  Descore.set(descoreState);
}

// Initialization
void subsystems::init() {
  Controller1.ButtonRight.pressed(toggleWings);
  Controller1.ButtonY.pressed(toggleDescore);
}
