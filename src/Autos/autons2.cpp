// autons.cpp - Autonomous routines

#include "vex.h"
#include "robot-config.h"
#include "Autos/autonUtil.h"

void Highside(){
  chassis.set_coordinates(0,0,0);
  IntakeMotors.spin(fwd,100,pct);
  delayedCall(toggleWings,3000);
  drive_to_wall(850);
}

void Lowside(){

}

void Skills(){

}

void Solo(){

}