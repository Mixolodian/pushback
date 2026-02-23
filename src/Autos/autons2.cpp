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

  chassis.set_coordinates(0,0,0);
  default_constants();
  Descore.set(true);

  chassis.set_drive_constants(6, 3.2, 0, 20, 0);
  IntakeMotors.spin(fwd,100,pct);
  chassis.drive_distance(3);
  chassis.turn_to_point(-10,15);
  chassis.drive_to_point(-10,15);
  chassis.turn_to_point(-9.5,28);
  IntakeMotors.spin(fwd,0,pct);


  chassis.drive_to_point(-10,28.5);
  chassis.turn_to_angle(225);
  IntakeMotors.spin(fwd,70,pct);
  //delayedCall(toggleWings,200);

  delayedCall(toggleMid,180);
  chassis.drive_to_point(-4,35);
  wait(1,sec);
  chassis.drive_distance(-0.5);
  default_constants();

  chassis.drive_to_point(-39,0);
  Wings.set(false);

  Matchloader.set(true);
  chassis.turn_to_point(-40,-12);
  drive_to_wall(270);
  IntakeMotors.spin(fwd,100,pct);
  chassis.drive_distance(2);
  wait(1,sec); 


  chassis.drive_to_point(-54,10);
  Matchloader.set(false);
  IntakeMotors.spin(fwd,0,pct);

  chassis.drive_to_point(-54,82);
  chassis.turn_to_angle(270);
  drive_to_wall(450);
  chassis.turn_to_angle(0);
  chassis.drive_distance(-10,0,8,0,1,100,500);
  chassis.set_coordinates(-42,75,Inertial.angle(degrees));
  Hood.set(true);
  IntakeMotors.spin(fwd,100,pct);
  wait(2,sec);

  chassis.turn_to_angle(3);
  Matchloader.set(true);
  drive_to_wall(270);
  Hood.set(false);
  chassis.drive_distance(2);
  wait(1.2,sec);
  chassis.drive_to_point(-42,80);
  chassis.turn_to_angle(0);
  chassis.drive_distance(-10,0,8,0,1,100,500);


  Hood.set(true);
  wait(2.5,sec);
  chassis.drive_distance(4);
  Hood.set(false);
  IntakeMotors.spin(fwd,0,pct);

//INSERT MID GOAL HERE  <---
  chassis.set_drive_exit_conditions(1.5, 250, 6000);
  chassis.turn_to_point(52,80);
  chassis.drive_to_point(52,80);
  default_constants();

  chassis.turn_to_angle(0);
  drive_to_wall(270);
  IntakeMotors.spin(fwd,100,pct);
  chassis.drive_distance(2);
  wait(1.2,sec);
  chassis.drive_to_point(65,75);
  Matchloader.set(false);
  chassis.drive_to_point(65,0);

  chassis.turn_to_angle(90);
  drive_to_wall(450);
  chassis.turn_to_angle(180);
  chassis.drive_distance(-8,0,8,0,1,100,300);
  Hood.set(true);

  chassis.set_coordinates(0,0,Inertial.angle(degrees));
  wait(2.5,sec);
  Matchloader.set(true);
  chassis.turn_to_angle(183);
  drive_to_wall(270);
  Hood.set(false);
  chassis.drive_distance(2);
  wait(1.2,sec);
  chassis.drive_to_point(0,10);
  chassis.drive_distance(-8,0,8,0,1,100,300);
  Hood.set(true);
  wait(2.5,sec);

  chassis.drive_distance(5);
  chassis.turn_to_point(31, -30);
  chassis.drive_to_point(31, -30);
  chassis.turn_to_angle(270);
  chassis.drive_distance(8);







}

void Solo(){

}