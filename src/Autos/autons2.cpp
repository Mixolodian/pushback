// autons.cpp - Autonomous routines

#include "vex.h"
#include "robot-config.h"
#include "Autos/autonUtil.h"

// Stores odometry position when wall-aligning to correct coordinate drift
double Xcords = 0;
double Ycords = 0;

void Highside(){
  chassis.set_coordinates(0,0,0);
  IntakeMotors.spin(fwd,100,pct);
  delayedCall(toggleWings,3000); // deploy wings after 3s
  drive_to_wall(850);
}

void Lowside(){
  // Start positioned at left goal (-42,75)
  chassis.set_coordinates(-42,75,Inertial.angle(degrees));


  chassis.drive_distance(3);
  chassis.set_drive_exit_conditions(3,200,1200);
  chassis.turn_to_point(-18, 101);
  chassis.drive_to_point(-18, 101);
  chassis.turn_to_angle(80); // swing to face wall
  //chassis.turn_to_angle(80);
  chassis.drive_distance(2);
  default_constants();
  // Capture corrected position after wall align
  Xcords = chassis.get_X_position();
  Ycords = chassis.get_Y_position();

  // Drive into far wall to re-zero odometry
  chassis.turn_to_angle(90);
  IntakeMotors.spin(fwd,100,pct);
  delayedCall(toggleWings,1800); // wings out to push far ball
  drive_to_wall(900, 6, 1, 4, 4000);

//FIN 37,112
  // Reset coordinates using corrected Y from wall align
  chassis.set_coordinates(36,Ycords,Inertial.heading(degrees));
  chassis.drive_distance(-5);
  Matchloader.set(false);
  chassis.turn_to_point(26,81);
  chassis.drive_to_point(26,81);

// RED 27,83
  // Approach red goal and score
  IntakeMotors.spin(fwd,30,pct);
  chassis.set_drive_exit_conditions(1,200,1500);
  chassis.drive_to_point(21.5,57);
  printf("\rX:%.2f Y:%.2f H:%.2f | DF:%.0f      ",
           chassis.get_X_position(),
           chassis.get_Y_position(),
           chassis.get_absolute_heading(),
           DistanceFront.objectDistance(mm));
  chassis.set_turn_exit_conditions(1,200,1000);
  chassis.turn_to_angle(46);
  default_constants();
  IntakeMotors.spin(fwd,60,pct);
  delayedCall(toggleMid,250); // deploy mid wings
  chassis.drive_distance(-2.5);
  wait(1,sec);
  IntakeMotors.spin(fwd,90,pct);
  wait(0.8,sec);
  Matchloader.set(true);
  chassis.drive_distance(-0.5);

//MG 21,61
  // Return to mid goal area
  chassis.drive_to_point(25,80);
  Wings.set(false);

}

void Skills(){

  // --- INIT ---
  chassis.set_coordinates(0,0,0);
  IntakeMotors.spin(fwd,100,pct);
  Descore.set(true);

  // --- LEFT MID GOAL ---
  chassis.set_drive_constants(6, 3.2, 0, 20, 0); // slower constants for precision
  IntakeMotors.spin(fwd,100,pct);
  chassis.drive_distance(3);
  chassis.turn_to_point(-10,15);
  chassis.drive_to_point(-10,15);
  chassis.turn_to_point(-9.5,28);

  // Tight exit conditions to stop precisely at goal
  chassis.set_drive_exit_conditions(0.5,200,1000);
  chassis.set_turn_exit_conditions(1,150,1000);
  chassis.drive_to_point(-10,30);
  IntakeMotors.spin(fwd,0,pct);

  // --- PUSH BALL INTO LEFT MID GOAL ---
  chassis.set_turn_exit_conditions(1,200,1000);
  chassis.turn_to_angle(230);
  Wings.set(true);
  default_constants();
  IntakeMotors.spin(fwd,60,pct);

  chassis.drive_distance(-1.8);
  Wings.set(true);
  wait(0.5,sec);
  IntakeMotors.spin(fwd,100,pct);
  wait(0.6,sec);
  chassis.drive_distance(-0.5);
  default_constants();

  // --- ROUTE TO MATCHLOADER (bottom left) ---
  chassis.drive_to_point(-39,0);
  Wings.set(false);

  // Align to matchloader wall and load first ball
  Matchloader.set(true);
  chassis.turn_to_point(-40,-12);
  drive_to_wall(270);
  IntakeMotors.spin(fwd,100,pct);
  chassis.drive_distance(2,180,9,1,5,50,200); // short forward push into loader
  wait(1.3,sec);

  // --- ROUTE TO LEFT GOAL ---
  chassis.drive_to_point(-54,10);
  Matchloader.set(false);
  IntakeMotors.spin(fwd,0,pct);

  chassis.drive_to_point(-54,82);
  chassis.turn_to_angle(270);
  drive_to_wall(450); // align to left wall
  chassis.turn_to_angle(0);
  chassis.drive_distance(-7,0,8,0,1,100,200); // back into goal
  chassis.set_coordinates(-42,75,Inertial.angle(degrees)); // re-zero at left goal

  // --- SCORE BALL 1 AT LEFT GOAL ---
  Hood.set(true);
  IntakeMotors.spin(fwd,100,pct);
  wait(1.8,sec);
  chassis.drive_distance(3,0,8,8,2,1,100); // drive out of goal

  // Reload from matchloader
  chassis.turn_to_angle(4);
  Matchloader.set(true);
  drive_to_wall(270);
  Hood.set(false);
  chassis.drive_distance(1.8,0,9,1,5,50,100); // back into loader
  wait(1.2,sec);

  // --- SCORE BALL 2 AT LEFT GOAL ---
  chassis.drive_to_point(-42,80);
  chassis.turn_to_angle(0);
  chassis.drive_distance(-6,0,8,0,1,100,200); // back into goal

  Hood.set(true);
  wait(2,sec);
  chassis.drive_distance(4); // exit goal
  Hood.set(false);
  IntakeMotors.spin(fwd,0,pct);
  Matchloader.set(false);

//INSERT MID GOAL HERE  <---

  // --- NAVIGATE TO FAR WALL FOR COORDINATE CORRECTION ---
  chassis.drive_distance(3);
  chassis.set_drive_exit_conditions(3,200,1200);
  chassis.turn_to_point(-17, 101);
  chassis.drive_to_point(-17, 101);
  chassis.turn_to_angle(80); // swing to face wall
  //chassis.turn_to_angle(80);
  chassis.drive_distance(3);
  default_constants();
  // Capture corrected position after wall align
  Xcords = chassis.get_X_position();
  Ycords = chassis.get_Y_position();

  // Drive into far wall to re-zero odometry
  chassis.turn_to_angle(90);
  snapCorrectY();
  IntakeMotors.spin(fwd,100,pct);
  delayedCall(toggleWings,1800); 
  // wings out to push far ball
  
  drive_to_wall(900, 6, 1, 4, 4000);
  snapCorrectY();
//FIN 37,112
  // Reset coordinates using corrected Y from wall align
  chassis.set_coordinates(36,Ycords,Inertial.heading(degrees));
  chassis.drive_distance(-5);
  chassis.turn_to_angle(180);
  //chassis.turn_to_point(24,62.5);
  //chassis.set_drive_exit_conditions(1,200,1500);
  //chassis.drive_to_point(24,61.5);
  
  /*printf("\rX:%.2f Y:%.2f H:%.2f | DF:%.0f      ",
           chassis.get_X_position(),
           chassis.get_Y_position(),
           chassis.get_absolute_heading(),
           DistanceFront.objectDistance(mm));
  chassis.set_turn_exit_conditions(1,200,1000);
  chassis.turn_to_angle(46);
  default_constants();
  IntakeMotors.spin(fwd,50,pct);
  delayedCall(toggleMid,140);
  chassis.drive_distance(-4.6);
 
  Wings.set(true);
  wait(1,sec);
  IntakeMotors.spin(fwd,70,pct);
  wait(1,sec);
  Matchloader.set(true);
  chassis.drive_distance(-1);*/

//MG 21,61
  // Return to mid goal area
  chassis.drive_distance(13);
  Wings.set(false);
  // --- ROUTE TO RIGHT SIDE ---
  default_constants();
  chassis.turn_to_angle(90);
  drive_to_wall(450);
  chassis.turn_to_angle(0);
  chassis.drive_distance(-8,0,8,0,1,100,300); // back into right goal
  Hood.set(true);
  wait(1.3,sec);
  // Align to right matchloader wall and load ball
  chassis.turn_to_angle(0);
  Hood.set(false);
  drive_to_wall(270);
  IntakeMotors.spin(fwd,100,pct);
  chassis.drive_distance(1.8,0,9,1,5,50,100); // push into loader
  wait(1.2,sec);
  chassis.drive_to_point(65,75);
  Matchloader.set(false);

  // --- ROUTE TO RIGHT GOAL ---
  chassis.drive_to_point(65,5);
  chassis.turn_to_angle(90);
  drive_to_wall(440); // align to right wall
  chassis.turn_to_angle(180);
  chassis.drive_distance(-8,0,8,0,1,100,300); // back into right goal
  Hood.set(true);

  // --- SCORE BALL 1 AT RIGHT GOAL ---
  chassis.set_coordinates(0,0,Inertial.angle(degrees)); // re-zero at right goal
  wait(1.8,sec);
  Matchloader.set(true);
  chassis.drive_distance(3,180,8,8,2,1,100); // exit goal

  // Reload from right matchloader
  chassis.turn_to_angle(180);
  drive_to_wall(250);
  Hood.set(false);
  chassis.drive_distance(1.8,180,9,1,5,50,100); // back into loader
  wait(1.2,sec);

  // --- SCORE BALL 2 AT RIGHT GOAL ---
  chassis.set_drive_exit_conditions(5,50,1000);
  chassis.drive_to_point(0,10);
  chassis.drive_distance(-6,0,8,0,1,100,200); // back into goal
  Hood.set(true);
  wait(1.8,sec);
  Matchloader.set(false);

  // --- END: PUSH BALL INTO RIGHT MID GOAL ---
  chassis.set_drive_exit_conditions(4,100,1000);
  chassis.set_turn_exit_conditions(3,100,1000);

  chassis.set_coordinates(0, 0, 0);
  chassis.drive_distance(5);
  chassis.turn_to_point(31, 30);
  chassis.drive_to_point(31, 30);
  chassis.turn_to_angle(90);
  chassis.drive_distance(8);
  chassis.drive_stop(brake);
  
}

void Solo(){
  default_constants();
  chassis.set_coordinates(0, 0, 270);
  chassis.set_turn_exit_conditions(3,100,1000);

  // --- LOAD FIRST BALL ---
  IntakeMotors.spin(fwd,100,pct);
  Matchloader.set(true);
  chassis.drive_to_point(28,0);
  chassis.turn_to_angle(180);
  drive_to_wall(280,8,1,5,500);
  chassis.drive_distance(2,180,9,0,1,100,200); // push into loader
  wait(0.3,sec);

  // --- SCORE AT GOAL ---
  chassis.set_drive_exit_conditions(3,100,400);
  chassis.drive_to_point(32,15);
  chassis.drive_distance(-5,180,8,0,1,100,200); // back into goal
  Hood.set(true);
  wait(1,sec);
  Matchloader.set(false);
  Hood.set(false);

  // --- PUSH MID BALL ---
  Controller1.rumble("---"); // notify driver
  chassis.set_drive_exit_conditions(1,100,300);
  chassis.drive_distance(4);
  default_constants();
  chassis.set_turn_exit_conditions(3,100,1000);

  chassis.turn_to_point(10, 28);
  delayedCall(toggleWings, 600); // wings open during approach
  chassis.drive_to_point(10, 28);

  chassis.turn_to_point(-32,28);
  Matchloader.set(false);
  delayedCall(toggleWings,600);
  chassis.drive_to_point(-32,28);
  chassis.drive_to_point(-62,12);
  chassis.turn_to_angle(180);

  // --- SCORE SECOND BALL AT LEFT GOAL ---
  chassis.drive_distance(-8,180,10,1,1,100,400); // back into left goal
  Hood.set(true);
  Matchloader.set(true);
  chassis.turn_to_angle(179);
  wait(1.2,sec);
  chassis.set_turn_exit_conditions(3,200,900);
  
  chassis.drive_distance(4,180,10,0,1,100,200); // exit goal
  drive_to_wall(275,8,1,5,500);
  Hood.set(false);
  chassis.drive_distance(2.3,180,9,0,1,100,150); // push into loader
  wait(0.3,sec);
  // --- PUSH LEFT MID GOAL BALL ---
  delayedCall(toggleMid, 1600); // mid wings deploy during drive
  chassis.set_drive_exit_conditions(5,100,900);
  chassis.drive_to_point(-30,37);
  chassis.set_drive_exit_conditions(3,100,200);
  chassis.drive_to_point(-27,39);
  chassis.drive_distance(-4);
  chassis.drive_stop(brakeType::coast);
}
