/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       autons.cpp                                                */
/*    Description:  Autonomous routines                                       */
/*                                                                            */
/*    CONTENTS:                                                               */
/*    1. Toggle Functions                                                     */
/*    2. Test Autonomous Routines                                             */
/*    3. Competition Autonomous Routines                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "robot-config.h"
#include "subsystems.h"
#include "Autos/autonUtil.h"

/*============================================================================*/
/*                                                                            */
/*                      SECTION 1: TOGGLE FUNCTIONS                           */
/*                                                                            */
/*============================================================================*/

bool MidGoaling = false;
bool longGoaling = false;
bool Pstatus = false;

int GoalWrapper() {
  while(true){
    if(MidGoaling && DistanceBack.objectDistance(mm) < 250){
      Wings.set(true);
      Hood.set(true);
    }
    if(longGoaling && DistanceSide.objectDistance(mm) < 30){
      Hood.set(true);
    }
    else{
    }
    vex::this_thread::sleep_for(20);
  }
  return 0;
}

void MatchToggle(){
  Matchloader.set(true);
}

void MidGoalToggle(){
  Wings.set(true);
}

void HoodToggle(){
  Hood.set(true);
}

/**
 * @brief Debug thread that prints robot pose to terminal
 * @return 0 (never returns in normal operation)
 *
 * Usage: vex::task poseTask(printPoseThread);
 */



/*============================================================================*/
/*                                                                            */
/*                   SECTION 2: TEST AUTONOMOUS ROUTINES                      */
/*                                                                            */
/*============================================================================*/

/**
 * @brief High side autonomous routine
 */
void high_side_auto() {
  default_constants();
  IntakeMotors.setMaxTorque(100,pct);

  chassis.set_coordinates(0, 0, 270);  ;

  IntakeMotors.spin(forward, 100, pct);
  chassis.set_drive_constants(5.2, 3.5, 0, 17, 0);    // Forward/backward
  delayedCall(MatchToggle,650);
  chassis.drive_to_point(-32,-5);

  default_constants();

  chassis.turn_to_angle(135);
  Matchloader.set(false);
    chassis.set_drive_constants(5.2, 3.5, 0, 17, 0);    // Forward/backward

delayedCall(MidGoalToggle,280);
    chassis.drive_distance(-5);
  default_constants();
  IntakeMotors.spin(fwd,50,pct);
  wait(0.67,sec);
  Wings.set(false);
  IntakeMotors.spin(fwd,100,pct);
  chassis.turn_to_point(-15,-30);
  chassis.drive_to_point(-15,-30);
  Matchloader.set(true);
  chassis.turn_to_angle(90);
      chassis.set_drive_constants(5.2, 3.5, 0, 17, 0);    // Forward/backward

  chassis.drive_distance(10);
      chassis.set_drive_constants(9, 3.5, 0, 17, 0);    // Forward/backward

  Hood.set(false);
  chassis.drive_to_point(-30,-31.5);
  Hood.set(true);
  wait(1.4,sec);
  Matchloader.set(false);

  //chassis.turn_to_angle(90);
  /*chassis.drive_distance(25);
  Hood.set(false);
  wait(0.2,sec);
  chassis.drive_to_point(-30,-31.5);
  Hood.set(true);*/

  chassis.drive_distance(5);
  chassis.turn_to_angle(45);
  chassis.drive_distance(-5);
  chassis.turn_to_angle(90);
  chassis.drive_distance(-7);
  chassis.turn_to_angle(90);

}

/**
 * @brief Turn test - placeholder for turn testing
 * Expected: Robot completes a full rotation and returns to start angle
 */
void turn_test() {
  // TODO: Implement turn test
  default_constants();
  IntakeMotors.setMaxTorque(100,pct);
  chassis.set_coordinates(0,0,0);

  IntakeMotors.spin(forward, 100, pct);
  chassis.drive_distance(3);
  chassis.drive_distance(-16.5);
  Matchloader.set(true);
  chassis.turn_to_angle(270);
  chassis.set_drive_constants(4, 3.5, 0, 17, 0);    // Forward/backward
  chassis.drive_distance(4);
  default_constants();
  delayedCall(HoodToggle,1000);
  chassis.drive_distance(-15);
  Matchloader.set(false);
  wait(1,sec);
  chassis.drive_distance(4);
  Hood.set(false);
  chassis.turn_to_point(25,-19);
  delayedCall(MatchToggle,1000);
  chassis.drive_to_point(25,-17);
  Matchloader.set(false);
  chassis.turn_to_point(24,27);
  chassis.drive_to_point(24,27);
  chassis.turn_to_angle(315);
  delayedCall(MidGoalToggle,150);
  chassis.set_drive_constants(3, 3.5, 0, 17, 0);    // Forward/backward
  chassis.drive_to_point(36,16);
  wait(0.4,sec);
  Wings.set(false);
  default_constants();
  chassis.turn_to_point(9,52);
  chassis.drive_to_point(9,52);
  chassis.turn_to_angle(270);
  chassis.drive_distance(-3);
  Hood.set(true);

}

/**
 * @brief Odometry test - displays position data on Brain screen
 *
 * Push the robot around manually to verify odometry tracking accuracy.
 * Displays X, Y, Heading, and tracker positions.
 */
void odom_test() {
  // Initialize odometry
  chassis.set_coordinates(0, 0, 0);
  Intake.spin(fwd,100,volt); // Continuous position display loop
  wait(5,sec);
  chassis.drive_distance(5);
  while (1) {
    // Display on Brain screen

    // Print to terminal
    printf("\rX: %8.2f   Y: %8.2f   Heading: %8.2f",
           chassis.get_X_position(),
           chassis.get_Y_position(),
           chassis.get_absolute_heading());
    fflush(stdout);

    task::sleep(50);
  }
}

/**
 * @brief Low side autonomous routine
 */
void low_side_auto() {
  default_constants();
  IntakeMotors.setMaxTorque(100,pct);

  chassis.set_coordinates(0, 0, 270);

  IntakeMotors.spin(forward, 100, pct);
  chassis.set_drive_constants(5.2, 3.5, 0, 17, 0);    // Forward/backward
  delayedCall(MatchToggle,650);
  chassis.drive_to_point(-35,5);

  default_constants();
  Matchloader.set(false);

  chassis.turn_to_angle(215);
    chassis.set_drive_constants(5.2, 3.5, 0, 17, 0);    // Forward/backward

    chassis.drive_distance(4);
  default_constants();
  IntakeMotors.spin(fwd,-70,pct);
  wait(1,sec);
  IntakeMotors.spin(fwd,100,pct);
  chassis.drive_distance(-6);
  chassis.turn_to_point(-15,28);
  chassis.drive_to_point(-15,28);
  Matchloader.set(true);
  chassis.turn_to_angle(90);
  chassis.set_drive_constants(5.8, 3.5, 0, 17, 0);    // Forward/backward

  chassis.drive_distance(18);
      chassis.set_drive_constants(9, 3.5, 0, 17, 0);    //
  Hood.set(false);
  chassis.drive_to_point(-30,31.5);
  Hood.set(true);
  wait(1.4,sec);
  Matchloader.set(false);
  IntakeMotors.spin(fwd,0,pct);

  //chassis.turn_to_angle(90);
  /*chassis.drive_distance(25);
  Hood.set(false);
  wait(0.2,sec);
  chassis.drive_to_point(-30,-31.5);
  Hood.set(true);*/

  chassis.drive_distance(5);
  chassis.turn_to_angle(45);
  chassis.drive_distance(-5);
  chassis.turn_to_angle(90);
  chassis.drive_distance(-6.5);
  chassis.turn_to_angle(90);

}

/**
 * @brief Holonomic odometry test - placeholder
 * Expected: Square pattern with 360° rotation, ending at start
 */
void holonomic_odom_test() {
  // TODO: Implement holonomic odom test

}

/*============================================================================*/
/*                                                                            */
/*                 SECTION 3: COMPETITION AUTONOMOUS ROUTINES                 */
/*                                                                            */
/*============================================================================*/

/**
 * @brief Skills autonomous routine
 * 1-minute driver skills or programming skills run
 */
void SKILLS() {
  // TODO: Implement skills autonomous
  chassis.set_coordinates(0, 0, 0);
  IntakeMotors.setMaxTorque(100, percent);
  default_constants();
  task GoalTask(GoalWrapper);
  //middle goal
  chassis.drive_to_point(0,22);
  chassis.set_drive_constants(2.5, 3.5, 0, 17, 0);    // Forward/backward
  chassis.turn_to_point(-14,22);
  IntakeMotors.spin(forward, 100, pct);
  chassis.drive_to_point(-20,20);
  Matchloader.set(true);
  chassis.drive_to_point(-8,31);
  IntakeMotors.spin(fwd,0,pct);
  chassis.turn_to_angle(225);
  delayedCall(MidGoalToggle,150);
  chassis.drive_distance(-2);
  Matchloader.set(false);
  IntakeMotors.spin(fwd,100,pct);
  wait(1,sec);

  //MATCH LOADER 1
  default_constants();
  chassis.turn_to_point(-42,0);
  chassis.drive_to_point(-42,0);
  Matchloader.set(true);
  Wings.set(false);
  chassis.set_drive_constants(3, 3.5, 0, 17, 0);    // Forward/backward
  chassis.turn_to_point(-42,-15);
  chassis.drive_to_point(-42,-15);
  wait(1,sec);
  default_constants();

  //MATCHLOADER 2
  chassis.drive_distance(-5);
  Matchloader.set(false);
  IntakeMotors.spin(fwd,0,pct);

  chassis.turn_to_point(-53,10);
  chassis.drive_to_point(-53,10);
  chassis.turn_to_point(-53,90);
  chassis.drive_to_point(-53,90);
  chassis.turn_to_angle(270);
  drive_to_wall(430,7,1,1,2000,false);
  //SCORING
  longGoaling = true;
  chassis.turn_to_angle(0);
  drive_to_wall(925,7,1,1,2000,false);
  IntakeMotors.spin(fwd,100,pct);
  Controller1.rumble("---");
  wait(2,sec);
  chassis.turn_to_angle(0);
  Hood.set(false);
  chassis.set_coordinates(-40,76,0);
  Matchloader.set(true);
  chassis.set_drive_constants(3, 3.5, 0, 17, 0);    // Forward/backward
  chassis.drive_to_point(-40,105);
  wait(1.4,sec);
  default_constants();
  //SCORING
  chassis.drive_to_point(-41,73);
  Hood.set(true);
  wait(2,sec);
  Controller1.rumble("---");
  Matchloader.set(false);
  IntakeMotors.spin(fwd,0,pct);
  Hood.set(false);

  //THIRD MATACHLOADER
  chassis.drive_distance(2);
  chassis.turn_to_point(51,84);
  chassis.drive_to_point(51,84);
  Matchloader.set(true);
  IntakeMotors.spin(fwd,100,pct);
  chassis.set_drive_constants(4, 3.5, 0, 17, 0);    // Forward/backward
  IntakeMotors.spin(forward, 100, pct);
  chassis.turn_to_point(52,106);
  chassis.drive_to_point(52,106);
  wait(1.4,sec);
  default_constants();
  Matchloader.set(false);
  IntakeMotors.spin(fwd,0,pct);
  //SCORE
  chassis.drive_distance(-5);
  chassis.turn_to_point(65,85);
  chassis.drive_to_point(65,85);
  chassis.turn_to_point(65,0);
  chassis.drive_to_point(65,0);
  chassis.turn_to_angle(90);
  //
  //
  //
  //
  drive_to_wall(430,7,1,1,2000,false);
  chassis.turn_to_angle(180);
  longGoaling = true;
  drive_to_wall(923,7,1,1,600,false);
  IntakeMotors.spin(fwd,100,pct);
  wait(2,sec);
  Hood.set(false);
  longGoaling = false;

  //MATCHLOADER 4
  chassis.set_coordinates(0, 0, 0);
  Matchloader.set(true);
  chassis.set_drive_constants(3, 3.5, 0, 17, 0);    // Forward/backward
  chassis.drive_to_point(1.5,28.5);
  wait(1.4,sec);
  default_constants();
//SCORING
  chassis.drive_to_point(0,0);
  Hood.set(true);
  wait(2,sec);
  Matchloader.set(false);
  Hood.set(false);
  //PARKING
  chassis.drive_distance(5);
  chassis.turn_to_point(31,30);
  chassis.drive_to_point(31,30);
  chassis.turn_to_angle(90);
  chassis.drive_distance(8);
}

/**
 * @brief Full match autonomous - complex multi-stage routine
 *
 * SEQUENCE:
 * 1. Initialize subsystems and odometry
 * 2. Navigate to first match loader
 * 3. Collect and score triballs
 * 4. Navigate to long goal
 * 5. Score final triballs
 */
void full_test() {
  /*--------------------------------------------------------------------------*/
  /*                    SUBSYSTEM INITIALIZATION                              */
  /*--------------------------------------------------------------------------*/
  task GoalTask(GoalWrapper);
  longGoaling = true;



  // Autonomous complete
}

/**
 * @brief Right side autonomous - placeholder
 */
void rightside() {
  // TODO: Implement right side autonomous
}

/**
 * @brief Left side autonomous - placeholder
 */
void leftside() {
  // TODO: Implement left side autonomous
}
