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
//mikecoxlong
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
    if(longGoaling && DistanceSide.objectDistance(mm) < -1){
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

  chassis.set_coordinates(0, 0, 270);  

  IntakeMotors.spin(forward, 100, pct);
  chassis.set_drive_constants(5.2, 3.5, 0, 17, 0);    // Forward/backward
  delayedCall(MatchToggle,650);
  chassis.drive_to_point(-31,-5);

  default_constants();

  chassis.turn_to_angle(135);
    chassis.set_drive_constants(6, 3.5, 0, 17, 0);    // Forward/backward

delayedCall(MidGoalToggle,280);
    chassis.drive_distance(-5.3);
  default_constants();
  IntakeMotors.spin(fwd,100,pct);
  wait(0.8,sec);
  IntakeMotors.spin(fwd,100,pct);
  chassis.turn_to_point(-15,-30);
  chassis.drive_to_point(-15,-30);
  Matchloader.set(true);
  Wings.set(false);
  chassis.turn_to_angle(90);
      chassis.set_drive_constants(5.2, 3.5, 0, 17, 0);    // Forward/backward
        chassis.set_drive_exit_conditions(1.5, 250, 1200);

  chassis.drive_distance(14);
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
  
  chassis.drive_distance(3);
  Matchloader.set(true);
  chassis.drive_to_point(0,-37);
  Matchloader.set(true);
  IntakeMotors.spin(fwd,100,pct);
  chassis.set_drive_constants(7, 3.5, 0, 19, 0);    // Forward/backward
  chassis.set_drive_exit_conditions(1.5, 100, 400);
  chassis.turn_to_angle(270);
  drive_to_wall(220,7,1,4,900,false);
  chassis.drive_distance(3);
  default_constants();

  chassis.drive_to_point(20,-40);
  Hood.set(true);
  Matchloader.set(false);
  wait(1.2,sec);

  //chassis.turn_to_angle(20,8,2,200,300);
  chassis.drive_distance(5);
 chassis.turn_to_point(24,-18);
  Hood.set(false);
  delayedCall(MatchToggle,500);
  chassis.drive_to_point(25,-18);
  chassis.turn_to_point(25,28);
  Matchloader.set(false);
  
  delayedCall(MatchToggle,800);
  chassis.set_drive_exit_conditions(0.5, 300, 3400);
  chassis.drive_to_point(26,28.5);
  default_constants();
  chassis.turn_to_angle(310);
  Matchloader.set(false);
  delayedCall(MidGoalToggle,260);
  chassis.drive_distance(-6);

  wait(0.3,sec);
  IntakeMotors.spin(fwd,-50,pct);
  Wings.set(false);
  IntakeMotors.spin(fwd,100,pct);
  chassis.drive_to_point(6,53);
  chassis.turn_to_angle(270);

  drive_to_wall(925,12,1,4,400,false);
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
 *
 * This autonomous starts on the low side of the field (positive Y direction)
 * and executes a sequence to collect and score triballs.
 */
void low_side_auto() {
  /*--------------------------------------------------------------------------*/
  /*                         PHASE 1: INITIALIZATION                          */
  /*--------------------------------------------------------------------------*/
  default_constants();                    // Reset PID constants to default values
  IntakeMotors.setMaxTorque(100,pct);     // Set intake to full torque

  chassis.set_coordinates(0, 0, 270);     // Set starting position: origin, facing left (270°)

  /*--------------------------------------------------------------------------*/
  /*                    PHASE 2: FIRST BALL COLLECTION                     */
  /*--------------------------------------------------------------------------*/
  IntakeMotors.spin(forward, 100, pct);   // Start intake spinning to collect triballs
  chassis.set_drive_constants(5.2, 3.5, 0, 17, 0);  // Set faster drive constants for approach
  delayedCall(MatchToggle,650);           // Schedule matchloader toggle after 650ms
  chassis.drive_to_point(-35,5);          // Drive to first triball location

  /*--------------------------------------------------------------------------*/
  /*                      PHASE 3: FIRST SCORING ATTEMPT                      */
  /*--------------------------------------------------------------------------*/
  default_constants();                    // Reset to default PID constants
  Matchloader.set(false);                 // Retract matchloader

  chassis.turn_to_angle(215);             // Turn to face scoring direction (southwest)
  chassis.set_drive_constants(5.2, 3.5, 0, 17, 0);  // Set drive constants for precision

  chassis.drive_distance(4);              // Short forward drive to position for outtake
  default_constants();                    // Reset constants
  IntakeMotors.spin(fwd,-70,pct);         // Reverse intake to outtake/eject triball
  wait(1,sec);                            // Wait for outtake to complete
  IntakeMotors.spin(fwd,100,pct);         // Resume forward intake
  chassis.drive_distance(-6);             // Back up after scoring

  /*--------------------------------------------------------------------------*/
  /*                    PHASE 4: SECOND TRIBALL COLLECTION                    */
  /*--------------------------------------------------------------------------*/
  chassis.turn_to_point(-15,28);          // Turn to face next triball location
  chassis.drive_to_point(-15,28);         // Drive to collect second triball
  Matchloader.set(true);                  // Deploy matchloader for scoring position
  chassis.turn_to_angle(90);              // Turn to face forward (toward goal)
  chassis.set_drive_constants(5.8, 3.5, 0, 17, 0);  // Slightly faster drive constants

  /*--------------------------------------------------------------------------*/
  /*                       PHASE 5: GOAL APPROACH & SCORE                     */
  /*--------------------------------------------------------------------------*/
  chassis.drive_distance(18);             // Drive forward toward the goal
  chassis.set_drive_constants(9, 3.5, 0, 17, 0);    // High speed for final approach
  Hood.set(false);                        // Lower hood to prepare for scoring
  chassis.drive_to_point(-30,31.5);       // Drive to optimal scoring position at goal
  Hood.set(true);                         // Raise hood to release triballs into goal
  wait(1.4,sec);                          // Wait for triballs to settle/score
  Matchloader.set(false);                 // Retract matchloader
  IntakeMotors.spin(fwd,0,pct);           // Stop intake motors

  /*--------------------------------------------------------------------------*/
  /*                      PHASE 6: REPOSITIONING/CLEANUP                      */
  /*--------------------------------------------------------------------------*/
  // Commented out alternative approach:
  // chassis.turn_to_angle(90);
  // chassis.drive_distance(25);
  // Hood.set(false);
  // wait(0.2,sec);
  // chassis.drive_to_point(-30,-31.5);
  // Hood.set(true);

  chassis.drive_distance(5);              // Short drive forward to clear goal
  chassis.turn_to_angle(45);              // Turn to 45° (northeast direction)
  chassis.drive_distance(-5);             // Back up
  chassis.turn_to_angle(90);              // Turn to face forward (90°)
  chassis.drive_distance(-6.5);           // Final backup to safe position
  chassis.turn_to_angle(90);              // Ensure facing 90° at end of autonomous

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
  chassis.drive_to_point(-8,32);
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
  chassis.turn_to_point(-42,-14);
  chassis.drive_to_point(-42,-14);
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
  //test?
  longGoaling = true;
  chassis.turn_to_angle(0);
  //chassis.drive_to_point(-40,76);
  drive_to_wall(925,7,1,1,2000,false);
  chassis.set_coordinates(-40,76,0);
  Hood.set(true);
  IntakeMotors.spin(fwd,100,pct);
  Controller1.rumble("---");
  wait(2,sec);
  chassis.turn_to_angle(0);
  Hood.set(false);
  Matchloader.set(true);
  chassis.set_drive_constants(3, 3.5, 0, 17, 0);    // Forward/backward
  chassis.drive_to_point(-40,105);
  wait(1.4,sec);
  default_constants();
  //SCORING
  chassis.drive_to_point(-40,76);
  Hood.set(true);
  wait(2,sec);
  Controller1.rumble("---");
  Matchloader.set(false);
  IntakeMotors.spin(fwd,0,pct);
  Hood.set(false);

  //THIRD MATACHLOADER
  chassis.drive_distance(2);
  chassis.turn_to_point(54,84);
  chassis.drive_to_point(54,84);
  Matchloader.set(true);
  IntakeMotors.spin(fwd,100,pct);
  chassis.set_drive_constants(4, 3.5, 0, 17, 0);    // Forward/backward
  IntakeMotors.spin(forward, 100, pct);
  chassis.turn_to_point(53,106);
  chassis.drive_to_point(53,106);
  wait(1.4,sec);
  default_constants();
  Matchloader.set(false);
  IntakeMotors.spin(fwd,0,pct);
  //SCORE
  chassis.drive_distance(-5);
  chassis.turn_to_point(67,85);
  chassis.drive_to_point(67,85);
  chassis.turn_to_point(67,0);
  chassis.drive_to_point(67,0);
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
  chassis.set_coordinates(0,0,0);
  longGoaling = true;
  drive_to_wall(445,8,1,4,1000,false);
  chassis.turn_to_angle(90);
  drive_to_wall(925,8,1,4,1000,false);



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
