// autons.cpp - Autonomous routines

#include "vex.h"
#include "robot-config.h"
#include "Autos/autonUtil.h"

// Toggle Functions
bool MidGoaling = false;
bool longGoaling = false;

int GoalWrapper() {
  while (true) {
    if (MidGoaling && DistanceBack.objectDistance(mm) < 250) {
      Wings.set(true);
      Hood.set(true);
    }
    if (longGoaling && DistanceSide.objectDistance(mm) < 250) {
      Hood.set(true);
    }
    vex::this_thread::sleep_for(20);
  }
  return 0;
}

void MatchToggle() {
  Matchloader.set(true);
}

void MidGoalToggle() {
  Wings.set(true);
}

void HoodToggle() {
  Hood.set(true);
}

// Test Autonomous Routines
void high_side_auto() {
  default_constants();
  IntakeMotors.setMaxTorque(100, pct);
  chassis.set_coordinates(0, 0, 270);

  IntakeMotors.spin(forward, 100, pct);
  chassis.set_drive_constants(5.2, 3.5, 0, 17, 0);
  delayedCall(MatchToggle, 650);
  chassis.drive_to_point(-31, -5);

  default_constants();
  chassis.turn_to_angle(135);
  chassis.set_drive_constants(6, 3.5, 0, 17, 0);

  delayedCall(MidGoalToggle, 280);
  chassis.drive_distance(-5.3);
  default_constants();
  IntakeMotors.spin(fwd, 100, pct);
  wait(0.8, sec);

  chassis.turn_to_point(-15, -30);
  chassis.drive_to_point(-15, -30);
  Matchloader.set(true);
  Wings.set(false);
  chassis.turn_to_angle(90);
  chassis.set_drive_constants(5.2, 3.5, 0, 17, 0);
  chassis.set_drive_exit_conditions(1.5, 250, 1200);

  chassis.drive_distance(14);
  chassis.set_drive_constants(9, 3.5, 0, 17, 0);

  Hood.set(false);
  chassis.drive_to_point(-30, -31.5);
  Hood.set(true);
  wait(1.4, sec);
  Matchloader.set(false);

  chassis.drive_distance(5);
  chassis.turn_to_angle(45);
  chassis.drive_distance(-5);
  chassis.turn_to_angle(90);
  chassis.drive_distance(-7);
  chassis.turn_to_angle(90);
}

void turn_test() {
  default_constants();
  IntakeMotors.setMaxTorque(100, pct);
  chassis.set_coordinates(0, 0, 0);

  chassis.drive_distance(3);
  Matchloader.set(true);
  chassis.drive_to_point(0, -37);
  IntakeMotors.spin(fwd, 100, pct);
  chassis.set_drive_constants(7, 3.5, 0, 19, 0);
  chassis.set_drive_exit_conditions(1.5, 100, 400);
  chassis.turn_to_angle(270);
  drive_to_wall(220, 7, 1, 4, 900);
  chassis.drive_distance(3);
  default_constants();

  chassis.drive_to_point(20, -40);
  Hood.set(true);
  Matchloader.set(false);
  wait(1.2, sec);

  chassis.drive_distance(5);
  chassis.turn_to_point(24, -18);
  Hood.set(false);
  delayedCall(MatchToggle, 500);
  chassis.drive_to_point(25, -18);
  chassis.turn_to_point(25, 28);
  Matchloader.set(false);

  delayedCall(MatchToggle, 800);
  chassis.set_drive_exit_conditions(0.5, 300, 3400);
  chassis.drive_to_point(26, 28.5);
  default_constants();
  chassis.turn_to_angle(310);
  Matchloader.set(false);
  delayedCall(MidGoalToggle, 260);
  chassis.drive_distance(-6);

  wait(0.3, sec);
  IntakeMotors.spin(fwd, -50, pct);
  Wings.set(false);
  IntakeMotors.spin(fwd, 100, pct);
  chassis.drive_to_point(6, 53);
  chassis.turn_to_angle(270);

  drive_to_wall(925, 12, 1, 4, 400);
  Hood.set(true);
}

void odom_test() {
  chassis.set_coordinates(0, 0, 0);
  Intake.spin(fwd, 100, volt);
  wait(5, sec);
  chassis.drive_distance(5);
}

void low_side_auto() {
  // Phase 1: Initialization
  default_constants();
  IntakeMotors.setMaxTorque(100, pct);
  chassis.set_coordinates(0, 0, 270);

  // Phase 2: First ball collection
  IntakeMotors.spin(forward, 100, pct);
  chassis.set_drive_constants(5.2, 3.5, 0, 17, 0);
  delayedCall(MatchToggle, 650);
  chassis.drive_to_point(-35, 5);

  // Phase 3: First scoring attempt
  default_constants();
  Matchloader.set(false);
  chassis.turn_to_angle(215);
  chassis.set_drive_constants(5.2, 3.5, 0, 17, 0);

  chassis.drive_distance(4);
  default_constants();
  IntakeMotors.spin(fwd, -70, pct);
  wait(1, sec);
  IntakeMotors.spin(fwd, 100, pct);
  chassis.drive_distance(-6);

  // Phase 4: Second triball collection
  chassis.turn_to_point(-15, 28);
  chassis.drive_to_point(-15, 28);
  Matchloader.set(true);
  chassis.turn_to_angle(90);
  chassis.set_drive_constants(5.8, 3.5, 0, 17, 0);

  // Phase 5: Goal approach & score
  chassis.drive_distance(18);
  chassis.set_drive_constants(9, 3.5, 0, 17, 0);
  Hood.set(false);
  chassis.drive_to_point(-30, 31.5);
  Hood.set(true);
  wait(1.4, sec);
  Matchloader.set(false);
  IntakeMotors.spin(fwd, 0, pct);

  // Phase 6: Repositioning
  chassis.drive_distance(5);
  chassis.turn_to_angle(45);
  chassis.drive_distance(-5);
  chassis.turn_to_angle(90);
  chassis.drive_distance(-6.5);
  chassis.turn_to_angle(90);
}

// Competition Autonomous Routines
void SKILLS() {
  chassis.set_coordinates(0, 0, 0);
  IntakeMotors.setMaxTorque(100, percent);
  default_constants();
  task GoalTask(GoalWrapper);

  // Middle goal
  chassis.drive_to_point(0, 22);
  chassis.set_drive_constants(2.5, 3.5, 0, 17, 0);
  chassis.turn_to_point(-14, 22);
  IntakeMotors.spin(forward, 100, pct);
  chassis.drive_to_point(-20, 20);
  Matchloader.set(true);
  chassis.drive_to_point(-8, 32);
  IntakeMotors.spin(fwd, 0, pct);
  chassis.turn_to_angle(225);
  delayedCall(MidGoalToggle, 150);
  chassis.drive_distance(-2);
  Matchloader.set(false);
  IntakeMotors.spin(fwd, 100, pct);
  wait(1, sec);

  // Matchloader 1
  default_constants();
  chassis.turn_to_point(-42, 0);
  chassis.drive_to_point(-42, 0);
  Matchloader.set(true);
  Wings.set(false);
  chassis.set_drive_constants(3, 3.5, 0, 17, 0);
  chassis.turn_to_point(-42, -14);
  chassis.drive_to_point(-42, -14);
  wait(1, sec);
  default_constants();

  // Matchloader 2
  chassis.drive_distance(-5);
  Matchloader.set(false);
  IntakeMotors.spin(fwd, 0, pct);

  chassis.turn_to_point(-53, 10);
  chassis.drive_to_point(-53, 10);
  chassis.turn_to_point(-53, 90);
  chassis.drive_to_point(-53, 90);
  chassis.turn_to_angle(270);
  drive_to_wall(430, 7, 1, 1, 2000);

  // Scoring
  longGoaling = true;
  chassis.turn_to_angle(0);
  drive_to_wall(925, 7, 1, 1, 2000);
  chassis.set_coordinates(-40, 76, 0);
  Hood.set(true);
  IntakeMotors.spin(fwd, 100, pct);
  Controller1.rumble("---");
  wait(2, sec);
  chassis.turn_to_angle(0);
  Hood.set(false);
  Matchloader.set(true);
  chassis.set_drive_constants(3, 3.5, 0, 17, 0);
  chassis.drive_to_point(-40, 105);
  wait(1.4, sec);
  default_constants();

  // Scoring
  chassis.drive_to_point(-40, 76);
  Hood.set(true);
  wait(2, sec);
  Controller1.rumble("---");
  Matchloader.set(false);
  IntakeMotors.spin(fwd, 0, pct);
  Hood.set(false);

  // Third matchloader
  chassis.drive_distance(2);
  chassis.turn_to_point(54, 84);
  chassis.drive_to_point(54, 84);
  Matchloader.set(true);
  IntakeMotors.spin(fwd, 100, pct);
  chassis.set_drive_constants(4, 3.5, 0, 17, 0);
  chassis.turn_to_point(53, 106);
  chassis.drive_to_point(53, 106);
  wait(1.4, sec);
  default_constants();
  Matchloader.set(false);
  IntakeMotors.spin(fwd, 0, pct);

  // Score
  chassis.drive_distance(-5);
  chassis.turn_to_point(67, 85);
  chassis.drive_to_point(67, 85);
  chassis.turn_to_point(67, 0);
  chassis.drive_to_point(67, 0);
  chassis.turn_to_angle(90);
  drive_to_wall(430, 7, 1, 1, 2000);
  chassis.turn_to_angle(180);
  longGoaling = true;
  drive_to_wall(923, 7, 1, 1, 600);
  IntakeMotors.spin(fwd, 100, pct);
  wait(2, sec);
  Hood.set(false);
  longGoaling = false;

  // Matchloader 4
  chassis.set_coordinates(0, 0, 0);
  Matchloader.set(true);
  chassis.set_drive_constants(3, 3.5, 0, 17, 0);
  chassis.drive_to_point(1.5, 28.5);
  wait(1.4, sec);
  default_constants();

  // Scoring
  chassis.drive_to_point(0, 0);
  Hood.set(true);
  wait(2, sec);
  Matchloader.set(false);
  Hood.set(false);

  // Parking
  chassis.drive_distance(5);
  chassis.turn_to_point(31, 30);
  chassis.drive_to_point(31, 30);
  chassis.turn_to_angle(90);
  chassis.drive_distance(8);
}

void full_test() {
  task GoalTask(GoalWrapper);
  chassis.set_coordinates(0, 0, 0);
  longGoaling = true;
  drive_to_wall(445, 8, 1, 4, 1000);
  chassis.turn_to_angle(90);
  drive_to_wall(925, 8, 1, 4, 1000);
}
