#include "main.h"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 50;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

// OUTTAKE
const int OUTTAKE_TIME = 500; // time to run outtake motor in ms
const int OUTTAKE_SPEED = 110; // speed to run outtake motor

// SIZE
const int ROBOT_LENGTH = 18; // robot length in inches

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}
///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}


void matchload_left() {
  chassis.pid_drive_set(32_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(-90_deg, 80);
  matchloader.set(true);
  chassis.pid_wait();
  intake.move(OUTTAKE_SPEED);
  chassis.pid_drive_set(6_in, 127);
  chassis.pid_wait();
  pros::delay(OUTTAKE_TIME * 2);
  intake.move(0);
  matchloader.set(false);
  chassis.pid_drive_set(-24_in, DRIVE_SPEED);
  chassis.pid_wait();
  intake.move(OUTTAKE_SPEED);
  outtake.move(OUTTAKE_SPEED);
  pros::delay(OUTTAKE_TIME * 4);
  intake.move(0);
  outtake.move(0);

}
void matchload_right() {
  chassis.pid_drive_set(32_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, 80);
  matchloader.set(true);
  chassis.pid_wait();
  intake.move(OUTTAKE_SPEED);
  chassis.pid_drive_set(6_in, 127);
  chassis.pid_wait();
  outtake.move(OUTTAKE_SPEED);
  pros::delay(OUTTAKE_TIME * 2);
  intake.move(0);
  outtake.move(0);
  matchloader.set(false);
  chassis.pid_drive_set(-24_in, DRIVE_SPEED);
  chassis.pid_wait();
  intake.move(OUTTAKE_SPEED);
  outtake.move(OUTTAKE_SPEED);
  pros::delay(OUTTAKE_TIME * 4);
  intake.move(0);
  outtake.move(0);

}

void kamakaze_right() {
  chassis.pid_drive_set(28_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(-91_deg, TURN_SPEED);
  chassis.pid_wait();
  intake.move(OUTTAKE_SPEED);
  chassis.pid_drive_set(12.5_in, DRIVE_SPEED);
  chassis.pid_wait();
  intake.move(0);

  chassis.pid_drive_set(10.5_in, DRIVE_SPEED + 10);
  chassis.pid_wait();
  intake.move(-OUTTAKE_SPEED);
  pros::delay(OUTTAKE_TIME * 3); // 1nce is 500ms
  intake.move(0);
}

void skillsAuton(){
  matchload_right();  
  //escape the starting matchload
  chassis.pid_drive_set(15_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(225_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(34_in, DRIVE_SPEED);
  /// outtake kamikaze at center goals
  pros::delay(OUTTAKE_TIME * 2);
  intake.move(OUTTAKE_SPEED);
  chassis.pid_wait();
  intake.move(0);

  chassis.pid_drive_set(12_in, DRIVE_SPEED);
  chassis.pid_wait();
  intake.move(-OUTTAKE_SPEED);
  pros::delay(OUTTAKE_TIME * 3); 
  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  intake.move(0);
  chassis.pid_wait();
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(46_in, DRIVE_SPEED);
  pros::delay(200); 
  intake.move(OUTTAKE_SPEED);
  chassis.pid_wait();
  intake.move(0);
  chassis.pid_wait();
  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-17_in, DRIVE_SPEED);
  chassis.pid_wait();
  middlegoal.set(false);

  intake.move(OUTTAKE_SPEED);
  outtake.move(OUTTAKE_SPEED);
  pros::delay(OUTTAKE_TIME * 4);
  intake.move(0);
  outtake.move(0);
  middlegoal.set(true);
// Cross over field to side opposite starting point
  chassis.pid_drive_set(53_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();
  matchloader.set(true);
  chassis.pid_drive_set(10_in, DRIVE_SPEED);
  intake.move(OUTTAKE_SPEED);
  outtake.move(OUTTAKE_SPEED);
  pros::delay(100);
  outtake.move(0);
  pros::delay(OUTTAKE_TIME * 2);  
  chassis.pid_wait();
  intake.move(0);
  outtake.move(0);
  matchloader.set(false);
  chassis.pid_drive_set(-26_in, DRIVE_SPEED);
  chassis.pid_wait();
  intake.move(OUTTAKE_SPEED);
  outtake.move(OUTTAKE_SPEED);
  pros::delay(OUTTAKE_TIME * 4);
  intake.move(0);
  outtake.move(0);
  chassis.pid_drive_set(30_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(65_in, 127);
  intake.move(OUTTAKE_SPEED);
  outtake.move(OUTTAKE_SPEED);
  chassis.pid_wait();
  intake.move(0);
  outtake.move(0);  
  // chassis.pid_turn_set(-30_deg, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(34_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // chassis.pid_turn_set(-60_deg, TURN_SPEED);
  // chassis.pid_wait();
  
}
void autonSkillsV2() {
  // 1 Forward
  chassis.pid_drive_set(10_in, DRIVE_SPEED); chassis.pid_wait();
  // 2 Left 90
  chassis.pid_turn_set(-90_deg, TURN_SPEED); chassis.pid_wait();
  // 3 Forward
  chassis.pid_drive_set(10_in, DRIVE_SPEED); chassis.pid_wait();
  // 4 Left 90
  chassis.pid_turn_set(-90_deg, TURN_SPEED); chassis.pid_wait();
  // 5 Forward
  chassis.pid_drive_set(10_in, DRIVE_SPEED); chassis.pid_wait();
  // 6 Intake
  intake.move(OUTTAKE_SPEED);
  // 7 Backward
  chassis.pid_drive_set(-10_in, DRIVE_SPEED); chassis.pid_wait();
  // 8 Outtake
  outtake.move(OUTTAKE_SPEED);
  // 9 Forward
  chassis.pid_drive_set(10_in, DRIVE_SPEED); chassis.pid_wait();
  // 10 Right 90
  chassis.pid_turn_set(90_deg, TURN_SPEED); chassis.pid_wait();
  // 11 Forward
  chassis.pid_drive_set(10_in, DRIVE_SPEED); chassis.pid_wait();
  // 12 Right 90
  chassis.pid_turn_set(90_deg, TURN_SPEED); chassis.pid_wait();
  // 13 Forward
  chassis.pid_drive_set(10_in, DRIVE_SPEED); chassis.pid_wait();
  // 14 Right 90
  chassis.pid_turn_set(90_deg, TURN_SPEED); chassis.pid_wait();
  // 15 Forward
  chassis.pid_drive_set(10_in, DRIVE_SPEED); chassis.pid_wait();
  // 16 Left 90
  chassis.pid_turn_set(-90_deg, TURN_SPEED); chassis.pid_wait();
  // 17 Forward
  chassis.pid_drive_set(10_in, DRIVE_SPEED); chassis.pid_wait();
  // 18 Intake
  intake.move(OUTTAKE_SPEED);
  // 19 Backwards
  chassis.pid_drive_set(-10_in, DRIVE_SPEED); chassis.pid_wait();
  // 20 Outtake
  outtake.move(OUTTAKE_SPEED);
  // 21 Forward
  chassis.pid_drive_set(10_in, DRIVE_SPEED); chassis.pid_wait();
  // 22 Right 90
  chassis.pid_turn_set(90_deg, TURN_SPEED); chassis.pid_wait();
  // 23 Forward
  chassis.pid_drive_set(10_in, DRIVE_SPEED); chassis.pid_wait();
  // 24 Right 90
  chassis.pid_turn_set(90_deg, TURN_SPEED); chassis.pid_wait();
  // 25 Forward
  chassis.pid_drive_set(10_in, DRIVE_SPEED); chassis.pid_wait();
  // 26 Left 90
  chassis.pid_turn_set(-90_deg, TURN_SPEED); chassis.pid_wait();
  // 27 Forward
  chassis.pid_drive_set(10_in, DRIVE_SPEED); chassis.pid_wait();
  // 28 Intake
  intake.move(OUTTAKE_SPEED);
  // 29 Backward
  chassis.pid_drive_set(-10_in, DRIVE_SPEED); chassis.pid_wait();
  // 30 Outtake
  outtake.move(OUTTAKE_SPEED);
  // 31 Forward
  chassis.pid_drive_set(10_in, DRIVE_SPEED); chassis.pid_wait();
  // 32 Right 90
  chassis.pid_turn_set(90_deg, TURN_SPEED); chassis.pid_wait();
  // 33 Forward
  chassis.pid_drive_set(10_in, DRIVE_SPEED); chassis.pid_wait();
  // 34 Right 90
  chassis.pid_turn_set(90_deg, TURN_SPEED); chassis.pid_wait();
  // 35 Forward
  chassis.pid_drive_set(10_in, DRIVE_SPEED); chassis.pid_wait();
  // 36 Right 90
  chassis.pid_turn_set(90_deg, TURN_SPEED); chassis.pid_wait();
  // 37 Forward
  chassis.pid_drive_set(10_in, DRIVE_SPEED); chassis.pid_wait();
  // 38 Left 90
  chassis.pid_turn_set(-90_deg, TURN_SPEED); chassis.pid_wait();
  // 39 Forward
  chassis.pid_drive_set(10_in, DRIVE_SPEED); chassis.pid_wait();
  // 40 Intake
  intake.move(OUTTAKE_SPEED);
  // 41 Backward
  chassis.pid_drive_set(-10_in, DRIVE_SPEED); chassis.pid_wait();
  // 42 Outtake
  outtake.move(OUTTAKE_SPEED);
  // 43 Forward
  chassis.pid_drive_set(10_in, DRIVE_SPEED); chassis.pid_wait();
  // 44 Right 135
  chassis.pid_turn_set(135_deg, TURN_SPEED); chassis.pid_wait();
  // 45 Forward
  chassis.pid_drive_set(10_in, DRIVE_SPEED); chassis.pid_wait();
  // 46 Intake
  intake.move(OUTTAKE_SPEED);
  // 47 Outtake
  outtake.move(OUTTAKE_SPEED);
  // 48 Backward
  chassis.pid_drive_set(-10_in, DRIVE_SPEED); chassis.pid_wait();
  // 49 Left 90
  chassis.pid_turn_set(-90_deg, TURN_SPEED); chassis.pid_wait();
  // 50 Forward
  chassis.pid_drive_set(10_in, DRIVE_SPEED); chassis.pid_wait();
  // 51 Rotate 100000 -> interpret as very large turn
  chassis.pid_turn_set(100000_deg, TURN_SPEED); chassis.pid_wait();
}

// TUFF NAME BRO TRUST ME
void razorAutonRight() {
  // 1. Forward
  chassis.pid_drive_set(24_in, DRIVE_SPEED); chassis.pid_wait();
  
  // 2. Turn Right 90
  chassis.pid_turn_set(90_deg, TURN_SPEED); chassis.pid_wait();
  
  // 3. Extend Matchloader Piston
  matchloader.set(true);
  
  // 4. Forward
  chassis.pid_drive_set(12_in, DRIVE_SPEED); chassis.pid_wait();
  
  // 5. Wait 2 or 3 seconds to collect balls
  pros::delay(2500); 
  
  // 6. Backwards (to goal)
  chassis.pid_drive_set(-24_in, DRIVE_SPEED); chassis.pid_wait();
  
  // 7. Outtake
  outtake.move(0); 
  pros::delay(2500); 
  outtake.move_voltage(0); 
  
  // 8. Backwards
  chassis.pid_drive_set(-12_in, DRIVE_SPEED); chassis.pid_wait();
  
  // 9. Turn 45 degrees to left
  chassis.pid_turn_set(-45_deg, TURN_SPEED); chassis.pid_wait();
  
  // 10. Backwards
  chassis.pid_drive_set(-12_in, DRIVE_SPEED); chassis.pid_wait();
  
  // 11. Turn 45 Degrees to right
  chassis.pid_turn_set(45_deg, TURN_SPEED); chassis.pid_wait();
  
  // 12. Extend Descore Piston
  wing.set(true);
  
  // 13. Backward (until auton line)
  chassis.pid_drive_set(-36_in, DRIVE_SPEED); chassis.pid_wait();
}

void razorAutonLeft() {
  // Add your own auton code here!
}

void soloAWP() {
  // Add your own auton code here!
}

// tuning
void tuningRotate() {
  chassis.pid_turn_set(360_deg, TURN_SPEED);
  chassis.pid_wait();
}