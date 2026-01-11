// main.cpp
// PLEASE USE SNAKE_CASE

#include "vex.h"
using namespace vex;

// Core objects
brain Brain;
competition Competition;

// Constants
int deadzone = 5;

// Drive motors
motor front_left  = motor(PORT20, false);
motor front_right = motor(PORT19, true);
motor back_left   = motor(PORT18, false);
motor back_right  = motor(PORT13, true);

// Drive motor groups
motor_group left_side  = motor_group(back_left, front_left);
motor_group right_side = motor_group(back_right, front_right);

// Bottom intake motor
motor bottom_intake = motor(PORT2);

// Controller
controller controller_1;

// Deadzone helper function
int apply_deadzone(int value, int zone = deadzone) {
  if (abs(value) < zone)
    return 0;
  return value;
}

// Intake Helper functions
void intake_on() {
  bottom_intake.spin(reverse, 100, pct);
}

void intake_off() {
  bottom_intake.stop();
}


void pre_auton(void) {
  // Pre autonomous setup
}

void autonomous(void) {
  // Autonomous code
}

void usercontrol(void) {
  int left_power;
  int right_power;
  while (true) {

    // Tank drive input
    left_power  = apply_deadzone(controller_1.Axis3.position());
    right_power = apply_deadzone(controller_1.Axis2.position());

    // Apply power to motors
    left_side.spin(fwd, left_power, pct);
    right_side.spin(fwd, right_power, pct);

    wait(20, msec);
  }
}

void display_temperatures() {
  while (true) {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Front Left: %.1f f", front_left.temperature(fahrenheit));
    Brain.Screen.newLine();
    Brain.Screen.print("Front Right: %.1f f", front_right.temperature(fahrenheit));
    Brain.Screen.newLine();
    Brain.Screen.print("Back Left: %.1f f", back_left.temperature(fahrenheit));
    Brain.Screen.newLine();
    Brain.Screen.print("Back Right: %.1f f", back_right.temperature(fahrenheit));

    wait(1000, msec);
  }
}

// Competition function setup
int main() {

  controller_1.ButtonR1.pressed(intake_on);
  controller_1.ButtonR1.released(intake_off);

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
