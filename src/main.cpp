/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       anish                                                     */
/*    Created:      10/9/2025, 5:12:18 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

brain Brain;


// A global instance of competition
competition Competition;

motor frontLeft = motor(PORT10, false);
motor frontRight = motor(PORT20, true);
motor backLeft = motor(PORT15, false);;
motor backRight = motor(PORT2, true);
controller Controller1;
//inertial inertialSensor = inertial(PORT10);


//Deadzone value

int deadzone = 5;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // Read controller axes
    double forward = 0;  // Forward/Backward
    double strafe = 0;  // Left/Right
    double turn = 0;  // Rotation

    if (Controller1.Axis3.position(percent) > deadzone || Controller1.Axis3.position(percent) < (deadzone*-1)) forward = Controller1.Axis3.position(percent);

    if (Controller1.Axis4.position(percent) > deadzone || Controller1.Axis4.position(percent) < (deadzone*-1)) strafe  = Controller1.Axis4.position(percent);

    if (Controller1.Axis1.position(percent) > deadzone || Controller1.Axis1.position(percent) < (deadzone*-1)) turn = Controller1.Axis1.position(percent);
    // Combine values for X-drive motion
    double frontLeftPower  = forward + strafe + turn;
    double frontRightPower = forward - strafe - turn;
    double backLeftPower   = forward - strafe + turn;
    double backRightPower  = forward + strafe - turn;

    // Clamp values manually to range [-100, 100]
    if (frontLeftPower > 100) frontLeftPower = 100;
    if (frontLeftPower < -100) frontLeftPower = -100;

    if (frontRightPower > 100) frontRightPower = 100;
    if (frontRightPower < -100) frontRightPower = -100;

    if (backLeftPower > 100) backLeftPower = 100;
    if (backLeftPower < -100) backLeftPower = -100;

    if (backRightPower > 100) backRightPower = 100;
    if (backRightPower < -100) backRightPower = -100;

    // Apply motor power
    frontLeft.spin(fwd,  frontLeftPower,  pct);
    frontRight.spin(fwd, frontRightPower, pct);
    backLeft.spin(fwd,   backLeftPower,   pct);
    backRight.spin(fwd,  backRightPower,  pct);

    wait(20, msec);  // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

int displayTemperatures() {
  while (true) {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Front Left: %.1f f", frontLeft.temperature(fahrenheit));
    Brain.Screen.newLine();
    Brain.Screen.print("Front Right: %.1f f", frontRight.temperature(fahrenheit));
    Brain.Screen.newLine();
    Brain.Screen.print("Back Left: %.1f f", backLeft.temperature(fahrenheit));
    Brain.Screen.newLine();
    Brain.Screen.print("Back Right: %.1f f", backRight.temperature(fahrenheit));

    wait(500, msec); // update twice per second
  }
  return 0;
}



//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.

  thread displayThread = thread(displayTemperatures);

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
