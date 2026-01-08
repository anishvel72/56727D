//main.cpp

#include "vex.h"

using namespace vex;

brain Brain;


// A global instance of competition
competition Competition;


//inertial inertialSensor = inertial(PORT10);


//Deadzone value

int deadzone = 5;

//global instances of motors and other devices here
motor frontLeft = motor(PORT10, false);
motor frontRight = motor(PORT20, true);
motor backLeft = motor(PORT15, false);;
motor backRight = motor(PORT2, true);
controller Controller1;




void pre_auton(void) {

  // All activities that occur before the competition starts (ex:  clearing encoders, setting servo positions ...)
}

void autonomous(void) {
  // Insert autonomous user code here.
}

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

    wait(1000, msec); // 1 time per second
  }
  return 0;
}



//Competition Function Setup and Callback
int main() {
  


  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
