// main.cpp
// PLEASE USE SNAKE_CASE

#include "vex.h"
using namespace vex;

// Core objects
brain Brain;
competition Competition;

volatile bool paused = false;


float abs(float value){
  if (value < 0){
    return value * -1;
  }
  else if (value ==0){
    return 0;
  }
  else {return value;}
}


// General Constants
int deadzone = 5;
int cpu_MSEC_delay = 20;
const float wheel_diameter = 2.5; // inches
const double wheel_circumference = wheel_diameter * 3.141592;



// Exit tolerances
const float DISTANCE_TOLERANCE_DEG = 5.0;
const float HEADING_TOLERANCE_DEG  = 1.0;

// Turn PID constants
const float TURN_KP = 0.7;
const float TURN_KI = 0.0;
const float TURN_KD = 0.25;

const float TURN_ERROR_TOLERANCE = 1.0;   // degrees
const int   TURN_SETTLE_TIME_MS  = 120;

// Drive motors
motor front_left  = motor(PORT1, false);
motor front_right = motor(PORT20, true);
motor back_left   = motor(PORT4, false);
motor back_right  = motor(PORT9, true);

// Drive motor groups
motor_group left_side  = motor_group(back_left, front_left);
motor_group right_side = motor_group(back_right, front_right);

// Bottom intake motor
motor bottom_intake = motor(PORT3);
motor middle_intake = motor(PORT10);
motor top_intake = motor(PORT6);
motor high_intake = motor(PORT5);

// Controller
controller controller_1;

//Inertial
inertial inertial_sensor = inertial(PORT13);


//Piston
digital_out piston_extend = digital_out(Brain.ThreeWirePort.A);
digital_out piston_retract = digital_out(Brain.ThreeWirePort.B);
bool piston_out = false;


void pistonChange(void){
  piston_out = !piston_out;
  if (piston_out){
    piston_extend.set(true);
    piston_retract.set(false);
  }
  else {
    piston_extend.set(false);
    piston_retract.set(true);
  }
}
// Deadzone helper function
int apply_deadzone(int value, int zone = deadzone) {
  if (abs(value) < zone)
    return 0;
  return value;
}

//Drive PID Helper functions
double clamp(double value, double max_mag) {
  if (value >  max_mag) return  max_mag;
  if (value < -max_mag) return -max_mag;
  return value;
}



double average_drive_position_deg() {
  return (
    fabs(front_left.position(deg)) +
    fabs(back_left.position(deg)) +
    fabs(front_right.position(deg)) +
    fabs(back_right.position(deg))
  ) / 4.0;
}


// Intake Helper functions
void low_goal_score(void){
  bottom_intake.spin(forward, 100, pct);
  middle_intake.spin(reverse, 100, pct);
  top_intake.spin(reverse, 100, pct);
}

void top_goal_score(void){
  bottom_intake.spin(reverse, 100, pct);
  middle_intake.spin(forward, 100, pct);
  top_intake.spin(reverse, 100, pct);
  high_intake.spin(reverse, 100, pct);
}

void storage(void){
  bottom_intake.spin(reverse, 100, pct);
  middle_intake.spin(forward, 100, pct);
  top_intake.spin(forward, 100, pct);
}

void intake_stop(void){
  bottom_intake.stop();
  middle_intake.stop();
  top_intake.stop();
  high_intake.stop();
}


//WORKS!
void turn_to_angle(float target_angle, float kP = 0.2, float maxSpeed = 100){
    inertial_sensor.setRotation(0, degrees);

    float speed = 100;

    bool inSettlingRange = false;
    int timer = 0;

    float error;

    while (true){
      error = target_angle - inertial_sensor.rotation();
      if (abs(error) < 1){
        left_side.spin(forward, 0, percent);
        right_side.spin(forward, 0, percent);            
        inSettlingRange = true;
        break;
      }
      else {
        inSettlingRange = false;
      }
      

      if (inSettlingRange){
        timer += cpu_MSEC_delay;
      }
      else {
        timer = 0;
      }

      if (timer > 100){
        break;
      }

      speed = kP * error;
      if (speed > maxSpeed){
        speed = maxSpeed;
      }
      if (speed < -maxSpeed){
        speed = -maxSpeed;
      }


      left_side.spin(forward, speed, percent);
      right_side.spin(reverse, speed, percent);
      wait(cpu_MSEC_delay, msec);
    }
    left_side.spin(forward, 0, percent);
    right_side.spin(forward, 0, percent);

}
    
void drive_forward_pid(
  float distance_inches,
  float max_speed = 50,
  float kP_drive = 0.6,
  float kP_heading = 0.5
) {
  // Reset encoders
  front_left.setPosition(0, degrees);
  front_right.setPosition(0, degrees);
  back_left.setPosition(0, degrees);
  back_right.setPosition(0, degrees);

  // Target distance in degrees
  float target_rotation =
    (distance_inches / wheel_circumference) * 360.0;

  // Hold starting heading
  float start_heading = inertial_sensor.heading(degrees);

  int settle_time = 0;
  const int settle_required = 120;

  while (true) {

    // Pause-safe
    if (paused) {
      left_side.stop(brake);
      right_side.stop(brake);
      wait(cpu_MSEC_delay, msec);
      continue;
    }

    // Average encoder distance (magnitude only)
    float current_rotation =
      (fabs(front_left.position(degrees)) +
       fabs(front_right.position(degrees)) +
       fabs(back_left.position(degrees)) +
       fabs(back_right.position(degrees))) / 4.0;

    // Signed distance error
    float rotation_error = target_rotation - current_rotation;

    // Heading correction
    float heading_error =
      start_heading - inertial_sensor.heading(degrees);

    if (heading_error > 180) heading_error -= 360;
    if (heading_error < -180) heading_error += 360;

    float heading_correction = heading_error * kP_heading;

    // Drive PID (P-only, sign comes from error)
    float speed = rotation_error * kP_drive;
    speed = clamp(speed, fabs(max_speed));

    // Apply drive
    left_side.spin(forward,
      speed + heading_correction, percent);
    right_side.spin(forward,
      speed - heading_correction, percent);

    // Debug print
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Target: %.1f", target_rotation);
    Brain.Screen.newLine();
    Brain.Screen.print("Current: %.1f", current_rotation);
    Brain.Screen.newLine();
    Brain.Screen.print("Error: %.1f", rotation_error);
    Brain.Screen.newLine();
    Brain.Screen.print("Speed: %.1f", speed);

    // Exit condition
    if (fabs(rotation_error) < DISTANCE_TOLERANCE_DEG &&
        fabs(heading_error) < HEADING_TOLERANCE_DEG) {
      settle_time += cpu_MSEC_delay;
    } else {
      settle_time = 0;
    }

    if (settle_time >= settle_required) {
      break;
    }

    wait(cpu_MSEC_delay, msec);
  }

  // Stop motors
  left_side.stop(brake);
  right_side.stop(brake);
}

void pre_auton(void) {
  // Pre autonomous setup
  Brain.Screen.print("Calibrating....");
  inertial_sensor.calibrate();
  while (inertial_sensor.isCalibrating()){
    wait(cpu_MSEC_delay, msec);
  }

  Brain.Screen.clearScreen();
  Brain.Screen.print('Calibration Complete');
  wait(200, msec);
  Brain.Screen.clearScreen();
}

void autonomous(void) {
  /*
  left_side.spin(reverse, 100, percent);
  right_side.spin(reverse, 100, percent);
  wait(500, msec);
  
  
  left_side.spin(forward, 100, percent);
  right_side.spin(forward, 100, percent);
  wait(2000, msec);

  left_side.spin(reverse, 100, percent);
  right_side.spin(reverse, 100, percent);
  wait(750, msec);
  
  left_side.spin(forward, 0, percent);
  right_side.spin(forward, 0, percent);
  */


  storage();
  left_side.spin(forward, 100, percent);
  right_side.spin(forward, 100, percent);
  wait(1000, msec);



  left_side.spin(forward, 0, percent);
  right_side.spin(forward, 0, percent);
  intake_stop();


  turn_to_angle(-45);

  left_side.spin(forward, 100, percent);
  right_side.spin(forward, 100, percent);
  wait(200, msec);

  left_side.spin(forward, 0, percent);
  right_side.spin(forward, 0, percent);

  low_goal_score();

  wait(500, msec);

  intake_stop();

  turn_to_angle(135);

  left_side.spin(forward, 100, percent);
  right_side.spin(forward, 100, percent);
  wait(1500, msec);
  turn_to_angle(90);

  left_side.spin(forward, 100, percent);
  right_side.spin(forward, 100, percent);
  wait(2000, msec);

  left_side.spin(reverse, 100, percent);
  right_side.spin(reverse, 100, percent);
  wait(750, msec);
  
  left_side.spin(forward, 0, percent);
  right_side.spin(forward, 0, percent);








}

void usercontrol(void) {
  int forward_power;
  int turn;
  int left_power;
  int right_power;



  while (true) {
    
    
    forward_power = apply_deadzone(controller_1.Axis3.position());
    turn = apply_deadzone(controller_1.Axis1.position());

    left_power  = forward_power + turn;
    right_power = forward_power - turn;

    // Clamp values to valid motor range
    if (left_power > 100) {left_power = 100;}
    if (left_power < -100) {left_power = -100;}
    
    if (right_power > 100) {right_power = 100;}
    if (right_power < -100) {right_power = -100;}

    left_side.spin(forward, left_power, percent);
    right_side.spin(forward, right_power, percent);

    wait(cpu_MSEC_delay, msec);
  }
}



void display(void) {
  while (true) {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);

    Brain.Screen.print("Inertial Heading: %.1f", inertial_sensor.heading(degrees));
    Brain.Screen.newLine();
    Brain.Screen.print("Front Left: %.1f f", front_left.temperature(fahrenheit));
    Brain.Screen.newLine();
    Brain.Screen.print("Front Right: %.1f f", front_right.temperature(fahrenheit));
    Brain.Screen.newLine();
    Brain.Screen.print("Back Left: %.1f f", back_left.temperature(fahrenheit));
    Brain.Screen.newLine();
    Brain.Screen.print("Back Right: %.1f f", back_right.temperature(fahrenheit));

    wait(500, msec);
  }
}



void pause(void){
  left_side.spin(forward, 0, percent);
  right_side.spin(forward, 0, percent);
  paused = true;
}
int main() {
  pre_auton();
  //Competition properties
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  
  //Display
  //thread blinkScreenThread = thread(display);

  //Button Press Event Triggers
  controller_1.ButtonL2.pressed(top_goal_score);
  controller_1.ButtonL1.pressed(storage);
  controller_1.ButtonR1.pressed(low_goal_score);

  controller_1.ButtonL2.released(intake_stop);
  controller_1.ButtonL1.released(intake_stop);
  controller_1.ButtonR1.released(intake_stop);

  controller_1.ButtonB.pressed(pistonChange);
  controller_1.ButtonY.pressed(pistonChange);

  controller_1.ButtonA.pressed(pause);

  while (true) {
      wait(cpu_MSEC_delay, msec);
  }
}
