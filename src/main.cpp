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

bool slowed_down = 0;

void slow_toggle(void){
  slowed_down = !slowed_down;
}

mutex odometry_mutex;
// General Constants
int deadzone = 5;
int cpu_MSEC_delay = 20;
const float wheel_diameter = 82.55; // mm
const double wheel_circumference = wheel_diameter * M_PI;

//Odometry
rotation forward_tracker = rotation(PORT7, true);
rotation sideways_tracker = rotation(PORT5);
inertial inertial_sensor = inertial(20, left);


double last_theta;
//gps gps_sensor = gps(PORT7, 180);

//Odometry Tracking values
double global_x = 0;
double global_y = 0;

double last_f = 0;
double last_s = 0;


// Drive motors
motor front_left  = motor(PORT1, true);
motor front_right = motor(PORT9, false);
motor back_left   = motor(PORT2, true);
motor back_right  = motor(PORT10, false);

// Drive motor groups
motor_group left_side  = motor_group(back_left, front_left);
motor_group right_side = motor_group(back_right, front_right);

// Bottom intake motor
motor bottom_intake = motor(PORT5);
motor middle1_intake = motor(PORT6);
motor middle2_intake = motor(PORT8);
motor high_intake = motor(PORT13, true);
motor unclog_storage_intake = motor(PORT11);


// Controller
controller controller_1;

//Piston
// Piston 1 (Original)
digital_out piston_extend_1 = digital_out(Brain.ThreeWirePort.A);
digital_out piston_retract_1 = digital_out(Brain.ThreeWirePort.B);
bool piston_out_1 = false;

// Piston 2 (New Piston)
digital_out piston_extend_2 = digital_out(Brain.ThreeWirePort.C);
digital_out piston_retract_2 = digital_out(Brain.ThreeWirePort.D);
bool piston_out_2 = false;

double TRACKING_WHEEL_CIRCUMFERENCE = 50.8 * M_PI; //mm

void piston1Change(void) {
  piston_out_1 = !piston_out_1;
  
  if (piston_out_1) {
    piston_extend_1.set(true);
    piston_retract_1.set(false);
  } else {
    piston_extend_1.set(false);
    piston_retract_1.set(true);
  }
}

// Toggle for Piston 2 (Extend/Retract)
void piston2Change(void) {
  piston_out_2 = !piston_out_2;
  
  if (piston_out_2) {
    piston_extend_2.set(true);
    piston_retract_2.set(false);
  } else {
    piston_extend_2.set(false);
    piston_retract_2.set(true);
  }
}
// Deadzone helper function
int apply_deadzone(int value, int zone = deadzone) {
  if (abs(value) < zone)
    return 0;
  return value;
}

// Intake Helper functions
void low_goal_score(void){
  bottom_intake.spin(reverse, 50, pct);
  middle1_intake.spin(reverse, 100, pct);
  unclog_storage_intake.spin(reverse, 100, pct);
  middle2_intake.spin(reverse, 100, pct);
  high_intake.spin(reverse, 100, pct);
}

void top_goal_score(void){
  bottom_intake.spin(forward, 100, pct);
  middle1_intake.spin(reverse, 100, pct);
  unclog_storage_intake.spin(reverse, 100, pct);
  middle2_intake.spin(forward, 100, pct);
  high_intake.spin(forward, 100, pct);
}

void storage(void){
  bottom_intake.spin(forward, 100, pct);
  middle1_intake.spin(forward, 100, pct);
  unclog_storage_intake.spin(reverse, 100, pct);
  middle2_intake.spin(forward, 100, pct);
  high_intake.spin(reverse, 100, pct);
}

void intake_stop(void){
  bottom_intake.stop();
  middle1_intake.stop();
  middle2_intake.stop();
  high_intake.stop();
  unclog_storage_intake.stop();
}

void middle_goal_score(void){
  bottom_intake.spin(forward, 100, pct);
  middle1_intake.spin(reverse, 100, pct);
  middle2_intake.spin(forward, 100, pct);
  high_intake.spin(reverse, 100, pct);
}


//WORKS!
void turn_by_angle(float target_angle, float kP = 0.2, float maxSpeed = 100){
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

void pre_auton(void) {
  // Pre autonomous setup
  Brain.Screen.print("Calibrating....");
  inertial_sensor.calibrate();
  while (inertial_sensor.isCalibrating()){
    wait(cpu_MSEC_delay, msec);
  }

  last_theta = inertial_sensor.rotation(deg);
  Brain.Screen.clearScreen();
  Brain.Screen.print('Calibration Complete');
  wait(200, msec);
  Brain.Screen.clearScreen();

  
  forward_tracker.setPosition(0, degrees);
  sideways_tracker.setPosition(0, degrees);
  inertial_sensor.setRotation(0, degrees);

}

void autonomous(void) {
  /*

  *** THIS IS THE ORIGINAL SKILLS AUTON
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
  

  left_side.spin(forward, 100, percent);
  right_side.spin(forward, 100, percent);
  wait(1500, msec);
  


  left_side.spin(forward, 30, percent);
  right_side.spin(forward, 30, percent);
  wait(500, msec);


  left_side.spin(forward, 30, percent);
  right_side.spin(forward, 40, percent);
  bottom_intake.spin(forward, 100, pct);
  middle1_intake.spin(forward, 100, pct);
  unclog_storage_intake.spin(reverse, 100, pct);
  middle2_intake.spin(forward, 100, pct);
  high_intake.spin(reverse, 100, pct);
  wait(1500, msec);

  left_side.spin(reverse, 20, percent);
  right_side.spin(reverse, 20, percent);
  wait(400, msec);
  
  left_side.stop(coast);
  right_side.stop(coast);

  wait(1000, msec);

  bottom_intake.spin(reverse, 30, pct);
  middle1_intake.spin(reverse, 50, pct);
  middle2_intake.spin(reverse, 50, pct);
  wait(1000, msec);

  bottom_intake.stop(coast);
  middle1_intake.stop(coast);
  middle2_intake.stop(coast);
  */
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
    if (left_power > (100)) {left_power = (100);}
    if (left_power < -(100)) {left_power = -(100);}
    
    if (right_power > (100)) {right_power = (100);}
    if (right_power < -(100)) {right_power = -(100);}


    if (slowed_down){
      left_power *= 0.3;
      right_power *= 0.3;
    }

    left_side.spin(forward, left_power, percent);
    right_side.spin(forward, right_power, percent);

    wait(cpu_MSEC_delay, msec);
  }
}

void display(void) {
  while (true) {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);

    Brain.Screen.print("MATCH-AUTON RUNS HERE");
    Brain.Screen.newLine();

    Brain.Screen.print("Forward Tracker: %.2f mm", forward_tracker.position(degrees));
    Brain.Screen.newLine();

    Brain.Screen.print("Sideways Tracker: %.2f mm", sideways_tracker.position(degrees));
    Brain.Screen.newLine();

    Brain.Screen.print("Inertial Heading: %.2f degrees", inertial_sensor.heading(degrees));
    Brain.Screen.newLine();


    odometry_mutex.lock();
    Brain.Screen.print("X: %.2f", global_x);
    Brain.Screen.newLine();
    Brain.Screen.print("Y: %.2f", global_y);
    odometry_mutex.unlock();



    wait(100, msec);
  }
}


/*

***OLD ODOMETRY TASK

void odom_Task(void) {
  while (true) {
    double f = forward_tracker.position(deg);
    double s = sideways_tracker.position(deg);

    double df = (f - last_f) * TRACKING_WHEEL_CIRCUMFERENCE / 360.0;
    double ds = (s - last_s) * TRACKING_WHEEL_CIRCUMFERENCE / 360.0;

    last_f = f;
    last_s = s;



    double theta = inertial_sensor.rotation(deg) * M_PI / 180.0;

    double dy = df * cos(theta) + ds * sin(theta);
    double dx = ds * cos(theta) - df * sin(theta);

    odometry_mutex.lock();
    global_x += dx;
    global_y += dy;
    odometry_mutex.unlock();
    wait(20, msec);
  }
}
*/


const double FORWARD_OFFSET = -95.0;   // mm (negative because it's to the LEFT)
const double SIDEWAYS_OFFSET = 32.0;   // mm (positive because it's in FRONT)

void odom_Task(void) {
  while (true) {
    double f = forward_tracker.position(deg);
    double s = sideways_tracker.position(deg);

    double df = (f - last_f) * TRACKING_WHEEL_CIRCUMFERENCE / 360.0;
    double ds = (s - last_s) * TRACKING_WHEEL_CIRCUMFERENCE / 360.0;

    last_f = f;
    last_s = s;

    double theta = inertial_sensor.rotation(deg) * M_PI / 180.0;
    double dtheta = theta - last_theta;
    last_theta = theta;

    // Remove arc motion from wheel movements
    double local_df = df - (SIDEWAYS_OFFSET * dtheta);
    double local_ds = ds + (FORWARD_OFFSET * dtheta);

    // Transform to global coordinates
    double dy = local_df * cos(theta) + local_ds * sin(theta);
    double dx = local_ds * cos(theta) - local_df * sin(theta);

    odometry_mutex.lock();
    global_x += dx;
    global_y += dy;
    odometry_mutex.unlock();
    wait(20, msec);
  }
}



int main() {
  pre_auton();
  //Competition properties
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  
  //Display and Odometry
  thread blinkScreen_thread = thread(display);
  thread odometry_thread = thread(odom_Task);

  controller_1.ButtonB.pressed(slow_toggle);
  controller_1.ButtonB.released(slow_toggle);
  //Button Press Event Triggers
  
  controller_1.ButtonL1.pressed(top_goal_score);
  controller_1.ButtonL2.pressed(storage);
  controller_1.ButtonL2.released(intake_stop);
  controller_1.ButtonL1.released(intake_stop);
  //controller_1.ButtonA.pressed(piston1Change);
  //controller_1.ButtonY.pressed(piston2Change);

  controller_1.ButtonR2.pressed(middle_goal_score);
  controller_1.ButtonR2.released(intake_stop);
  controller_1.ButtonR1.pressed(low_goal_score);
  controller_1.ButtonR1.released(intake_stop);


  while (true) {
      wait(cpu_MSEC_delay, msec);
  }
}
