/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <string.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>


// ================================================================
// ===                       USER DEFINE                       ===
// ================================================================

#define START_MOTOR  1
#define START_SERVOS 1

// ================================================================
// ===                    MOTOR/SERVO SETTINGS                  ===
// ================================================================


double MIN_LINEAR_BASE_PLAN = 0.079;
double MAX_LINEAR_BASE_PLAN = 0.319;
double MIN_ANGULAR_BASE_PLAN = -1.0;
double MAX_ANGULAR_BASE_PLAN = 1.0;

double MIN_LINEAR = 1.3;
double MAX_LINEAR = 2.0;
double MIN_ANGULAR = -1.5;
double MAX_ANGULAR = 1.5;

double STEP_TIME = 100.0;
double MIN_MOTOR_SPEED = 75.0;

int pwmPin0 = 5; ///< Arduino pin for motor 1 spin control.
int pwmPin1 = 6; ///< Arduino pin for motor 2 spin control.
int INaPin0 = 8; ///< Motor 1 Spin control.
int INbPin0 = 7; ///< Motor 1 Spin control.
int INaPin1 = 12; ///< Motor 2 Spin control.
int INbPin1 = 13; ///< Motor 2 Spin control.

int servoPin1 = 3;
int servoPin2 = 4;
int servoPin3 = 2;
int servoPin4 = 11;
int servoPin5 = 9;
int servoPin6 = 10;

uint16_t currentServo1 = 0;
uint16_t currentServo2 = 0;
int currentright = 0;
int currentleft = 0;
double rightdelay = 0;
double leftdelay = 0;
double delay = 0;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;

geometry_msgs::Twist twist_msg;
ros::Publisher cmd_vel_debug("cmd_vel_debug", &twist_msg);

// ================================================================
// ===                    FUNCTIONS                    ===
// ================================================================
/**
 * Servos setup function
 * 
 * This function attaches servos to their respective pins
 * 
 * @param none
 * @retval  none
 */
void start_servos(){
    // initialize servos
    servo1.attach(servoPin1);
    servo2.attach(servoPin2);
    servo3.attach(servoPin3);
    servo4.attach(servoPin4);
    servo5.attach(servoPin5);
    servo6.attach(servoPin6);
}

/**
 * Servo control function
 * 
 * The servo controller excepts an angle between 0 and 180 and
 * the particular Servo (class) you would like to control.
 * 
 * @param angle angle (0 to 180 degrees)
 * @param servo servo to set angle to
 */
void control_servo1(const std_msgs::UInt16& cmd_msg){
    while (currentServo1 != cmd_msg.data){
      if (abs(cmd_msg.data - currentServo1 <= 5.0)){
        currentServo1 = cmd_msg.data;
      }
      else{
        currentServo1 = currentServo1 + sgn(cmd_msg.data - currentServo1)*5;
      }
      servo5.write(CurrentServo1);
    }
}

void control_servo2(const std_msgs::UInt16& cmd_msg){
    while (currentServo2 != cmd_msg.data){
      if (abs(cmd_msg.data - currentServo2 <= 5.0)){
        currentServo2 = cmd_msg.data;
      }
      else{
        currentServo2 = currentServo2 + sgn(cmd_msg.data - currentServo2)*5;
      }
      servo6.write(CurrentServo2);
    }
}




/**
 * Motor setup function
 * 
 * This function sets up the pins to control 2 motors
 * 
 * @param none
 * @retval  none
 */
void start_motor(){
    // set pins to output
    pinMode(pwmPin0, OUTPUT);
    pinMode(pwmPin1, OUTPUT);
    pinMode(INaPin0, OUTPUT);
    pinMode(INbPin0, OUTPUT);
    pinMode(INaPin1, OUTPUT);
    pinMode(INbPin1, OUTPUT);
}

/**
 * Motor control function
 * 
 * The motor controler itself expects a positive integer as speed setting.
 * Forward an backwards rotation are handled through the INaPin and INbPin
 * pin.
 * 
 * @param speed controls rotation rate
 * @param pwmPin motor pin for speed control
 * @param INaPin motor pin for motor spin direction A
 * @param INbPin motor pin for motor spin direction B
 */
void write_to_motor(int speed, int pwmPin, int INaPin, int INbPin){
    if(speed > 0){
        analogWrite(pwmPin, speed);
        digitalWrite(INaPin, HIGH);
        digitalWrite(INbPin, LOW);
    }
    else if(speed < 0){
        analogWrite(pwmPin, -speed);
        digitalWrite(INaPin, LOW);
        digitalWrite(INbPin, HIGH);
    }
    else{
        digitalWrite(INaPin, LOW);
        digitalWrite(INbPin, LOW);
    }
}

/**
 * Function to tranform a value x which is in the current range to the target range.
 * 
 * @param x, the value that is within the current range
 * @param curr_min, the lower bound of the current range
 * @param curr_max, the upper bound of the current range
 * @param targ_min, the lower bound of the target range
 * @param targ_max, the upper bound of the target range
 * @retval y, the new value within the target range
 */
double scale_range(double x, double curr_min, double curr_max, double targ_min, double targ_max){
  return ((x - curr_min) / (curr_max - curr_min)) * (targ_max - targ_min) + targ_min;
}

/**
 * Main motor function
 * 
 * Takes a ros::geometry_msgs::Twist command converts it to a
 * right and left integer (for each motor) and writes to both
 * motors.
 * 
 * @param cmd_msg from the subsribed ros topic "cmd_vel"
 * @retval none
 */
void control_motor(const geometry_msgs::Twist& cmd_msg){
    double x;
    if(cmd_msg.linear.x == 0.0){
      x = 0.0;
    }else{
      x = scale_range(cmd_msg.linear.x,
                              MIN_LINEAR_BASE_PLAN,
                              MAX_LINEAR_BASE_PLAN,
                              MIN_LINEAR,
                              MAX_LINEAR);
    }
    double angular;
    if(cmd_msg.angular.z == 0.0){
     angular = 0.0;
    }else{
      angular = scale_range(cmd_msg.angular.z,
                                    MIN_ANGULAR_BASE_PLAN,
                                    MAX_ANGULAR_BASE_PLAN,
                                    MIN_ANGULAR,
                                    MAX_ANGULAR);
    }
    geometry_msgs::Twist debug_msg;
    
    debug_msg.linear.x = float(x);
    debug_msg.angular.z = float(angular);
    
    cmd_vel_debug.publish(&debug_msg);
    
    int right = int((x + angular) * 50);
    int left = int((x - angular) * 50);

    // control motors
    currentright = right;
    currentleft = left;
  }


// ================================================================
// ===                      SETUP                       ===
// ================================================================
/**
* Setup of the Arduino node.
* 
* - Set serial communication speed to 115200
* - Initialize motor controls
* - Initialize servo controls
*/

ros::NodeHandle nh;

ros::Subscriber<std_msgs::UInt16> sub_servo1("servo1", control_servo1);
ros::Subscriber<std_msgs::UInt16> sub_servo2("servo2", control_servo2);
ros::Subscriber<geometry_msgs::Twist> sub_motors("cmd_vel", control_motor);


void setup() {
    nh.initNode();
    nh.subscribe(sub_servo1);
    nh.subscribe(sub_servo2);
    nh.subscribe(sub_motors);
    nh.advertise(cmd_vel_debug);

    if (START_SERVOS == 1){
        start_servos();
    }
    if (START_MOTOR == 1){
        start_motor();
    }
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop(){
    nh.spinOnce();
    if ((abs(currentright) < MIN_MOTOR_SPEED)&&((abs(currentright)<MIN_MOTOR_SPEED)){
        rightdelay = STEP_TIME*(double)abs(currentright)/MIN_MOTOR_SPEED;
        leftdelay = STEP_TIME*(double)abs(currentleft)/MIN_MOTOR_SPEED;
        write_to_motor((double)sgn(currentright)*MIN_MOTOR_SPEED, pwmPin0, INaPin0, INbPin0);
        write_to_motor((double)sgn(currentleft)*MIN_MOTOR_SPEED, pwmPin1, INaPin1, INbPin1);
        delay(min(rightdelay, leftdelay));
      if (rightdelay < leftdelay){
        delay(rightdelay);
        write_to_motor(0.0, pwmPin0, INaPin0, INbPin0);
        delay(leftdelay-rightdelay);
        write_to_motor(0.0, pwmPin1, INaPin1, INbPin1);
        delay(STEP_TIME-leftdelay);
      }
      else{
        delay(leftdelay);
        write_to_motor(0.0, pwmPin1, INaPin1, INbPin1);
        delay(rightdelay-leftdelay);
        write_to_motor(0.0, pwmPin0, INaPin0, INbPin0);
        delay(STEP_TIME-rightdelay);
      }
    }
    else if ((abs(currentright) < MIN_MOTOR_SPEED)||((abs(currentright)<MIN_MOTOR_SPEED)){
      if (abs(currentright) < abs(currentright)){
        delay = STEP_TIME*(double)abs(currentright)/MIN_MOTOR_SPEED;
        write_to_motor((double)sgn(currentright)*MIN_MOTOR_SPEED, pwmPin0, INaPin0, INbPin0);
        write_to_motor(currentleft, pwmPin1, INaPin1, INbPin1);
        delay(delay);
        write_to_motor(0.0, pwmPin0, INaPin0, INbPin0);
      }
      else{
        delay = STEP_TIME*(double)abs(currentleft)/MIN_MOTOR_SPEED;
        write_to_motor((double)sgn(currentleft)*MIN_MOTOR_SPEED, pwmPin1, INaPin1, INbPin1);
        write_to_motor(currentright, pwmPin0, INaPin0, INbPin0);
        delay(delay);
        write_to_motor(0.0, pwmPin1, INaPin1, INbPin1);
      }
    }
    else{
      write_to_motor(currentright, pwmPin0, INaPin0, INbPin0);
      write_to_motor(currentleft, pwmPin1, INaPin1, INbPin1);
      delay(1);
    }
}
