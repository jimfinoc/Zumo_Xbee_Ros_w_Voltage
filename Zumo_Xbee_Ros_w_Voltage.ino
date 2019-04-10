/*
 * This example uses the ZumoMotors library to drive each motor on the Zumo
 * forward, then backward. The yellow user LED is on when a motor should be
 * running forward and off when a motor should be running backward. If a
 * motor on your Zumo has been flipped, you can correct its direction by
 * uncommenting the call to flipLeftMotor() or flipRightMotor() in the setup()
 * function.
 */

//#include <Wire.h>
#include <ros.h>
// #include "ArduinoHardware.h"
#include <ZumoShield.h>
#include <std_msgs/Float32.h>

#define LED_PIN 13

ZumoMotors motors;
ros::NodeHandle nh;
bool moveRobot;

struct TrackSpeedType {
  float move_forward_back_number;
  float move_left_right_number;
  float twist_left_right_number;
  float button_press_number;
  int left_speed;
  int right_speed;
} TrackSpeed;


void messageMoveForwardBack( const std_msgs::Float32& move_forward_back_msg){
  TrackSpeed.move_forward_back_number = move_forward_back_msg.data;
}

void messageMoveLeftRight( const std_msgs::Float32& move_left_right_msg){
  TrackSpeed.move_left_right_number = move_left_right_msg.data;
}

void messageTwistLeftRight( const std_msgs::Float32& twist_left_right_msg){
  TrackSpeed.twist_left_right_number = twist_left_right_msg.data;
}

void messageButtonPress( const std_msgs::Float32& button_press_msg){
  TrackSpeed.button_press_number = button_press_msg.data;
}


ros::Subscriber<std_msgs::Float32> subMoveForwardBack("zumo/move/forward_back_value", &messageMoveForwardBack );
ros::Subscriber<std_msgs::Float32> subMoveRightLeft("zumo/move/left_right_value", &messageMoveLeftRight );
ros::Subscriber<std_msgs::Float32> subTwistLeftRight("zumo/twist/left_right_value", &messageTwistLeftRight );
ros::Subscriber<std_msgs::Float32> subLeftStickButtonPress("zumo/button/left_stick_press_value", &messageButtonPress );

std_msgs::Float32 voltage_msg;
ros::Publisher voltage("zumo/voltage", &voltage_msg);


void setup()
{
  pinMode(LED_PIN, OUTPUT);
  nh.initNode();

  nh.advertise(voltage);

  nh.subscribe(subMoveForwardBack);
  nh.subscribe(subMoveRightLeft);
  nh.subscribe(subTwistLeftRight);
  nh.subscribe(subLeftStickButtonPress);



  TrackSpeed.left_speed = 0;
  TrackSpeed.right_speed = 0;
  // uncomment one or both of the following lines if your motors' directions need to be flipped
  //motors.flipLeftMotor(true);
  //motors.flipRightMotor(true);

}

void loop()
{
  TrackSpeed.left_speed = 0;
  TrackSpeed.right_speed = 0;

//  float batteryVoltage = analogRead(1) * 5000L * 3/2 / 1023;
//  voltage_msg.data = batteryVoltage;
  static uint32_t pre_voltage;
  if (millis()-pre_voltage >= 1000)
  {
    voltage_msg.data = analogRead(1) * 5.0 * 3/2 / 1023;
    voltage.publish( &voltage_msg );
  }

  //math for moving forward or back
  if (TrackSpeed.move_forward_back_number != 0)
  {
    TrackSpeed.left_speed = (int) (300 * TrackSpeed.move_forward_back_number);
    TrackSpeed.right_speed = (int) (300 * TrackSpeed.move_forward_back_number);
  }

  //math for moving left or right, but must be moving forward or back first
  if (TrackSpeed.move_left_right_number > 0)
  {
    TrackSpeed.left_speed  = (int)(TrackSpeed.left_speed * (1 - TrackSpeed.move_left_right_number*TrackSpeed.move_left_right_number));
  }
  else if (TrackSpeed.move_left_right_number < 0)
  {
    TrackSpeed.right_speed = (int)(TrackSpeed.right_speed * (1 - TrackSpeed.move_left_right_number*TrackSpeed.move_left_right_number));
  }

  //math for twisting left or right
  if (TrackSpeed.twist_left_right_number != 0)
  {
    TrackSpeed.left_speed += (int) (-75 * TrackSpeed.twist_left_right_number);
    TrackSpeed.right_speed += (int) (75 * TrackSpeed.twist_left_right_number);
  }

  //math with button for turbo speed!!
  if (TrackSpeed.button_press_number > 0 )
  {
    TrackSpeed.left_speed *= 2;
    TrackSpeed.right_speed *= 2;
    // TrackSpeed.left_speed = (int) (10 * TrackSpeed.move_forward_back_number);
    // TrackSpeed.right_speed = (int) (10 * TrackSpeed.move_forward_back_number);

  }

  // now get the motors moving
  if (TrackSpeed.left_speed != 0 || TrackSpeed.right_speed != 0 )
  {
    digitalWrite(LED_PIN, HIGH);
    motors.setLeftSpeed(  TrackSpeed.left_speed);
    motors.setRightSpeed( TrackSpeed.right_speed);
  } else {
    digitalWrite(LED_PIN, LOW);
    motors.setLeftSpeed(  0);
    motors.setRightSpeed( 0);
  }


  nh.spinOnce();
  delay(10);

}
