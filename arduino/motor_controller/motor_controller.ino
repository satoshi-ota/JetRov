#include <ros.h>
#include <Arduino.h>
#include <MsTimer2.h>
#include <Servo.h>
#include <geometry_msgs/Twist.h>
#include <marin_msgs/MarinCtrl.h>
#include <marin_msgs/MarinFeedback.h>

#define VAL_MIN 1000
#define VAL_MAX 2000
#define NEUTRAL 1500

const int pinA = 2;
const int pinB = 3;

volatile int pulse_count;
int tgt_pulse;
float tgt_angle;
float Kp_current;
float Ki_current;
int old_error;
int control_amount;
int current_error;
int motor_control_val;

float feedback_gain_p;
float feedback_gain_i;

float steer_servo_val;

ros::NodeHandle  nh;
Servo motor;  // create object to control a esc
Servo steer;  // create object to control a steering servo

marin_msgs::MarinFeedback output_msgs;
ros::Publisher pub_ctrlFB("/controller_feedback", &output_msgs);

void enc_changedPinA(){
  if(digitalRead(pinA)){
    if(digitalRead(pinB)) --pulse_count;  //逆転
    else ++pulse_count;                   //正転
  } else {
    if(digitalRead(pinB)) ++pulse_count;  //正転
    else --pulse_count;                   //逆転
  }
}

void enc_changedPinB(){
  if(digitalRead(pinB)){
    if(digitalRead(pinA)) ++pulse_count;  //正転
    else --pulse_count;                   //逆転
  } else {
    if(digitalRead(pinA)) --pulse_count;  //逆転
    else ++pulse_count;                   //正転
  }
}

void setTargetCB(const marin_msgs::MarinCtrl& input_msg){
  tgt_pulse = input_msg.tgt_pulse;
  tgt_angle = input_msg.tgt_steer;
}

void setPIDparam(){
  Kp_current = feedback_gain_p; Ki_current = feedback_gain_i;
}

void output(){
  current_error = tgt_pulse - pulse_count;
  int control_input = Kp_current * (current_error - old_error) + Ki_current * current_error;
  control_amount += control_input;

  old_error = current_error;
  control_amount = min(250, control_amount);
  control_amount = max(-250, control_amount);
  motor_control_val = control_amount + NEUTRAL;
  steer_servo_val = 90 + tgt_angle  / 30 * 90;
  steer_servo_val = min(180, steer_servo_val);
  steer_servo_val = max(0, steer_servo_val);
  motor.writeMicroseconds(motor_control_val);
  steer.write(steer_servo_val);
  setRobotData();
  pub_ctrlFB.publish(&output_msgs);
  pulse_count = 0;
}

void setRobotData() {
  output_msgs.header.stamp = nh.now();
  output_msgs.tgt_pulse = tgt_pulse;
  output_msgs.pulse_count = pulse_count;
  output_msgs.pulse_error = current_error;
  output_msgs.current_esc_val = motor_control_val;
  output_msgs.current_servo_val =  steer_servo_val;
}

ros::Subscriber<marin_msgs::MarinCtrl> sub("/ackermann_cmd_vel", &setTargetCB);

void setup() {
  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub_ctrlFB);

  motor.attach(9);
  steer.attach(10);
  attachInterrupt(0, enc_changedPinA, CHANGE);
  attachInterrupt(1, enc_changedPinB, CHANGE);

  MsTimer2::set(20, output);
  MsTimer2::start();

  while(!nh.connected()) {nh.spinOnce();}

  //getParams
  if (!nh.getParam("feedback_gain_p", &feedback_gain_p, 2)){
    feedback_gain_p = 0.3;
  }

  if (!nh.getParam("feedback_gain_i", &feedback_gain_i, 2)){
    feedback_gain_i = 0.1;
  }

  setPIDparam();
}

void loop() {
  nh.spinOnce();
  delay(10);
}
