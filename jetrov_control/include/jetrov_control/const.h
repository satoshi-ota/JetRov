#ifndef JETROV_CONTROL_CONST_H
#define JETROV_CONTROL_CONST_H

namespace jetrov_control{

//steer rad
static constexpr double MAX_STEER_ANGLE = 0.05;

//wheel base m
static constexpr double WHEEL_BASE = 0.48;

//encoder
//wheel diameter m
static constexpr double ENCODER_WHEEL_DIAMETER = 0.05;

//encoder resolution
static constexpr int ENCODER_RESOLUTION = 1440;

//safty m/s
static constexpr double MAX_SPEED = 0.5;

//pwm resolution
static constexpr int PWM_RESOLUTON = 4096;

//controller frequency Hz
static constexpr int CONTROL_FREQUENCY = 50;

//ESC
//pulse width micro sec
static constexpr int ESC_PULSE_WIDTH_MAX = 2000;
static constexpr int ESC_PULSE_WIDTH_MIN = 1000;

//output range
static constexpr int ESC_OUTPUT_MAX = 255;
static constexpr int ESC_OUTPUT_MIN = -255;

//STEER Servo micro sec
static constexpr int STEER_SERVO_PULSE_WIDTH_MAX = 2500;
static constexpr int STEER_SERVO_PULSE_WIDTH_MIN = 500;

//output range
static constexpr int STEER_SERVO_OUTPUT_MAX = 1
static constexpr int STEER_SERVO_OUTPUT_MIN = -1;

}  //namespace jetrov_control

#endif // JETROV_CONTROL_CONST_H
