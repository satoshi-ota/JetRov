#ifndef JETROV_JOY_INTERFACE_JOYSTICK_CONTROL_NODE_H
#define JETROV_JOY_INTERFACE_JOYSTICK_CONTROL_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <jetrov_msgs/default_topics.h>
#include <jetrov_control/const.h>

namespace joy_interface
{

struct Axes {
    int linear_x;
    int steer_angle;
    int linear_x_direction;
    int steer_angle_deirection;
};

struct Buttons {
    int emergency_stop;
    int ctrl_enable;
    int ctrl_mode;
};

struct Max {
    double linear_x;
    double steer_angle;
};

class JoyStickControlNode
{
public:
    JoyStickControlNode();
    ~JoyStickControlNode();

    void EmergencyStop();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    jetrov_msgs::Command cmd_msg_;

    ros::Subscriber joy_sub_;

    ros::Publisher vel_pub_;

private:
    void JoyCB();
};

} //namespace joy_interface

#endif //JETROV_JOY_INTERFACE_JOYSTICK_CONTROL_NODE_H
