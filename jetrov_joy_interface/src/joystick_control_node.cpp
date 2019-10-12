#include "joystick_control_node.h"

namespace joy_interface
{

JoyStickControlNode::JoyStickControlNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    :nh_(nh),
     private_nh_(private_nh)
{
    cmd_msg_.linear.x = 0;
    cmd_msg_.linear.y = 0;
    cmd_msg_.linear.z = 0;
    cmd_msg_.steer_angle = 0;
    cmd_msg_.emergency_stop = false;

    private_nh_.param("axis_linear_x_", axes_.linear_x, 0);
    private_nh_.param("axis_steer_angle_", axes_.steer_angle, 1);

    private_nh_.param("axis_direction_linear_x", axes_.linear_x_direction, -1);
    private_nh_.param("axis_direction_pitch", axes_.steer_angle_direction, 1);

    private_nh_.param("max_linear_x_", max_.linear_x, jetrov_control::MAX_SPEED);
    private_nh_.param("max_steer_angle_", max_.steer_angle, jetrov_control::MAX_STEER_ANGLE);

    private_nh_.param("button_emergency_stop_", buttons_.emergency_stop, 0);
    private_nh_.param("control_mode_", buttons_.ctrl_mode, 1);

    joy_sub_ = nh_.subscribe("/joy", 1, &JoyStickControlNode::JoyCB, this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>
                                   (jetrov_msgs::default_topics::COMMAND_JOY, 0);
}

JoyStickControlNode::~JoyStickControlNode(){ }

void JoyStickControlNode::EmergencyStop()
{
    cmd_msg_.linear.x = 0;
    cmd_msg_.linear.y = 0;
    cmd_msg_.linear.z = 0;
    cmd_msg_.steer_angle = 0;
    cmd_msg_.emergency_stop = true;
}

void JoyStickControlNode::JoyCB(const sensor_msgs::JoyConstPtr& joy_msg)
{
    current_joy_ = *joy_msg;


    if(joy_msg->buttons[buttons_.emergency_stop])
    {
        EmergencyStop();
    }
    else
    {
        cmd_msg_.linear.x
            = joy_msg->axes[axes_.linear_x] * max_.linear_x * axes_.linear_x_direction;
        cmd_msg_.steer_angle
            = joy_msg->axes[axes_.steer_angle] * max_.steer_angle * axes_.steer_angle_direction;
    }

    vel_pub_.publish(cmd_msg_);
}

} //namespace joy_interface


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_interface_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  joy_interface::JoyStickControlNode joy_stick_control_node(nh, private_nh);

  ros::spin();

  return 0;
}
