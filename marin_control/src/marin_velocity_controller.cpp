#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <marin_msgs/MarinCtrl.h>

const float pi = 3.14159265359;

class CTRL
{
private:
  ros::NodeHandle private_nh;
  marin_msgs::MarinCtrl output_msg;
  ros::Subscriber sub;
  ros::Publisher pub;

  int enc_ppr;
  float enc_dia;
  float ctrl_freq;
  float whl_base;

public:
  CTRL()
  {
    //get paramters
    if (!private_nh.getParam ("encoder_ppr", enc_ppr))
      enc_ppr = 1440;
    if (!private_nh.getParam ("encoder_diameter", enc_dia))
      enc_dia = 0.05;
    if (!private_nh.getParam ("control_frequency", ctrl_freq))
      ctrl_freq = 50;
    if (!private_nh.getParam ("wheel_base", whl_base))
      whl_base = 0.48;

    sub = private_nh.subscribe ("/cmd_vel", 1, &CTRL::ctrlCB, this);
    pub = private_nh.advertise<marin_msgs::MarinCtrl> ("/ackermann_cmd_vel", 0);
  }

  void ctrlCB (const geometry_msgs::Twist& msg)
  {
    output_msg.header.stamp = ros::Time::now();
    output_msg.tgt_pulse = msg.linear.x / ctrl_freq / enc_dia / pi * enc_ppr;
    output_msg.tgt_steer = ackermann_func(msg.linear.x, msg.angular.z);

    pub.publish (output_msg);
  }

  float ackermann_func(float v, float omega)
  {
    if(omega == 0 || v == 0) return 0;
    float radius = v / omega;
    return atan(whl_base / radius) * 180 / pi;
  }
};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "convert_to_ackermann_node");
  CTRL Marin_Ctrl;
  ros::spin();
}
