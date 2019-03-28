#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>
#include <marin_msgs/MarinCtrl.h>

class Joy
{
private:
  ros::NodeHandle private_nh;
  ros::Subscriber sub;
  ros::Publisher pub;
  marin_msgs::MarinCtrl output_msg;

public:
  Joy()
  {
    sub = private_nh.subscribe ("/joy", 1, &Joy::joy_cb, this);
    pub = private_nh.advertise<marin_msgs::MarinCtrl> ("/ackermann_cmd", 0);
  }
  void joy_cb (const sensor_msgs::Joy& input_msg)
  {
    float throttle = input_msg.axes[1];
    float steer = input_msg.axes[2];

    output_msg.header.stamp = ros::Time::now();
    output_msg.tgt_pulse = 60 * throttle;
    output_msg.tgt_steer = 30 * steer;

    pub.publish (output_msg);
  }
};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "joy_control_node");
  Joy JoyMarin;
  ros::spin();
}
