#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <marin_msgs/MarinFeedback.h>
#include <marin_msgs/MarinStatus.h>

marin_msgs::MarinStatus robot_status;

void encCB(const marin_msgs::MarinFeedback& input_msg){
  robot_status.header.stamp = ros::Time::now();
  robot_status.header.frame_id = "/base_footprint";
  robot_status.pulse_count = input_msg.pulse_count;
}

void imuCB(const sensor_msgs::Imu& input_msg){
  robot_status.orientation = input_msg.orientation;
  robot_status.angular_velocity = input_msg.angular_velocity;
  robot_status.linear_acceleration = input_msg.linear_acceleration;
}

float hz=50;
int main(int argc, char **argv){
  ros::init(argc, argv, "robot_state_node");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  ros::Publisher output_pub = n.advertise<marin_msgs::MarinStatus> ("/robot_status", 0);
  ros::Subscriber input1_sub= n.subscribe("/controller_feedback", 10, encCB);
  ros::Subscriber input2_sub= n.subscribe("/imu/data", 10, imuCB);
  pn.getParam("hz",hz);

  ros::Rate loop_rate(hz);
  while(ros::ok()){
    output_pub.publish(robot_status);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
