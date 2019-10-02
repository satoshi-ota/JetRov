#include "dead_reckoning_node.h"

namespace jetrov_control
{

DeadReckoningNode::DeadReckoningNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    :nh_(nh),
     private_nh_(private_nh)
{
    imu_sub_ = nh_.subscribe("filterd_imu", 1, &DeadReckoningNode::ImuCB, this);
    pulse_sub_ = nh_.subscribe("enc_pulse", 1, &DeadReckoningNode::CurrentPulseCB, this);

    odom_pub_ = private_nh.advertise<nav_msgs::Odometry> ("/odom", 0);
}

DeadReckoningNode::~DeadReckoningNode(){ }

DeadReckoningNode::ImuCB(const sensor_msgs::ImuPtr& imu_msg)
{
    double omega = imu_msg->angular_velocity.z;
    dead_reckoning_.SetOmega(omega);

}

DeadReckoningNode::CurrentPulseCB(const std_msgs::Int32Ptr& pulse_msg)
{
    
}

} //namespace jetrov_control

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dead_reckoning_node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    jetrov_control::DeadReckoningNode dead_reckoning_node(nh, private_nh);

    ros::spin();

    return 0;
}
