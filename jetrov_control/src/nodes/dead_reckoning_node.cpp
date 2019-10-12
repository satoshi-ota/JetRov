#include "dead_reckoning_node.h"

using namespace message_filters;
namespace jetrov_control
{

DeadReckoningNode::DeadReckoningNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    :nh_(nh),
     private_nh_(private_nh)
{
    //messsage filter
    imu_sub_.subscribe(nh_, jetrov_msgs::default_topics::STATUS_IMU, 1);
    pulse_sub_.subscribe(nh_, jetrov_msgs::default_topics::STATUS_PULSE_COUNT, 1);

    sync_.reset(new Sync(MySyncPolicy(10), imu_sub_, pulse_sub_));
    sync_->registerCallback(boost::bind(&DeadReckoningNode::StatusCB, this, _1, _2));

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>
                                    (jetrov_msgs::default_topics::STATUS_ODOMETRY, 0);
}

DeadReckoningNode::~DeadReckoningNode(){ }

void DeadReckoningNode::StatusCB(const sensor_msgs::ImuConstPtr &imu_msg,
                                 const jetrov_msgs::PulseCountConstPtr &pulse_msg)
{
    double omega = imu_msg->angular_velocity.z;
    dead_reckoning_.SetOmega(omega);

    int current_pulse = pulse_msg->pulse_count;
    dead_reckoning_.SetCurrentPulse(current_pulse);

    dead_reckoning_.ComputeOdometry();
    dead_reckoning_.SetMsg();

    nav_msgs::Odometry odometry_msg;
    odometry_msg = dead_reckoning_.GetOdometry();

    odom_pub_.publish(odometry_msg);
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
