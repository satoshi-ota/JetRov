#ifndef JETROV_CONTROL_DEAD_RECKONING_NODE_H
#define JETROV_CONTROL_DEAD_RECKONING_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include "jetrov_msgs/default_topics.h"
#include "jetrov_msgs/PulseCount.h"
#include "jetrov_control/dead_reckoning.h"

namespace jetrov_control
{

class DeadReckoningNode
{
public:
    DeadReckoningNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~DeadReckoningNode();

    void sendOdometry();

private:
    //general
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    //class
    DeadReckoning dead_reckoning_;

    //message_filter
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;
    message_filters::Subscriber<jetrov_msgs::PulseCount> pulse_sub_;

    typedef message_filters::sync_policies::ApproximateTime
                             <sensor_msgs::Imu, jetrov_msgs::PulseCount> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;

    //publisher
    ros::Publisher odom_pub_;

private:
    void StatusCB(const sensor_msgs::ImuConstPtr &imu_msg,
                  const jetrov_msgs::PulseCountConstPtr &pulse_msg);

};

} //namespace jetrov_control

#endif //JETROV_CONTROL_DEAD_RECKONING_NODE_H
