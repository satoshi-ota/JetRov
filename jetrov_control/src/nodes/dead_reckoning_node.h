#ifdef JETROV_CONTROL_DEAD_RECKONING_NODE_H
# define JETROV_CONTROL_DEAD_RECKONING_NODE_H

#include <ros/ros.h>

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

    //subscriber
    ros::Subscriber imu_sub_;
    ros::Subscriber pulse_sub_;

    //publisher
    ros::Publisher odom_pub_;

private:
    void ImuCB(const sensor_msgs::ImuPtr& imu_msg);
    void CurrentPulseCB(const std_msgs::Int32Ptr& pulse_msg);

};

}

#endif JETROV_CONTROL_DEAD_RECKONING_NODE_H
