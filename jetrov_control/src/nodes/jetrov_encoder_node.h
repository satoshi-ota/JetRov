#ifndef JETROV_CONROL_JETROV_ENCODER_NODE_H
#define JETROV_CONROL_JETROV_ENCODER_NODE_H

#include <ros/ros.h>

#include "jetrov_control/ArduinoI2C.h"
#include "jetrov_msgs/PulseCount.h"
#include "jetrov_msgs/default_topics.h"

namespace jetrov_control
{

class JetrovEncoderNode
{
public:
    JetrovEncoderNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~JetrovEncoderNode();

    void sendEncoder();

    int inline getHz(){return hz_;};
private:
    //general
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    jetrov_msgs::PulseCount pulse_msg_;

    int hz_;

    Arduino *arduino = new Arduino();

    ros::Publisher encoder_pub_;
};

}

#endif //JETROV_CONROL_JETROV_ENCODER_NODE_H
