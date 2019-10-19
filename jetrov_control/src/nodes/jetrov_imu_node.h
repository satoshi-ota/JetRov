#ifndef JETROV_CONROL_JETROV_IMU_NODE_H
#define JETROV_CONROL_JETROV_IMU_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "jetrov_control/MPU6050.h"
#include "jetrov_msgs/default_topics.h"

namespace jetrov_control
{

class JetrovImuNode
{
public:
    JetrovImuNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~JetrovImuNode();

    void sendImu();

    int inline getHz(){return hz_;};
private:
    //general
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    sensor_msgs::Imu imu_msg_;

    int hz_;

    MPU6050 *mpu6050 = new MPU6050();

    ros::Publisher imu_pub_;
};

}

#endif //JETROV_CONROL_JETROV_IMU_NODE_H
