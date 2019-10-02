#ifdef JETROV_CONTROL_DEAD_RECKONING_H
#define JETROV_CONTROL_DEAD_RECKONING_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "jetrov_control/const.h"

namespace jetrov_control
{

class DeadReckoning
{
public:
    DeadReckoning();
    ~DeadReckoning();

    void computeVelocity();
    void ComputeOdometry();
    void SetMsg();

    inline void SetCurrentPulse(const int& current_pulse){current_pulse_ = current_pulse;};
    inline void SetOmega(const double& omega){omega_ = omega;};
    
    inline nav_msgs::Odometry GetOdometry(){return odom_msg_;};

private:
    int current_pulse_;

    float dt_;

    double x_; //unit: m
    double y_; //unit: m
    double theta_; //unit: rad

    double vr_;  //unit: m/s
    double vx_;  //unit: m/s
    double vy_;  //unit: m/s
    double omega_; //unit: rad/s

    ros::Time current_time_, last_time_;
    geometry_msgs::Quaternion yaw_quat_msg_;
    nav_msgs::Odometry odom_msg_;

};

} //jetrov_control

#endif //JETROV_CONTROL_DEAD_RECKONING_H
