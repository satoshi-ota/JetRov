#include "jetrov_control/dead_reckoning.h"

namespace jetrov_control
{

DeadReckoning::DeadReckoning()
    :x_(0.0),
     y_(0.0),
     theta_(0.0),
     vr_(0.0),
     vx_(0.0),
     vy_(0.0),
     omega_(0.0),
     current_time_(ros::Time::now()),
     last_time_(ros::Time::now()){ }

DeadReckoning::~DeadReckoning(){ }

void DeadReckoning::computeVelocity()
{
    vr_ = current_pulse_ * CONTROL_FREQUENCY * ENCODER_WHEEL_DIAMETER  * M_PI / ENCODER_RESOLUTION;

    vx_ = vr_ * cos(theta_);
    vy_ = vr_ * sin(theta_);
}

void DeadReckoning::ComputeOdometry()
{
    current_time_ = ros::Time::now();
    dt_ = (current_time_ - last_time_).toSec();

    double delta_theta;
    double delta_x;
    double delta_y;

    if(omega_ == 0)
    {
      delta_theta = 0;
      delta_x = vx_ * dt_;
      delta_y = vy_ * dt_;
    }
    else
    {
      double radius = vr_ / omega_;
      delta_theta = omega_ * dt_;
      delta_x = radius * (sin(theta_ + delta_theta)-sin(theta_));
      delta_y = radius * (cos(theta_) - cos(theta_ + delta_theta));
    }

    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_theta;

    last_time_ = current_time_;
}

void DeadReckoning::SetMsg()
{
    yaw_quat_msg_ = tf::createQuaternionMsgFromYaw(theta_);

    //odom
    odom_msg_.header.stamp = ros::Time::now();
    odom_msg_.header.frame_id = "/odom";
    odom_msg_.child_frame_id = "/base_footprint";

    odom_msg_.pose.pose.position.x = x_;
    odom_msg_.pose.pose.position.y = y_;
    odom_msg_.pose.pose.position.z = 0.0;
    odom_msg_.pose.pose.orientation = yaw_quat_msg_;

    odom_msg_.twist.twist.linear.x = vx_;
    odom_msg_.twist.twist.linear.y = vy_;
    odom_msg_.twist.twist.angular.z = omega_;
}

}
