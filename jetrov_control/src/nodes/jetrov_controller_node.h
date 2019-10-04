#ifndef JETROV_CONROL_JETROV_CONTROLLER_NODE_H
#define JETROV_CONROL_JETROV_CONTROLLER_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>

#include "jetrov_control/speed_controller.h"
#include "jetrov_control/steer_controller.h"
#include "jetrov_control/JetrovControllerConfig.h"

namespace jetrov_control
{

class JetrovControllerNode
{
public:
    JetrovControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~JetrovControllerNode();

    void InitializePWM();
    void InitializePCA9885();
    void ControllerReconfigureCB(jetrov_control::JetrovControllerConfig &config, uint32_t level);

    void ControlESC();
    void ControlSteerServo();

private:
    //general
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    //pmw
    int esc_input_max_;
    int esc_input_min_;
    int servo_input_max_;
    int servo_input_min_;

    boost::shared_ptr<dynamic_reconfigure::Server<jetrov_control::JetrovControllerConfig>> srv_;

    //topic
    geometry_msgs::Vector3 linear_, angular_;

    //class
    SpeedController speed_controller_;
    SteerController steer_controller_;
    //PCA9685 *pca9685 = new PCA9685();

    //subscriber
    ros::Subscriber twist_sub_;
    ros::Subscriber pulse_sub_;

private:
    void DesireTwistCB(const geometry_msgs::TwistPtr& twist_msg);
    void CurrentPulseCB(const std_msgs::Int32Ptr& pulse_msg);
};

} //namespace jetrov_control

#endif //JETROV_CONROL_JETROV_CONTROLLER_NODE_H
