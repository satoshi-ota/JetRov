#ifndef JETROV_CONTROL_SPEED_CONTROLLER_H
#define JETROV_CONTROL_SPEED_CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include "jetrov_control/const.h"
#include "jetrov_control/common.h"

namespace jetrov_control
{

class SpeedController
{
public:
    SpeedController();
    ~SpeedController();

    void ComputeESCOutput();

    inline void SetTargetPulse(const std_msgs::Int32& tgt_pulse){tgt_pulse_ = tgt_pulse;};
    inline void SetCurrentPulse(const std_msgs::Int32& current_pulse){current_pulse_ = current_pulse;};

    inline std_msgs::Int32 getOutput(){return output_;};

private:
    //target_pulse
    std_msgs::Int32 tgt_pulse_;

    //feedback_pulse
    std_msgs::Int32 current_pulse_;

    //pulse error
    std_msgs::Int32 current_error_;
    std_msgs::Int32 previous_error_;

    //contoller output
    std_msgs::Int32 output_;

};

} //namespace jetrov_control

#endif //JETROV_CONTROL_SPEED_CONTROLLER_H
