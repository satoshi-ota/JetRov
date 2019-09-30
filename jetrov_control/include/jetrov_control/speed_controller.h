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

    inline void SetTargetPulse(const int& tgt_pulse){tgt_pulse_ = tgt_pulse;};
    inline void SetCurrentPulse(const int& current_pulse){current_pulse_ = current_pulse;};

    inline int getOutput(){return output_;};

private:
    double Kp_;
    double Ki_;

    //target_pulse
    int tgt_pulse_;

    //feedback_pulse
    int current_pulse_;

    //pulse error
    int current_error_;
    int previous_error_;

    //contoller output
    int output_;

};

} //namespace jetrov_control

#endif //JETROV_CONTROL_SPEED_CONTROLLER_H
