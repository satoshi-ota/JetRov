#include "jetrov_control/speed_controller.h"

namespace jetrov_control
{

SpeedController::SpeedController()
    :Kp_(1.0),
     Ki_(1.0),
     tgt_pulse_(0){ }

SpeedController::~SpeedController(){ }

void SpeedController::ComputeESCOutput()
{
    current_error_ = tgt_pulse_ - current_pulse_;
    output_ += Kp_ * (current_error_ - previous_error_) + Ki_ * current_error_;
    previous_error_ = current_error_;
}

} //namespace system_commander
