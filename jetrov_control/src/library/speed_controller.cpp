#include "jetrov_control/speed_controller.h"

namespace jetrov_control
{

SpeedController::SpeedController()
    :Kp_(1),
     Ki_(1),
     tgt_pulse_(0),
     current_pulse_(0),
     current_error_(0),
     previous_error_(0),
     output_(0){ }

SpeedController::~SpeedController(){ }

void SpeedController::GainReconfig(jetrov_control::JetrovControllerConfig& config)
{
    Kp_ = config.Kp;
    Ki_ = config.Ki;
}

void SpeedController::ComputeESCOutput()
{
    current_error_ = tgt_pulse_ - current_pulse_;
    output_ += Kp_ * (current_error_ - previous_error_) + Ki_ * current_error_;
    previous_error_ = current_error_;

    output_ = std::min(ESC_OUTPUT_MAX, output_);
    output_ = std::max(ESC_OUTPUT_MIN, output_);
}

} //namespace system_commander
