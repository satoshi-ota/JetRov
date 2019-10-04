#include "jetrov_control/steer_controller.h"

namespace jetrov_control
{

SteerController::SteerController()
    :vel_(0),
     omega_(0){ }

SteerController::~SteerController(){ }

void SteerController::Vel2SteerAngle()
{
    if(vel_ == 0  || omega_ == 0)
        steer_angle_ = 0;
    else
    {
        double radius = vel_ / omega_;
        steer_angle_ = atan(WHEEL_BASE / radius);
    }
}

} //namespace jetrov_control
