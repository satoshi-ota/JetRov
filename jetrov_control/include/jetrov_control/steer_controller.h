#ifdef  JETROV_CONTROL_STEER_CONTROLLER_H
#define JETROV_CONTROL_STEER_CONTROLLER_H

#include <math.h>

#include <ros/ros.h>

#include "jetrov_control/const.h"

namespace jetrov_control
{

class SteerController
{
public:
    SteerController();
    ~SteerController();

    void Vel2SteerAngle();

    inline void SetLinearVel(const double& vel){vel_ = vel;};
    inline void SetangularVel(const double& omega){omega_ = omega;};

    inline double GetSteerAngle(){return steer_angle_;};

private:
    double vel_; // linear x
    double omega_; //angular z

    double steer_angle_;　//rad

};

} //namespace jetrov_control

#endif //JETROV_CONTROL_STEER_CONTROLLER_H
