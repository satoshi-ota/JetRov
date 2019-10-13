#include "jetrov_controller_node.h"

namespace jetrov_control
{

JetrovControllerNode::JetrovControllerNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    :nh_(nh),
     private_nh_(private_nh),
     steer_angle_(0.0)
{
    private_nh_.param("use_joy_", use_joy_, true);

    //set up dynamic reconfigure
    srv_ = boost::make_shared
            <dynamic_reconfigure::Server<jetrov_control::JetrovControllerConfig>>(private_nh);
    dynamic_reconfigure::Server<jetrov_control::JetrovControllerConfig>::CallbackType cb
        = boost::bind(&JetrovControllerNode::ControllerReconfigureCB, this, _1, _2);
    srv_->setCallback(cb);

    twist_sub_ = nh_.subscribe(jetrov_msgs::default_topics::COMMAND_VELOCITY, 1,
                               &JetrovControllerNode::DesireTwistCB, this);
    joy_sub_ = nh_.subscribe(jetrov_msgs::default_topics::COMMAND_JOY, 1,
                             &JetrovControllerNode::JoyCommandCB, this);
    pulse_sub_ = nh_.subscribe(jetrov_msgs::default_topics::STATUS_PULSE_COUNT, 1,
                               &JetrovControllerNode::CurrentPulseCB, this);

    InitializePWM();
    InitializePCA9685();
}

JetrovControllerNode::~JetrovControllerNode(){ }

void JetrovControllerNode::InitializePWM()
{
    esc_input_max_ = PWM_RESOLUTON * CONTROL_FREQUENCY * ESC_PULSE_WIDTH_MAX * 1e-6;
    esc_input_min_ = PWM_RESOLUTON * CONTROL_FREQUENCY * ESC_PULSE_WIDTH_MIN * 1e-6;
    esc_neutral_ = PWM_RESOLUTON * CONTROL_FREQUENCY * ESC_NEUTRAL * 1e-6;
    servo_input_max_ = PWM_RESOLUTON * CONTROL_FREQUENCY * STEER_SERVO_PULSE_WIDTH_MAX * 1e-6;
    servo_input_min_ = PWM_RESOLUTON * CONTROL_FREQUENCY * STEER_SERVO_PULSE_WIDTH_MIN * 1e-6;
    servo_neutral_ = PWM_RESOLUTON * CONTROL_FREQUENCY * STEER_SERVO_NEUTRAL * 1e-6;
}

void JetrovControllerNode::InitializePCA9685()
{
    int err = pca9685->openPCA9685();
    if (err < 0)
    {
        printf("Error: %d", pca9685->error);
    }
    else
    {
        printf("PCA9685 Device Address: 0x%02X\n",pca9685->kI2CAddress) ;
        pca9685->setAllPWM(0,0);
        pca9685->reset();
        pca9685->setPWMFrequency(CONTROL_FREQUENCY);
        pca9685->setPWM(STEER_SERVO_PORT, 0, servo_neutral_);
        pca9685->setPWM(ESC_PORT, 0, esc_neutral_);
    }
}


void JetrovControllerNode::ControllerReconfigureCB(
                                jetrov_control::JetrovControllerConfig &config,
                                uint32_t level)
{
    speed_controller_.GainReconfig(config);
}

void JetrovControllerNode::ControlESC()
{
    if(emergency_stop_ != true)
    {
        int tgt_pulse
        = linear_.x / FEEDBACK_FREQUENCY / ENCODER_WHEEL_DIAMETER  / M_PI * ENCODER_RESOLUTION;
        speed_controller_.SetTargetPulse(tgt_pulse);

        speed_controller_.ComputeESCOutput();
        int output = speed_controller_.getOutput();

        int output_pwm = map(output, ESC_OUTPUT_MIN, ESC_OUTPUT_MAX, esc_input_min_, esc_input_max_);
        pca9685->setPWM(ESC_PORT, 0, output_pwm);
    }
}

void JetrovControllerNode::ControlSteerServo()
{
    if(emergency_stop_ != true)
    {
        if(use_joy_ != true)
        {
            double vel = linear_.x;
            double omega = angular_.z;

            steer_controller_.SetLinearVel(vel);
            steer_controller_.SetAngularVel(omega);
            steer_controller_.Vel2SteerAngle();

            steer_angle_ = steer_controller_.GetSteerAngle();
        }

        double output = steer_angle_ / MAX_STEER_ANGLE;

        output = std::min(STEER_SERVO_OUTPUT_MAX, output);
        output = std::max(STEER_SERVO_OUTPUT_MIN, output);

        int output_pwm = map(output, STEER_SERVO_OUTPUT_MIN,
                                     STEER_SERVO_OUTPUT_MAX, servo_input_min_, servo_input_max_);
        pca9685->setPWM(STEER_SERVO_PORT, 0, output_pwm);
    }
}

void JetrovControllerNode::DesireTwistCB(const geometry_msgs::TwistPtr& twist_msg)
{
    linear_ = twist_msg->linear;
    angular_ = twist_msg->angular;
}

void JetrovControllerNode::JoyCommandCB(const jetrov_msgs::CommandPtr& cmd_msg)
{
    linear_ = cmd_msg->linear;
    steer_angle_ = cmd_msg->steer_angle;
    emergency_stop_ = cmd_msg->emergency_stop;
    if(emergency_stop_ == true)
    {
        pca9685->setPWM(STEER_SERVO_PORT, 0, servo_neutral_);
        pca9685->setPWM(ESC_PORT, 0, esc_neutral_);
    }
}

void JetrovControllerNode::CurrentPulseCB(const jetrov_msgs::PulseCountPtr& pulse_msg)
{
    speed_controller_.SetCurrentPulse(pulse_msg->pulse_count);
    ControlESC();
    ControlSteerServo();
}

} //namespace jetrov_control

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jetrov_controller_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  jetrov_control::JetrovControllerNode jetrov_controller_node(nh, private_nh);

  ros::spin();

  return 0;
}
