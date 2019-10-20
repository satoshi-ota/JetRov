#include "jetrov_encoder_node.h"

namespace jetrov_control
{

JetrovEncoderNode::JetrovEncoderNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    :nh_(nh),
     private_nh_(private_nh)
{
    private_nh_.param("encoder_hz_", hz_, 50);

    encoder_pub_ = nh_.advertise<jetrov_msgs::PulseCount>
                                    (jetrov_msgs::default_topics::STATUS_PULSE_COUNT, 0);

    int err = arduino->openArduino();
    if(err < 0)
    {
        printf("Error: %d", arduino->error);
    }
    else
    {
        printf("Arduino Device Address: 0x%02X\n", arduino->kI2CAddress);
    }
}

JetrovEncoderNode::~JetrovEncoderNode()
{
    arduino->closeArduino();
}

void JetrovEncoderNode::sendEncoder()
{
    arduino->readArduino();

    pulse_msg_.header.frame_id = "encoder_link";
    pulse_msg_.header.stamp = ros::Time::now();
    pulse_msg_.pulse_count = arduino->getPulseCount();

    encoder_pub_.publish(pulse_msg_);
}

} //namespace jetrov_control

int main(int argc, char **argv)
{
    ros::init(argc, argv, "jetrov_encoder_node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    jetrov_control::JetrovEncoderNode jetrov_encoder_node(nh, private_nh);

    int hz = jetrov_encoder_node.getHz();
    ros::Rate loop_rate(hz);

    while (ros::ok())
    {
        jetrov_encoder_node.sendEncoder();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
