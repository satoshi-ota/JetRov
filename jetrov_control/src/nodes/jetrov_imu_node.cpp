#include "jetrov_imu_node.h"

namespace jetrov_control
{

JetrovImuNode::JetrovImuNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    :nh_(nh),
     private_nh_(private_nh)
{
    private_nh_.param("hz_", hz_, 10);
    private_nh_.param("calibration_loop_", calibration_loop_, 100);

    imu_pub_ = nh_.advertise<sensor_msgs::Imu>
                                    (jetrov_msgs::default_topics::RAW_IMU, 0);

    int err = mpu6050->openMPU6050();
    if(err < 0)
    {
        printf("Error: %d", mpu6050->error);
    }
    else
    {
        printf("MPU6050 Device Address: 0x%02X\n", mpu6050->kI2CAddress);
        //for debugging
        int devAddress = mpu6050->readByte(MPU6050_WHO_AM_I);
        printf("I am 0x%02X\n", devAddress);

        mpu6050->writeByte(MPU6050_PWR_MGMT_1, 0x00);
    }

    mpu6050->calibration(calibration_loop_);
}

JetrovImuNode::~JetrovImuNode()
{
    mpu6050->closeMPU6050();
}

void JetrovImuNode::sendImu()
{
    mpu6050->readAccel();
    mpu6050->readGyro();

    imu_msg_.header.frame_id = "imu_link";
    imu_msg_.header.stamp = ros::Time::now();
    imu_msg_.angular_velocity.x = mpu6050->getGyroX();
    imu_msg_.angular_velocity.y = mpu6050->getGyroY();
    imu_msg_.angular_velocity.z = mpu6050->getGyroZ();
    imu_msg_.linear_acceleration.x = mpu6050->getAccelX();
    imu_msg_.linear_acceleration.y = mpu6050->getAccelY();
    imu_msg_.linear_acceleration.z = mpu6050->getAccelZ();

    imu_pub_.publish(imu_msg_);
}

} //namespace jetrov_control

int main(int argc, char **argv)
{
    ros::init(argc, argv, "jetrov_imu_node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    jetrov_control::JetrovImuNode jetrov_imu_node(nh, private_nh);

    int hz = jetrov_imu_node.getHz();
    ros::Rate loop_rate(hz);

    while (ros::ok())
    {
        jetrov_imu_node.sendImu();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
