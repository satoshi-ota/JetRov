#include <ros.h>
#include <Arduino.h>
#include <Wire.h>
#include <sensor_msgs/Imu.h>

#define MPU6050_WHO_AM_I     0x75  // Read Only
#define MPU6050_PWR_MGMT_1   0x6B  // Read and Write
#define MPU_ADDRESS  0x68

const int n_calib = 100;
int i = 0;

int16_t gyroX, gyroY, gyroZ, accX, accY, accZ, Temperature;
long gyroX_off, gyroY_off, gyroZ_off, accX_off, accY_off, accZ_off;

ros::NodeHandle nh;

sensor_msgs::Imu imu;
ros::Publisher pub_imu("/imu/offset", &imu);



void setup() {

  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pub_imu);

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU6050_WHO_AM_I);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();

}

void loop()
{
  while(i < n_calib)
  {
    calibImu();
    i++;
  }
  sendOffset();
  i = 0;
  nh.spinOnce();
}

void calibImu()
{
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);
  while (Wire.available() < 14);

  accX = Wire.read() << 8 | Wire.read();
  accY = Wire.read() << 8 | Wire.read();
  accZ = Wire.read() << 8 | Wire.read();
  Temperature = Wire.read() << 8 | Wire.read();
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();

  accX_off += accX;
  accY_off += accY;
  accZ_off += accZ;
  gyroX_off += gyroX;
  gyroY_off += gyroY;
  gyroZ_off += gyroZ;
}

void sendOffset()
{
  imu.header.frame_id = "imu_offset";
  imu.header.stamp = nh.now();
  imu.angular_velocity.x = gyroX_off / n_calib;
  imu.angular_velocity.y = gyroY_off / n_calib;
  imu.angular_velocity.z = gyroZ_off / n_calib;
  imu.linear_acceleration.x = accX_off / n_calib;
  imu.linear_acceleration.y = accY_off / n_calib;
  imu.linear_acceleration.z = accZ_off / n_calib;

  accX_off = 0;
  accY_off = 0;
  accZ_off = 0;
  gyroX_off = 0;
  gyroY_off = 0;
  gyroZ_off = 0;

  pub_imu.publish(&imu);
}
