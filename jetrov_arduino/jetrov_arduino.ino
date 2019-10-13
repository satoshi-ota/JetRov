#include <ros.h>
#include <Arduino.h>
#include <Wire.h>
#include <jetrov_msgs/PulseCount.h>
#include <sensor_msgs/Imu.h>

#define MPU6050_WHO_AM_I     0x75  // Read Only
#define MPU6050_PWR_MGMT_1   0x6B  // Read and Write
#define MPU_ADDRESS  0x68

volatile int pulse_count;
const int pinA = 2;
const int pinB = 3;
const int n_calib = 100;
const int i = 0;

int16_t gyroX, gyroY, gyroZ, ax, ay, az, Temperature;
int16_t gyroX_off, gyroY_off, gyroZ_off, ax_off, ay_off, az_off;

void enc_changedPinA()
{
  if(digitalRead(pinA))
  {
    if(digitalRead(pinB))
        ++pulse_count;
    else
        --pulse_count;
  }
  else
  {
    if(digitalRead(pinB))
        --pulse_count;
    else
        ++pulse_count;
  }
}

void enc_changedPinB()
{
  if(digitalRead(pinB))
  {
    if(digitalRead(pinA))
        --pulse_count;
    else
        ++pulse_count;
  }
  else
  {
    if(digitalRead(pinA))
        ++pulse_count;
    else
        --pulse_count;
  }
}

ros::NodeHandle nh;

sensor_msgs::Imu imu;
ros::Publisher pub_imu("/imu/data_raw", &imu);

jetrov_msgs::PulseCount cnt;
ros::Publisher pub_cnt("/status/pulse_count", &cnt);



void setup() {

  attachInterrupt(0, enc_changedPinA, CHANGE);
  attachInterrupt(1, enc_changedPinB, CHANGE);

  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pub_imu);
  nh.advertise(pub_cnt);

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU6050_WHO_AM_I);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);
  while (Wire.available() < 14);

  while (i < n_calib)
  {
      ax_off += Wire.read() << 8 | Wire.read();
      ay_off += Wire.read() << 8 | Wire.read();
      az_off += Wire.read() << 8 | Wire.read();
      Temperature = Wire.read() << 8 | Wire.read();
      gyroX_off += Wire.read() << 8 | Wire.read();
      gyroY_off += Wire.read() << 8 | Wire.read();
      gyroZ_off += Wire.read() << 8 | Wire.read();
  }

  ax_off = ax_off / n_calib;
  ay_off = ay_off / n_calib;
  az_off = ay_off / n_calib;
  gyroX_off = gyroX_off / n_calib;
  gyroY_off = gyroY_off / n_calib;
  gyroZ_off = gyroZ_off / n_calib;
}

void loop()
{
  sendImu();
  sendEncoder();
  nh.spinOnce();
}

void sendEncoder()
{
    cnt.header.frame_id = "encoder_link";
    cnt.header.stamp = nh.now();
    cnt.pulse_count = pulse_count;

    pulse_count = 0;

    pub_cnt.publish(&cnt);
}

void sendImu()
{
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);
  while (Wire.available() < 14);

  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  Temperature = Wire.read() << 8 | Wire.read();
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();

  imu.header.frame_id = "imu_link";
  imu.header.stamp = nh.now();
  imu.angular_velocity.x = (gyroX - gyroX_off) / 131.0 / 180 * PI;
  imu.angular_velocity.y = (gyroY - gyroY_off) / 131.0 / 180 * PI;
  imu.angular_velocity.z = (gyroZ - gyroZ_off) / 131.0 / 180 * PI;
  imu.linear_acceleration.x = (ax - ax_off) / 16384.0;
  imu.linear_acceleration.y = (ay - ay_off) / 16384.0;
  imu.linear_acceleration.z = (az - az_off) / 16384.0;

  pub_imu.publish(&imu);
}
