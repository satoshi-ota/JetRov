#include <math.h>

#include "jetrov_control/MPU6050.h"

MPU6050::MPU6050(int address)
{
    kI2CBus = 0;
    kI2CAddress = address;
    error = 0;
}

MPU6050::~MPU6050()
{
    closeMPU6050();
}

bool MPU6050::openMPU6050()
{
    char fileNameBuffer[32];
    sprintf(fileNameBuffer, "/dev/i2c-%d", kI2CBus);
    kI2CFileDescriptor = open(fileNameBuffer, O_RDWR);

    if(kI2CFileDescriptor < 0)
    {
        error = errno;
        return false;
    }

    if(ioctl(kI2CFileDescriptor, I2C_SLAVE, kI2CAddress) < 0)
    {
        error = errno;
        return false;
    }
    return true;
}

void MPU6050::closeMPU6050()
{
    if(kI2CFileDescriptor > 0)
    {
        close(kI2CFileDescriptor);
        kI2CFileDescriptor = -1;
    }
}

void MPU6050::calibration()
{

}

int MPU6050::readByte(int readRegister)
{
    int toReturn = i2c_smbus_read_byte_data(kI2CFileDescriptor, readRegister);
    if(toReturn < 0)
    {
        printf("MPU6050 Read Byte Error: %d", errno);
        error = errno;
        toReturn = -1;
    }
    return toReturn;
}

int MPU6050::writeByte(int writeRegister, int writeValue)
{
    int toReturn = i2c_smbus_write_byte_data(kI2CFileDescriptor, writeRegister, writeValue);
    if(toReturn < 0)
    {
        printf("MPU6050 Write Byte Error: %d", errno);
        error = errno;
        toReturn = -1;
    }
    return toReturn;
}

void MPU6050::readAccel()
{
    accel_x_raw_  = readByte(MPU6050_ACCEL_XOUT_H) << 8;
    accel_x_raw_ |= readByte(MPU6050_ACCEL_XOUT_L);
    accel_y_raw_  = readByte(MPU6050_ACCEL_YOUT_H) << 8;
    accel_y_raw_ |= readByte(MPU6050_ACCEL_YOUT_L);
    accel_z_raw_  = readByte(MPU6050_ACCEL_ZOUT_H) << 8;
    accel_z_raw_ |= readByte(MPU6050_ACCEL_ZOUT_L);

    accel_x_ = accel_x_raw_ / 16384.0;
    accel_y_ = accel_y_raw_ / 16384.0;
    accel_z_ = accel_z_raw_ / 16384.0;
    //for debugging
    //printf("Accel X: %lf\n", accel_x_);
}

void MPU6050::readGyro()
{
    gyro_x_raw_  = readByte(MPU6050_GYRO_XOUT_H) << 8;
    gyro_x_raw_ |= readByte(MPU6050_GYRO_XOUT_L);
    gyro_y_raw_  = readByte(MPU6050_GYRO_YOUT_H) << 8;
    gyro_y_raw_ |= readByte(MPU6050_GYRO_YOUT_L);
    gyro_z_raw_  = readByte(MPU6050_GYRO_ZOUT_H) << 8;
    gyro_z_raw_ |= readByte(MPU6050_GYRO_ZOUT_L);

    gyro_x_ = gyro_x_raw_ / 131.0 / 180 * M_PI;
    gyro_y_ = gyro_y_raw_ / 131.0 / 180 * M_PI;
    gyro_z_ = gyro_z_raw_ / 131.0 / 180 * M_PI;
}
