#ifndef MPU6050_H
#define MPU6050_H

#include <math.h>
#include <cstddef>
extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}
#include <sys/ioctl.h>
#include <cstdlib>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

class MPU6050
{
public:
    MPU6050(int address=0x68);
    ~MPU6050();

    bool openMPU6050();
    void closeMPU6050();

    void calibration();

    void readAccel();

    void readGyro();

    // Read the given register
    int readByte(int readRegister);
    // Write the the given value to the given register
    int writeByte(int writeRegister, int writeValue);

    int getError();

    int inline getAccelX(){return accel_x_raw_ / 16384;};
    int inline getAccelY(){return accel_y_raw_ / 16384;};
    int inline getAccelZ(){return accel_z_raw_ / 16384;};
    int inline getGyroX(){return gyro_x_raw_ / 131 / 180 * M_PI;};
    int inline getGyroY(){return gyro_y_raw_ / 131 / 180 * M_PI;};
    int inline getGyroZ(){return gyro_z_raw_ / 131 / 180 * M_PI;};

    unsigned char kI2CBus;         // I2C bus of the MPU6050
    int kI2CFileDescriptor;        // File Descriptor to the MPU6050
    int kI2CAddress;               // Address of MPU6050; defaults to 0x68
    int error;

private:
    int16_t accel_x_raw_;
    int16_t accel_y_raw_;
    int16_t accel_z_raw_;
    int16_t gyro_x_raw_;
    int16_t gyro_y_raw_;
    int16_t gyro_z_raw_;

    int16_t accel_x_offset_;
    int16_t accel_y_offset_;
    int16_t accel_z_offset_;
    int16_t gyro_x_offset_;
    int16_t gyro_y_offset_;
    int16_t gyro_z_offset_;
};

#define MPU6050_WHO_AM_I        0x75  // Read Only
#define MPU6050_PWR_MGMT_1      0x6B  // Read and Write
#define MPU_ADDRESS             0x68
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C

#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_ACCEL_XOUT_L    0x3C
#define MPU6050_ACCEL_YOUT_H    0x3D
#define MPU6050_ACCEL_YOUT_L    0x3E
#define MPU6050_ACCEL_ZOUT_H    0x3F
#define MPU6050_ACCEL_ZOUT_L    0x40

#define MPU6050_TEMP_OUT_H      0x41
#define MPU6050_TEMP_OUT_L      0x42

#define MPU6050_GYRO_XOUT_H     0x43
#define MPU6050_GYRO_XOUT_L     0x44
#define MPU6050_GYRO_YOUT_H     0x45
#define MPU6050_GYRO_YOUT_L     0x46
#define MPU6050_GYRO_ZOUT_H     0x47
#define MPU6050_GYRO_ZOUT_L     0x48

#endif //MPU6050_H
