#ifndef ARDUINO_H
#define ARDUINO_H

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

class Arduino
{
public:
    Arduino(int address=0x08);
    ~Arduino();

    bool openArduino();
    void closeArduino();

    void readArduino();

    // Read the given register
    int readByte(int readRegister);
    // Write the the given value to the given register
    int writeByte(int writeRegister, int writeValue);

    unsigned char kI2CBus;         // I2C bus of the Arduino
    int kI2CFileDescriptor;        // File Descriptor to the Arduino
    int kI2CAddress;               // Address of Arduino; defaults to 0x68
    int error;

    int inline getPulseCount(){return pulse_count_;};

private:
    int8_t pulse_count_;

};

#define ARDUINO_ADDRESS             0x08

#endif //ARDUINO_H
