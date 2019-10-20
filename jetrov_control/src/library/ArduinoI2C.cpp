#include <math.h>

#include "jetrov_control/ArduinoI2C.h"

Arduino::Arduino(int address)
{
    kI2CBus = 0;
    kI2CAddress = address;
    error = 0;
}

Arduino::~Arduino()
{
    closeArduino();
}

bool Arduino::openArduino()
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

void Arduino::closeArduino()
{
    if(kI2CFileDescriptor > 0)
    {
        close(kI2CFileDescriptor);
        kI2CFileDescriptor = -1;
    }
}

int Arduino::readByte(int readRegister)
{
    int toReturn = i2c_smbus_read_byte_data(kI2CFileDescriptor, readRegister);
    if(toReturn < 0)
    {
        printf("Arduino Read Byte Error: %d", errno);
        error = errno;
        toReturn = -1;
    }
    return toReturn;
}

int Arduino::writeByte(int writeRegister, int writeValue)
{
    int toReturn = i2c_smbus_write_byte_data(kI2CFileDescriptor, writeRegister, writeValue);
    if(toReturn < 0)
    {
        printf("Arduino Write Byte Error: %d", errno);
        error = errno;
        toReturn = -1;
    }
    return toReturn;
}

void Arduino::readArduino()
{
    pulse_count_  = readByte(ARDUINO_ADDRESS);
    //for debugging
    //printf("Accel X: %d\n", arduino_raw_);
}
