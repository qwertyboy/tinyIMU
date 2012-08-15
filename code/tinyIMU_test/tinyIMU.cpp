/*
tinyIMU.cpp
Arduino library for the tinyIMU 3-axis
accelerometer and gyroscope breakout board
*/

//Bring in the correct file
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "Wprogram.h"
#endif


#include <Wire.h>
#include "tinyIMU.h"

//Memory map for BMA220 accelerometer
//SPI memory map
#ifdef SPI_MODE  //Use this set of definitions for SPI

#define SOFTRESET 0x19
#define SUSPEND 0x18
#define WDT_TO_EN 0x17
#define WDT_TO_SEL 0x17
#define SPI3 0x17
#define SBIST_SIGN 0x11
#define SBIST 0x11
#define RANGE 0x11
#define SERIAL_HIGH_BW 0x10
#define FILT_CONFIG 0x10
#define SLEEP_EN 0x0F
#define SLEEP_DUR 0x0F
#define EN_X_CHANNEL 0x0F
#define EN_Y_CHANNEL 0x0F
#define EN_Z_CHANNEL 0x0F
#define RESET_INT 0x0E
#define LAT_INT 0x0E
#define EN_LOW 0x0E
#define EN_HIGH_X 0x0E
#define EN_HIGH_Y 0x0E
#define EN_HIGH_Z 0x0E
#define EN_DATA 0x0D
#define EN_ORIENT 0x0D
#define EN_SLOPE_X 0x0D
#define EN_CLOPE_Y 0x0D
#define EN_SLOPE_Z 0x0D
#define EN_TT_X 0x0D
#define EN_TT_Y 0x0D
#define EN_TT_Z 0x0D
#define TT_INT 0x0C
#define LOW_INT 0x0C
#define HIGH_INT 0x0C
#define DATA_INT 0x0C
#define SLOPE_INT 0x0C
#define ORIENT_INT 0x0B
#define ORIENT 0x0B
#define INT_FIRST_X 0x0B
#define INT_FIRST_Y 0x0B
#define INT_FIRST_Z 0x0B
#define INT_SIGN 0x0B
#define TIP_EN 0x0A
#define ORIENT_BLOCKING 0x0A
#define TT_SAMP 0x0A
#define ORIENT_EX 0x09
#define SLOPE_FILT 0x09
#define SLOPE_TH 0x09
#define SLOPE_DUR 0x09
#define TT_FILT 0x08
#define TT_TH 0x08
#define TT_DUR 0x08
#define LOW_HY 0x07
#define LOW_DUR 0x07
#define LOW_TH 0x06
#define HIGH_TH 0x06
#define HIGH_HY 0x05
#define HIGH_DUR 0x05
#define ACC_Z 0x04
#define ACC_Y 0x03
#define ACC_X 0x02
#define REVISION_ID 0x01
#define CHIP_ID 0x00

#endif //SPI_MODE



//I2C memory map
#ifdef I2C_MODE  //Use this set of definitions for I2C

#define SOFTRESET 0x32
#define SUSPEND 0x20
#define WDT_TO_EN 0x2E
#define WDT_TO_SEL 0x2E
#define SPI3 0x2E
#define SBIST_SIGN 0x22
#define SBIST 0x22
#define RANGE 0x22
#define SERIAL_HIGH_BW 0x20
#define FILT_CONFIG 0x20
#define SLEEP_EN 0x1E
#define SLEEP_DUR 0x1E
#define EN_X_CHANNEL 0x1E
#define EN_Y_CHANNEL 0x1E
#define EN_Z_CHANNEL 0x1E
#define RESET_INT 0x1C
#define LAT_INT 0x1C
#define EN_LOW 0x1C
#define EN_HIGH_X 0x1C
#define EN_HIGH_Y 0x1C
#define EN_HIGH_Z 0x1C
#define EN_DATA 0x1A
#define EN_ORIENT 0x1A
#define EN_SLOPE_X 0x1A
#define EN_CLOPE_Y 0x1A
#define EN_SLOPE_Z 0x1A
#define EN_TT_X 0x1A
#define EN_TT_Y 0x1A
#define EN_TT_Z 0x1A
#define TT_INT 0x18
#define LOW_INT 0x18
#define HIGH_INT 0x18
#define DATA_INT 0x18
#define SLOPE_INT 0x18
#define ORIENT_INT 0x16
#define ORIENT 0x16
#define INT_FIRST_X 0x16
#define INT_FIRST_Y 0x16
#define INT_FIRST_Z 0x16
#define INT_SIGN 0x16
#define TIP_EN 0x14
#define ORIENT_BLOCKING 0x14
#define TT_SAMP 0x14
#define ORIENT_EX 0x12
#define SLOPE_FILT 0x12
#define SLOPE_TH 0x12
#define SLOPE_DUR 0x12
#define TT_FILT 0x10
#define TT_TH 0x10
#define TT_DUR 0x10
#define LOW_HY 0xE
#define LOW_DUR 0xE
#define LOW_TH 0xC
#define HIGH_TH 0xC
#define HIGH_HY 0xA
#define HIGH_DUR 0xA
#define ACC_Z 0x8
#define ACC_Y 0x6
#define ACC_X 0x4
#define REVISION_ID 0x2
#define CHIP_ID 0x0

#endif //I2C_MODE


//Memory map for L3G4200 gyroscope
#define GYRO_WHO_AM_I 0x0F
#define GYRO_CTRL_REG1 0x20
#define GYRO_CTRL_REG2 0x21
#define GYRO_CTRL_REG3 0x22
#define GYRO_CTRL_REG4 0x23
#define GYRO_CTRL_REG5 0x24
#define GYRO_REFERENCE 0x25
#define GYRO_OUT_TEMP 0x26
#define GYRO_STATUS_REG 0x27
#define GYRO_OUT_X_L 0x28
#define GYRO_OUT_X_H 0x29
#define GYRO_OUT_Y_L 0x2A
#define GYRO_OUT_Y_H 0X2B
#define GYRO_OUT_Z_L 0x2C
#define GYRO_OUT_Z_H 0x2D
#define GYRO_FIFO_CTRL_REG 0x2E
#define GYRO_FIFO_SRC_REG 0x2F
#define GYRO_INT1_CFG 0x30
#define GYRO_INT1_SRC 0x31
#define GYRO_INT1_TSH_XH 0x32
#define GYRO_INT1_TSH_XL 0x33
#define GYRO_INT1_TSH_YH 0x34
#define GYRO_INT1_TSH_YL 0x35
#define GYRO_INT1_THS_ZH 0x36
#define GYRO_INT1_THS_ZL 0x37
#define GYRO_INT1_DURATION 0x38


void IMUinit(char MODE, byte ACCEL_CS, byte GYRO_CS)
{
  if(MODE == "SPI"){
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE2);
    
    digitalWrite(ACCEL_CS, HIGH);
    digitalWrite(GYRO_CS, HIGH);
  }
  
  if(MODE == "I2C"){
    Wire.begin();
  }
}
