#ifndef BNO055_H
#define BNO055_H

#include <stdint.h>


// Data of 2 bytes have before LSB byte then MSB byte.
// Temperature is a single byte data.

#define BNO055_ADDRESS_COM3_LOW 		((uint8_t)0x28)
#define BNO055_ADDRESS_COM3_HIGH 		((uint8_t)0x29)
#define OPR_MODE_REGISTER 					((uint8_t)0x3D)
#define ACCX_DATA_REGISTER 					((uint8_t)0x08)
#define ACCY_DATA_REGISTER 					((uint8_t)0x0A)
#define ACCZ_DATA_REGISTER 					((uint8_t)0x0C)
#define MAGX_DATA_REGISTER 					((uint8_t)0x0E)
#define MAGY_DATA_REGISTER 					((uint8_t)0x10)
#define MAGZ_DATA_REGISTER 					((uint8_t)0x12)
#define GYROX_DATA_REGISTER 				((uint8_t)0x14)
#define GYROY_DATA_REGISTER 				((uint8_t)0x16)
#define GYROZ_DATA_REGISTER 				((uint8_t)0x18)
#define EUL_HEADING_DATA_REGISTER 	((uint8_t)0x1A)
#define EUL_ROLL_DATA_REGISTER 			((uint8_t)0x1C)
#define EUL_PITCH_DATA_REGISTER 		((uint8_t)0x1E)
#define QUAW_DATA_REGISTER 					((uint8_t)0x20)
#define QUAX_DATA_REGISTER 					((uint8_t)0x22)
#define QUAY_DATA_REGISTER 					((uint8_t)0x24)
#define QUAZ_DATA_REGISTER 					((uint8_t)0x26)
#define LIAX_DATA_REGISTER 					((uint8_t)0x28)
#define LIAY_DATA_REGISTER 					((uint8_t)0x2A)
#define LIAZ_DATA_REGISTER 					((uint8_t)0x2C)
#define GRAVX_DATA_REGISTER 				((uint8_t)0x2E)
#define GRAVY_DATA_REGISTER 				((uint8_t)0x30)
#define GRAVZ_DATA_REGISTER 				((uint8_t)0x32)
#define TEMPERATURE_REGISTER 				((uint8_t)0x34)
#define SYS_TRIGGER_REGISTER 				((uint8_t)0x3F)

#define CONFIG_MODE 								((uint8_t)0)    // Registers writable.
#define AMG 												((uint8_t)7)    // Raw data from acc, mag and gyro.
#define IMU 												((uint8_t)8)    // Orientation from acc and gyro.
#define NDOF 												((uint8_t)12)   // Everything on.


typedef struct
{
  int16_t accelerometer[3];
  int16_t gyroscope[3];
  uint16_t heading;
} BNO055_Imu_Raw_Data_t;

typedef struct
{
  float accelerometer[3];
  float gyroscope[3];
  float heading;
} BNO055_Imu_Converted_Data_t;


extern BNO055_Imu_Raw_Data_t raw_Data;
extern BNO055_Imu_Converted_Data_t converted_Data;


extern void BNO055_Init(void);
extern inline void BNO055_Imu_State_Machine(void);
extern inline BNO055_Imu_Raw_Data_t BNO055_Imu_Read_Data(void);
extern inline BNO055_Imu_Converted_Data_t BNO055_Imu_Convert_Data(void);
static inline void BNO055_Change_Mode(const uint8_t mode);
static inline uint16_t BNO055_Read_Data(uint8_t reg);
static inline void BNO055_Write_Register(const uint8_t reg, const uint8_t value);


#endif
