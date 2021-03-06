/******************************************************************************************************************************************
  Sensor fusion modes are meant to calculate measures describing the orientation of the device in space. It can be distinguished 
  between non-absolute or relative orientation and absolute orientation. Absolute orientation means orientation of the sensor with 
  respect to the earth and its magnetic field. In other words, absolute orientation sensor fusion modes calculate the direction 
  of the magnetic north pole. In non-absolute or relative orientation modes, the heading of the sensor can vary depending on how
  the sensor is placed initially.
  
  The acceleration sensor is both exposed to the gravity force and to accelerations applied to the sensor due to movement. In fusion 
  modes it is possible to separate the two acceleration sources, and thus the sensor fusion data provides separately linear acceleration
  (i.e. acceleration that is applied due to movement) and the gravity vector.

  NDOF: This is a fusion mode with 9 degrees of freedom where the fused absolute orientation data is calculated from accelerometer, 
  gyroscope and the magnetometer. The advantages of combining all three sensors are a fast calculation, resulting in high output data 
  rate, and high robustness from magnetic field distortions. In this mode the Fast Magnetometer calibration is turned ON and thereby 
  resulting in quick calibration of the magnetometer and higher output data accuracy.
*******************************************************************************************************************************************/


#include "bno055.h"
#include "i2c.h"
#include "rtc.h"


BNO055_Imu_Raw_Data_t raw_Data;
BNO055_Imu_Converted_Data_t converted_Data;
BNO055_Imu_Converted_All_Data_t converted_All_Data;
BNO055_Calibration_State_t calibration_State;
BNO055_Self_Test_Result_t self_Test_Result;
float accelerometer_Divisor = BNO055_ACCEL_DIV_MSQ;
float gyroscope_Divisor = BNO055_GYRO_DIV_DPS;
float euler_Divisor = BNO055_EULER_DIV_DEG;
float temperature_Divisor = BNO055_TEMP_DIV_CELSIUS;
static uint8_t I2C_Data_Register_Buffer[45];
static int16_t I2C_Raw_Data_Register[23];


extern void BNO055_Init(void)
{
  BNO055_Unit_Config_t unit_Config;
  BNO055_Scale_Config_t sensor_Config;
  
  /*
  The fusion outputs of the BNO055 are tightly linked with the sensor configuration settings.
  Due to this fact, the sensor configuration is limited when BNO055 is configured to run in any
  of the fusion operating mode. In any of the non-fusion modes the configuration settings can
  be updated by writing to the configuration registers as defined in the following sections.
  The measurement units for the various data outputs (regardless of operation mode) can be
  configured by writing to the UNIT_SEL register.
  
  The temperature can be read from one of two sources, the temperature source can be
  selected by writing to the TEMP_SOURCE register.
  
  The output data format is based on the following convention regarding the rotation angles 
  for roll, pitch and heading / yaw:

    Android format:
    Pitch -> +180� to -180� (turning clockwise decreases values);
    Roll -> -90� to +90� (increasing with increasing inclination);
    Heading / Yaw ->  0� to 360� (turning clockwise increases values).

    Windows format:
    Pitch -> -180� to +180� (turning clockwise increases values);
    Roll -> -90� to +90� (increasing with increasing inclination);
    Heading / Yaw ->  0� to 360� (turning clockwise increases values).
  */

  unit_Config.acceleration = BNO055_ACCEL_UNIT_MSQ;
  unit_Config.gyroscope = BNO055_GYRO_UNIT_DPS;
  unit_Config.euler = BNO055_EULER_UNIT_DEG;
  unit_Config.fusion_Format = BNO055_FUSION_DATA_ANDROID;
  sensor_Config.accelerometer_Range = BNO055_ACCEL_RANGE_4G;
  sensor_Config.accelerometer_Bandwidth = BNO055_ACCEL_BW_31_25HZ;
  sensor_Config.accelerometer_Mode = BNO055_ACCEL_NORMAL;
  sensor_Config.magnetometer_Bandwidth = BNO055_MAG_DATA_OUTRATE_30HZ ;
  sensor_Config.magnetometer_Mode = BNO055_MAG_OPERATION_MODE_ACCURACY;
  sensor_Config.magnetometer_Power = BNO055_MAG_POWER_MODE_NORMAL;
  sensor_Config.gyroscope_Range = BNO055_GYRO_RANGE_2000DPS;
  sensor_Config.gyroscope_Bandwidth = BNO055_GYRO_BW_47HZ;
  sensor_Config.gyroscope_Mode = BNO055_GYRO_POWER_MODE_NORMAL;
  BNO055_Deint();
  BNO055_Write_Register(BNO055_PAGE_ID_ADDR, BNO055_PAGE_ZERO);
  BNO055_Write_Register(BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_CONFIG);
  HAL_Delay(20);
  BNO055_Write_Register(BNO055_PWR_MODE_ADDR, BNO055_POWER_MODE_NORMAL);
  BNO055_Write_Register(BNO055_TEMP_SOURCE_ADDR, BNO055_ACCEL_TEMP_EN);
  BNO055_Unit_Config(unit_Config);
  BNO055_Scale_Config(sensor_Config);
  BNO055_Self_Test_Result();
  BNO055_Set_External_Crystal_Use(BNO055_USE_EXTERNAL_CRYSTAL);
  BNO055_Write_Register(BNO055_PAGE_ID_ADDR, BNO055_PAGE_ZERO);
  BNO055_Write_Register(BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_NDOF);
  HAL_Delay(20);
}


extern inline void BNO055_Imu_Read_Data(void)
{
  uint8_t acc_Raw_Data[6];
  uint8_t gyr_Raw_Data[6];
  uint8_t heading_Raw_Data[2];
  uint8_t j = 0;
  
  BNO055_Read_Multiple_Register(BNO055_ACCEL_DATA_X_LSB_ADDR, acc_Raw_Data, 6);
  BNO055_Read_Multiple_Register(BNO055_GYRO_DATA_X_LSB_ADDR, gyr_Raw_Data, 6);
  BNO055_Read_Multiple_Register(BNO055_EULER_H_LSB_ADDR, heading_Raw_Data, 2);
  raw_Data.heading = ((int16_t)heading_Raw_Data[1] << 8) | (int16_t)heading_Raw_Data[0];

	for(uint8_t i = 0; i < 3; i++)
	{
    raw_Data.accelerometer[i] = ((int16_t)acc_Raw_Data[j + 1] << 8) | (int16_t)acc_Raw_Data[j];
    raw_Data.gyroscope[i] = ((int16_t)gyr_Raw_Data[j + 1] << 8) | (int16_t)gyr_Raw_Data[j];
    j += 2;
	}
  
  /*for(uint8_t i = 0; i < 3; i++)
	{
		raw_Data.accelerometer[i] = BNO055_Read_Data(BNO055_ACCEL_DATA_X_LSB_ADDR + (2 * i));
		raw_Data.gyroscope[i] = BNO055_Read_Data(BNO055_GYRO_DATA_X_LSB_ADDR + (2 * i));
	}

	raw_Data.heading = BNO055_Read_Data(BNO055_EULER_H_LSB_ADDR);*/
}


extern inline void BNO055_Imu_Read_All_Data(void)
{
  uint8_t j = 0;
  
  BNO055_Read_Multiple_Register(BNO055_ACCEL_DATA_X_LSB_ADDR, I2C_Data_Register_Buffer, 45);

  for(uint8_t i = 0; i < 22; i++)
  {
    I2C_Raw_Data_Register[i] = ((int16_t)I2C_Data_Register_Buffer[j + 1] << 8) | (int16_t)I2C_Data_Register_Buffer[j];
    j += 2;
  }
  
  I2C_Raw_Data_Register[22] = (int16_t)I2C_Data_Register_Buffer[44];
}


extern inline void BNO055_Imu_Convert_Data(void)
{
  for(uint8_t i = 0; i < 3; i++)
	{
		converted_Data.accelerometer[i] = raw_Data.accelerometer[i] / accelerometer_Divisor;
		converted_Data.gyroscope[i] = raw_Data.gyroscope[i] / gyroscope_Divisor;
	}
  
  converted_Data.heading = raw_Data.heading / euler_Divisor;
}


extern inline void BNO055_Imu_Convert_All_Data(void)
{
  for(uint8_t i = 0; i < 3; i++)
  {
    converted_All_Data.accelerometer[i] = I2C_Raw_Data_Register[i] / accelerometer_Divisor;
    converted_All_Data.magnetometer[i] = I2C_Raw_Data_Register[i + 3];
    converted_All_Data.gyroscope[i] = I2C_Raw_Data_Register[i + 6] / gyroscope_Divisor;
    converted_All_Data.euler[i] = I2C_Raw_Data_Register[i + 9] / euler_Divisor;
    converted_All_Data.quaterion[i] = I2C_Raw_Data_Register[i + 12];
    converted_All_Data.liner_Acceleration[i] = I2C_Raw_Data_Register[i + 16] / accelerometer_Divisor;
    converted_All_Data.gravitation_Acceleration[i] = I2C_Raw_Data_Register[i + 19] / accelerometer_Divisor;    
  }
  
  converted_All_Data.quaterion[3] = I2C_Raw_Data_Register[15];
  converted_All_Data.temperature = I2C_Raw_Data_Register[22] / temperature_Divisor;
}


extern inline void BNO055_Get_Calibration_Status(void)
{
  uint8_t calibration_Register_Value;
  
  // Current calibration status of a speficif sensor, read-only register.
  // 3 indicates fully calibrated; 0 indicates not calibrated.
  
  calibration_Register_Value = BNO055_Read_Register(BNO055_CALIB_STAT_ADDR);
  calibration_State.magnetometer = calibration_Register_Value & 0x03;
  calibration_State.accelerometer = (calibration_Register_Value >> 2) & 0x03;
  calibration_State.gyroscope = (calibration_Register_Value >> 4) & 0x03;
  calibration_State.system =  (calibration_Register_Value >> 6) & 0x03;
}


static void BNO055_Deint(void)
{
  //HAL_GPIO_WritePin(IMU_RESET_GPIO_Port, IMU_RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  //HAL_GPIO_WritePin(IMU_RESET_GPIO_Port, IMU_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(650);
}


//Crystal must be configured AFTER loading calibration data into BNO055.
static void BNO055_Set_External_Crystal_Use(uint8_t use_Crystal)
{

  uint8_t confg_Value;
  
  if((use_Crystal == BNO055_USE_EXTERNAL_CRYSTAL) | (use_Crystal == BNO055_USE_INTERNAL_OSCILLATOR))
  {
    BNO055_Write_Register(BNO055_SYS_TRIGGER_ADDR, use_Crystal);
  }
  
  else
  {
    confg_Value = BNO055_USE_INTERNAL_OSCILLATOR;
    BNO055_Write_Register(BNO055_SYS_TRIGGER_ADDR, confg_Value);
  }
}


static void BNO055_Self_Test_Result(void)
{
  uint8_t self_Test;
  
  BNO055_Read_Multiple_Register(BNO055_SELFTEST_RESULT_ADDR, &self_Test, 1);
  self_Test_Result.accelerometer = (self_Test & 0x01) > 0;
  self_Test_Result.magnetometer = (self_Test & 0x02) > 0;
  self_Test_Result.gryroscope = (self_Test & 0x04) > 0;
  self_Test_Result.microcontroller = (self_Test & 0x08) > 0;
}


static void BNO055_Unit_Config(BNO055_Unit_Config_t unit)
{
  uint8_t config_Byte = 0;
  config_Byte = unit.acceleration | (unit.gyroscope << 1) | (unit.euler << 2) | (unit.temperature << 4) | (unit.fusion_Format << 7);
  BNO055_Write_Register(BNO055_UNIT_SEL_ADDR, config_Byte);
  
  if(unit.acceleration == BNO055_ACCEL_UNIT_MG)
  {
    accelerometer_Divisor = BNO055_ACCEL_DIV_MG;
  }
  
  else
  {
    accelerometer_Divisor = BNO055_ACCEL_DIV_MSQ;
  }
  
  if(unit.gyroscope == BNO055_GYRO_UNIT_RPS)
  {
    gyroscope_Divisor = BNO055_GYRO_DIV_RPS;
  }
  
  else
  {
    gyroscope_Divisor = BNO055_GYRO_DIV_DPS;
  }
}


static void BNO055_Scale_Config(BNO055_Scale_Config_t scale)
{
  uint8_t reg_Value;
  
  BNO055_Write_Register(BNO055_PAGE_ID_ADDR, BNO055_PAGE_ONE);
  reg_Value = scale.accelerometer_Range | (scale.accelerometer_Bandwidth << 2) | (scale.accelerometer_Mode << 5);
  BNO055_Write_Register(BNO055_ACCEL_CONFIG_ADDR, reg_Value);
  reg_Value = scale.magnetometer_Bandwidth | (scale.magnetometer_Mode << 3) | (scale.magnetometer_Power << 5);
  BNO055_Write_Register(BNO055_MAG_CONFIG_ADDR, reg_Value);
  reg_Value = scale.gyroscope_Range | (scale.gyroscope_Bandwidth << 3);
  BNO055_Write_Register(BNO055_GYRO_CONFIG_ADDR, reg_Value);
  BNO055_Write_Register(BNO055_GYRO_MODE_CONFIG_ADDR, scale.gyroscope_Mode);
  BNO055_Write_Register(BNO055_PAGE_ID_ADDR, BNO055_PAGE_ZERO);
}


static inline uint8_t BNO055_Read_Register(uint8_t reg)
{
	uint8_t data = 0;

	HAL_I2C_Master_Transmit(&hi2c4, (uint16_t)(BNO055_ADDRESS_COM3_HIGH << 1), &reg, 1, 1000);
	HAL_I2C_Master_Receive(&hi2c4, (uint16_t)(BNO055_ADDRESS_COM3_HIGH << 1), &data, 1, 1000);
  return data;
}


static inline uint16_t BNO055_Read_Data(uint8_t reg)
{
	uint8_t buffer[2];
	uint16_t data = 0;

	HAL_I2C_Master_Transmit(&hi2c4, (uint16_t)(BNO055_ADDRESS_COM3_HIGH << 1), &reg, 1, 1000);
	HAL_I2C_Master_Receive(&hi2c4, (uint16_t)(BNO055_ADDRESS_COM3_HIGH << 1), buffer, 2, 1000);
	data = (((uint16_t)buffer[1] << 8) & 0xFF00) | ((uint16_t)buffer[0] & 0x00FF);
	return data;
}


static inline void BNO055_Read_Multiple_Register(uint8_t base_Reg, uint8_t *data, uint8_t n)
{
	HAL_I2C_Master_Transmit(&hi2c4, (uint16_t)(BNO055_ADDRESS_COM3_HIGH << 1), &base_Reg, 1, 1000);
	HAL_I2C_Master_Receive(&hi2c4, (uint16_t)(BNO055_ADDRESS_COM3_HIGH << 1), data, n, 1000);
}


static inline void BNO055_Write_Register(const uint8_t reg, const uint8_t value)
{
	uint8_t buffer[2];

	buffer[0] = reg;
	buffer[1] = value;
	HAL_I2C_Master_Transmit(&hi2c4, (uint16_t)(BNO055_ADDRESS_COM3_HIGH << 1), buffer, 2, 1000);
}
