#include "bno055.h"
#include "i2c.h"
#include "data.h"


BNO055_Imu_Raw_Data_t raw_Data;
BNO055_Imu_Converted_Data_t converted_Data;


extern void BNO055_Init(void)
{
  BNO055_Write_Register(SYS_TRIGGER_REGISTER, 128);
  BNO055_Change_Mode(NDOF);
}


extern inline BNO055_Imu_Raw_Data_t BNO055_Imu_Read_Data(void)
{ 
	for(uint8_t i = 0; i < 3; i++)
	{
		raw_Data.accelerometer[i] = BNO055_Read_Data(ACCX_DATA_REGISTER + (2 * i));
		raw_Data.gyroscope[i] = BNO055_Read_Data(GYROX_DATA_REGISTER + (2 * i));
	}

	raw_Data.heading = BNO055_Read_Data(EUL_HEADING_DATA_REGISTER);
  return raw_Data;
}


extern inline BNO055_Imu_Converted_Data_t BNO055_Imu_Convert_Data(void)
{
  for(uint8_t i = 0; i < 3; i++)
	{
		converted_Data.accelerometer[i] = (float)raw_Data.accelerometer[i] / (float)100;
		converted_Data.gyroscope[i] = (float)raw_Data.gyroscope[i] / (float)16;
	}
  
  converted_Data.heading = (float)raw_Data.heading / (float)16;
  return converted_Data;
}


static inline void BNO055_Change_Mode(const uint8_t mode)
{
	BNO055_Write_Register(OPR_MODE_REGISTER, mode);
 	
 	switch(mode)
 	{
    case CONFIG_MODE:
      HAL_Delay(10);
      break;

    default:
      HAL_Delay(25);
      break;
 	}
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


static inline void BNO055_Write_Register(const uint8_t reg, const uint8_t value)
{
	uint8_t buffer[2];
  HAL_StatusTypeDef err;

	buffer[0] = reg;
	buffer[1] = value;
	err = HAL_I2C_Master_Transmit(&hi2c4, (uint16_t)(BNO055_ADDRESS_COM3_HIGH << 1), buffer, 2, 1000);
  
  if(err != HAL_OK)
  {
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
    
    while(1)
    {
      HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
      HAL_Delay(100);
    }
  }
}
