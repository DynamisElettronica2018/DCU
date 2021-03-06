#ifndef BNO055_H
#define BNO055_H

#include <stdint.h>


/********************************I2C ADDRESS********************************/ 

#define BNO055_ADDRESS_COM3_LOW 		          ((uint8_t)0x28)
#define BNO055_ADDRESS_COM3_HIGH 		          ((uint8_t)0x29)


/*************************PAG 0 REGISTER DEFINITION*************************/ 

#define BNO055_CHIP_ID_ADDR                 	((uint8_t)0x00)
#define BNO055_ACCEL_REV_ID_ADDR							((uint8_t)0x01)
#define BNO055_MAG_REV_ID_ADDR              	((uint8_t)0x02)
#define BNO055_GYRO_REV_ID_ADDR             	((uint8_t)0x03)
#define BNO055_SW_REV_ID_LSB_ADDR							((uint8_t)0x04)
#define BNO055_SW_REV_ID_MSB_ADDR							((uint8_t)0x05)
#define BNO055_BL_REV_ID_ADDR									((uint8_t)0x06)
#define BNO055_PAGE_ID_ADDR				            ((uint8_t)0x07)
#define BNO055_ACCEL_DATA_X_LSB_ADDR					((uint8_t)0x08)
#define BNO055_ACCEL_DATA_X_MSB_ADDR					((uint8_t)0x09)
#define BNO055_ACCEL_DATA_Y_LSB_ADDR					((uint8_t)0x0A)
#define BNO055_ACCEL_DATA_Y_MSB_ADDR					((uint8_t)0x0B)
#define BNO055_ACCEL_DATA_Z_LSB_ADDR					((uint8_t)0x0C)
#define BNO055_ACCEL_DATA_Z_MSB_ADDR					((uint8_t)0x0D)
#define BNO055_MAG_DATA_X_LSB_ADDR						((uint8_t)0x0E)
#define BNO055_MAG_DATA_X_MSB_ADDR						((uint8_t)0x0F)
#define BNO055_MAG_DATA_Y_LSB_ADDR						((uint8_t)0x10)
#define BNO055_MAG_DATA_Y_MSB_ADDR						((uint8_t)0x11)
#define BNO055_MAG_DATA_Z_LSB_ADDR						((uint8_t)0x12)
#define BNO055_MAG_DATA_Z_MSB_ADDR						((uint8_t)0x13)
#define BNO055_GYRO_DATA_X_LSB_ADDR						((uint8_t)0x14)
#define BNO055_GYRO_DATA_X_MSB_ADDR						((uint8_t)0x15)
#define BNO055_GYRO_DATA_Y_LSB_ADDR						((uint8_t)0x16)
#define BNO055_GYRO_DATA_Y_MSB_ADDR						((uint8_t)0x17)
#define BNO055_GYRO_DATA_Z_LSB_ADDR						((uint8_t)0x18)
#define BNO055_GYRO_DATA_Z_MSB_ADDR						((uint8_t)0x19)
#define BNO055_EULER_H_LSB_ADDR								((uint8_t)0x1A)
#define BNO055_EULER_H_MSB_ADDR								((uint8_t)0x1B)
#define BNO055_EULER_R_LSB_ADDR								((uint8_t)0x1C)
#define BNO055_EULER_R_MSB_ADDR								((uint8_t)0x1D)
#define BNO055_EULER_P_LSB_ADDR								((uint8_t)0x1E)
#define BNO055_EULER_P_MSB_ADDR								((uint8_t)0x1F)
#define BNO055_QUATERNION_DATA_W_LSB_ADDR			((uint8_t)0x20)
#define BNO055_QUATERNION_DATA_W_MSB_ADDR			((uint8_t)0x21)
#define BNO055_QUATERNION_DATA_X_LSB_ADDR			((uint8_t)0x22)
#define BNO055_QUATERNION_DATA_X_MSB_ADDR			((uint8_t)0x23)
#define BNO055_QUATERNION_DATA_Y_LSB_ADDR			((uint8_t)0x24)
#define BNO055_QUATERNION_DATA_Y_MSB_ADDR			((uint8_t)0x25)
#define BNO055_QUATERNION_DATA_Z_LSB_ADDR			((uint8_t)0x26)
#define BNO055_QUATERNION_DATA_Z_MSB_ADDR			((uint8_t)0x27)
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR		((uint8_t)0x28)
#define BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR		((uint8_t)0x29)
#define BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR		((uint8_t)0x2A)
#define BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR		((uint8_t)0x2B)
#define BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR		((uint8_t)0x2C)
#define BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR		((uint8_t)0x2D)
#define BNO055_GRAVITY_DATA_X_LSB_ADDR				((uint8_t)0x2E)
#define BNO055_GRAVITY_DATA_X_MSB_ADDR				((uint8_t)0x2F)
#define BNO055_GRAVITY_DATA_Y_LSB_ADDR				((uint8_t)0x30)
#define BNO055_GRAVITY_DATA_Y_MSB_ADDR				((uint8_t)0x31)
#define BNO055_GRAVITY_DATA_Z_LSB_ADDR				((uint8_t)0x32)
#define BNO055_GRAVITY_DATA_Z_MSB_ADDR				((uint8_t)0x33)
#define BNO055_TEMP_ADDR											((uint8_t)0x34)
#define BNO055_CALIB_STAT_ADDR								((uint8_t)0x35)
#define BNO055_SELFTEST_RESULT_ADDR						((uint8_t)0x36)
#define BNO055_INTR_STAT_ADDR									((uint8_t)0x37)
#define BNO055_SYS_CLK_STAT_ADDR							((uint8_t)0x38)
#define BNO055_SYS_STAT_ADDR									((uint8_t)0x39)
#define BNO055_SYS_ERR_ADDR										((uint8_t)0x3A)
#define BNO055_UNIT_SEL_ADDR									((uint8_t)0x3B)
#define BNO055_DATA_SELECT_ADDR								((uint8_t)0x3C)
#define BNO055_OPR_MODE_ADDR									((uint8_t)0x3D)
#define BNO055_PWR_MODE_ADDR									((uint8_t)0x3E)
#define BNO055_SYS_TRIGGER_ADDR								((uint8_t)0x3F)
#define BNO055_TEMP_SOURCE_ADDR								((uint8_t)0x40)
#define BNO055_AXIS_MAP_CONFIG_ADDR						((uint8_t)0x41)
#define BNO055_AXIS_MAP_SIGN_ADDR							((uint8_t)0x42)
#define BNO055_SIC_MATRIX_0_LSB_ADDR					((uint8_t)0x43)
#define BNO055_SIC_MATRIX_0_MSB_ADDR					((uint8_t)0x44)
#define BNO055_SIC_MATRIX_1_LSB_ADDR					((uint8_t)0x45)
#define BNO055_SIC_MATRIX_1_MSB_ADDR					((uint8_t)0x46)
#define BNO055_SIC_MATRIX_2_LSB_ADDR					((uint8_t)0x47)
#define BNO055_SIC_MATRIX_2_MSB_ADDR					((uint8_t)0x48)
#define BNO055_SIC_MATRIX_3_LSB_ADDR					((uint8_t)0x49)
#define BNO055_SIC_MATRIX_3_MSB_ADDR					((uint8_t)0x4A)
#define BNO055_SIC_MATRIX_4_LSB_ADDR					((uint8_t)0x4B)
#define BNO055_SIC_MATRIX_4_MSB_ADDR					((uint8_t)0x4C)
#define BNO055_SIC_MATRIX_5_LSB_ADDR					((uint8_t)0x4D)
#define BNO055_SIC_MATRIX_5_MSB_ADDR					((uint8_t)0x4E)
#define BNO055_SIC_MATRIX_6_LSB_ADDR					((uint8_t)0x4F)
#define BNO055_SIC_MATRIX_6_MSB_ADDR					((uint8_t)0x50)
#define BNO055_SIC_MATRIX_7_LSB_ADDR					((uint8_t)0x51)
#define BNO055_SIC_MATRIX_7_MSB_ADDR					((uint8_t)0x52)
#define BNO055_SIC_MATRIX_8_LSB_ADDR					((uint8_t)0x53)
#define BNO055_SIC_MATRIX_8_MSB_ADDR					((uint8_t)0x54)
#define BNO055_ACCEL_OFFSET_X_LSB_ADDR				((uint8_t)0x55)
#define BNO055_ACCEL_OFFSET_X_MSB_ADDR				((uint8_t)0x56)
#define BNO055_ACCEL_OFFSET_Y_LSB_ADDR				((uint8_t)0X57)
#define BNO055_ACCEL_OFFSET_Y_MSB_ADDR				((uint8_t)0x58)
#define BNO055_ACCEL_OFFSET_Z_LSB_ADDR				((uint8_t)0x59)
#define BNO055_ACCEL_OFFSET_Z_MSB_ADDR				((uint8_t)0x5A)
#define BNO055_MAG_OFFSET_X_LSB_ADDR					((uint8_t)0x5B)
#define BNO055_MAG_OFFSET_X_MSB_ADDR					((uint8_t)0x5C)
#define BNO055_MAG_OFFSET_Y_LSB_ADDR					((uint8_t)0x5D)
#define BNO055_MAG_OFFSET_Y_MSB_ADDR					((uint8_t)0x5E)
#define BNO055_MAG_OFFSET_Z_LSB_ADDR					((uint8_t)0x5F)
#define BNO055_MAG_OFFSET_Z_MSB_ADDR					((uint8_t)0x60)
#define BNO055_GYRO_OFFSET_X_LSB_ADDR					((uint8_t)0x61)
#define BNO055_GYRO_OFFSET_X_MSB_ADDR					((uint8_t)0x62)
#define BNO055_GYRO_OFFSET_Y_LSB_ADDR					((uint8_t)0x63)
#define BNO055_GYRO_OFFSET_Y_MSB_ADDR					((uint8_t)0x64)
#define BNO055_GYRO_OFFSET_Z_LSB_ADDR					((uint8_t)0x65)
#define BNO055_GYRO_OFFSET_Z_MSB_ADDR					((uint8_t)0x66)
#define	BNO055_ACCEL_RADIUS_LSB_ADDR					((uint8_t)0x67)
#define	BNO055_ACCEL_RADIUS_MSB_ADDR					((uint8_t)0x68)
#define	BNO055_MAG_RADIUS_LSB_ADDR						((uint8_t)0x69)
#define	BNO055_MAG_RADIUS_MSB_ADDR						((uint8_t)0x6A)


/*************************PAG 1 REGISTER DEFINITION*************************/

#define BNO055_ACCEL_CONFIG_ADDR							((uint8_t)0x08)
#define BNO055_MAG_CONFIG_ADDR								((uint8_t)0x09)
#define BNO055_GYRO_CONFIG_ADDR								((uint8_t)0x0A)
#define BNO055_GYRO_MODE_CONFIG_ADDR					((uint8_t)0x0B)
#define BNO055_ACCEL_SLEEP_CONFIG_ADDR				((uint8_t)0x0C)
#define BNO055_GYRO_SLEEP_CONFIG_ADDR					((uint8_t)0x0D)
#define BNO055_MAG_SLEEP_CONFIG_ADDR					((uint8_t)0x0E)
#define BNO055_INT_MASK_ADDR									((uint8_t)0x0F)
#define BNO055_INT_ADDR												((uint8_t)0x10)
#define BNO055_ACCEL_ANY_MOTION_THRES_ADDR		((uint8_t)0x11)
#define BNO055_ACCEL_INTR_SETTINGS_ADDR				((uint8_t)0x12)
#define BNO055_ACCEL_HIGH_G_DURN_ADDR					((uint8_t)0x13)
#define BNO055_ACCEL_HIGH_G_THRES_ADDR				((uint8_t)0x14)
#define BNO055_ACCEL_NO_MOTION_THRES_ADDR			((uint8_t)0x15)
#define BNO055_ACCEL_NO_MOTION_SET_ADDR				((uint8_t)0x16)
#define BNO055_GYRO_INTR_SETING_ADDR					((uint8_t)0x17)
#define BNO055_GYRO_HIGHRATE_X_SET_ADDR				((uint8_t)0x18)
#define BNO055_GYRO_DURN_X_ADDR								((uint8_t)0x19)
#define BNO055_GYRO_HIGHRATE_Y_SET_ADDR				((uint8_t)0x1A)
#define BNO055_GYRO_DURN_Y_ADDR								((uint8_t)0x1B)
#define BNO055_GYRO_HIGHRATE_Z_SET_ADDR				((uint8_t)0x1C)
#define BNO055_GYRO_DURN_Z_ADDR								((uint8_t)0x1D)
#define BNO055_GYRO_ANY_MOTION_THRES_ADDR			((uint8_t)0x1E)
#define BNO055_GYRO_ANY_MOTION_SET_ADDR				((uint8_t)0x1F)


/**************************OPERATION MODE SETTING***************************/

#define BNO055_OPERATION_MODE_CONFIG			    ((uint8_t)0x00)
#define BNO055_OPERATION_MODE_ACCONLY			    ((uint8_t)0x01)
#define BNO055_OPERATION_MODE_MAGONLY			    ((uint8_t)0x02)
#define BNO055_OPERATION_MODE_GYRONLY			    ((uint8_t)0x03)
#define BNO055_OPERATION_MODE_ACCMAG			    ((uint8_t)0x04)
#define BNO055_OPERATION_MODE_ACCGYRO			    ((uint8_t)0x05)
#define BNO055_OPERATION_MODE_MAGGYRO			    ((uint8_t)0x06)
#define BNO055_OPERATION_MODE_AMG				      ((uint8_t)0x07)
#define BNO055_OPERATION_MODE_IMUPLUS		    	((uint8_t)0x08)
#define BNO055_OPERATION_MODE_COMPASS			    ((uint8_t)0x09)
#define BNO055_OPERATION_MODE_M4G				      ((uint8_t)0x0A)
#define BNO055_OPERATION_MODE_NDOF_FMC_OFF		((uint8_t)0x0B)
#define BNO055_OPERATION_MODE_NDOF				    ((uint8_t)0x0C)


/********************************POWER MDOE*********************************/

#define BNO055_POWER_MODE_NORMAL	            ((uint8_t)0x00)
#define BNO055_POWER_MODE_LOWPOWER	          ((uint8_t)0x01)
#define BNO055_POWER_MODE_SUSPEND	            ((uint8_t)0x02)


/*******************************UNIT DEFINES********************************/

#define BNO055_ACCEL_UNIT_MSQ	                ((uint8_t)0x00)
#define BNO055_ACCEL_UNIT_MG	                ((uint8_t)0x01)
#define BNO055_GYRO_UNIT_DPS	                ((uint8_t)0x00)
#define BNO055_GYRO_UNIT_RPS	                ((uint8_t)0x01)
#define BNO055_EULER_UNIT_DEG	                ((uint8_t)0x00)
#define BNO055_EULER_UNIT_RAD	                ((uint8_t)0x01)
#define BNO055_TEMP_UNIT_CELSIUS		          ((uint8_t)0x00)
#define BNO055_TEMP_UNIT_FAHRENHEIT		        ((uint8_t)0x01)
#define BNO055_FUSION_DATA_WINDOWS            ((uint8_t)0x00)
#define BNO055_FUSION_DATA_ANDROID            ((uint8_t)0x01)


/****************************CONVERSION DEFINES*****************************/

#define	BNO055_ACCEL_DIV_MSQ	                (100.0f)
#define	BNO055_ACCEL_DIV_MG		                (1.0f)
#define BNO055_MAG_DIV_UT	                    (16.0f)
#define BNO055_GYRO_DIV_DPS		                (16.0f)
#define BNO055_GYRO_DIV_RPS		                (900.0f)
#define BNO055_EULER_DIV_DEG		              (16.0f)
#define BNO055_EULER_DIV_RAD		              (900.0f)
#define BNO055_TEMP_DIV_FAHRENHEIT	          (0.5f)
#define BNO055_TEMP_DIV_CELSIUS		            (1.0f)


/******************************CONFIG DEFINES*******************************/

#define BNO055_ACCEL_RANGE_2G		              ((uint8_t)0x00)
#define BNO055_ACCEL_RANGE_4G		              ((uint8_t)0x01)
#define BNO055_ACCEL_RANGE_8G		              ((uint8_t)0x02)
#define BNO055_ACCEL_RANGE_16G		            ((uint8_t)0x03)
#define BNO055_ACCEL_BW_7_81HZ		            ((uint8_t)0x00)
#define BNO055_ACCEL_BW_15_63HZ		            ((uint8_t)0x01)
#define BNO055_ACCEL_BW_31_25HZ		            ((uint8_t)0x02)
#define BNO055_ACCEL_BW_62_5HZ		            ((uint8_t)0x03)
#define BNO055_ACCEL_BW_125HZ		              ((uint8_t)0x04)
#define BNO055_ACCEL_BW_250HZ		              ((uint8_t)0x05)
#define BNO055_ACCEL_BW_500HZ		              ((uint8_t)0x06)
#define BNO055_ACCEL_BW_1000HZ		            ((uint8_t)0x07)
#define BNO055_ACCEL_NORMAL			              ((uint8_t)0x00)
#define BNO055_ACCEL_SUSPEND		              ((uint8_t)0x01)
#define BNO055_ACCEL_LOWPOWER_1		            ((uint8_t)0x02)
#define BNO055_ACCEL_STANDBY		              ((uint8_t)0x03)
#define BNO055_ACCEL_LOWPOWER_2		            ((uint8_t)0x04)
#define BNO055_ACCEL_DEEPSUSPEND	            ((uint8_t)0x05)
#define BNO055_GYRO_RANGE_2000DPS		          ((uint8_t)0x00)
#define BNO055_GYRO_RANGE_1000DPS		          ((uint8_t)0x01)
#define BNO055_GYRO_RANGE_500DPS		          ((uint8_t)0x02)
#define BNO055_GYRO_RANGE_250DPS		          ((uint8_t)0x03)
#define BNO055_GYRO_RANGE_125DPS		          ((uint8_t)0x04)
#define BNO055_GYRO_BW_523HZ	                ((uint8_t)0x00)
#define BNO055_GYRO_BW_230HZ	                ((uint8_t)0x01)
#define BNO055_GYRO_BW_116HZ	                ((uint8_t)0x02)
#define BNO055_GYRO_BW_47HZ		                ((uint8_t)0x03)
#define BNO055_GYRO_BW_23HZ		                ((uint8_t)0x04)
#define BNO055_GYRO_BW_12HZ		                ((uint8_t)0x05)
#define BNO055_GYRO_BW_64HZ		                ((uint8_t)0x06)
#define BNO055_GYRO_BW_32HZ		                ((uint8_t)0x07)
#define BNO055_GYRO_POWER_MODE_NORMAL				  ((uint8_t)0x00)
#define BNO055_GYRO_POWER_MODE_FASTPOWERUP		((uint8_t)0x01)
#define BNO055_GYRO_POWER_MODE_DEEPSUSPEND		((uint8_t)0x02)
#define BNO055_GYRO_POWER_MODE_SUSPEND				((uint8_t)0x03)
#define BNO055_GYRO_POWER_MODE_POWERSAVE	    ((uint8_t)0x04)
#define BNO055_MAG_DATA_OUTRATE_2HZ			      ((uint8_t)0x00)
#define BNO055_MAG_DATA_OUTRATE_6HZ			      ((uint8_t)0x01)
#define BNO055_MAG_DATA_OUTRATE_8HZ			      ((uint8_t)0x02)
#define BNO055_MAG_DATA_OUTRATE_10HZ		      ((uint8_t)0x03)
#define BNO055_MAG_DATA_OUTRATE_15HZ		      ((uint8_t)0x04)
#define BNO055_MAG_DATA_OUTRATE_20HZ		      ((uint8_t)0x05)
#define BNO055_MAG_DATA_OUTRATE_25HZ		      ((uint8_t)0x06)
#define BNO055_MAG_DATA_OUTRATE_30HZ		      ((uint8_t)0x07)
#define BNO055_MAG_OPERATION_MODE_LOWPOWER		((uint8_t)0x00)
#define BNO055_MAG_OPERATION_MODE_REGULAR			((uint8_t)0x01)
#define BNO055_MAG_OPERATION_MODE_ENHANCED	  ((uint8_t)0x02)
#define BNO055_MAG_OPERATION_MODE_ACCURACY		((uint8_t)0x03)
#define BNO055_MAG_POWER_MODE_NORMAL			    ((uint8_t)0x00)
#define BNO055_MAG_POWER_MODE_SLEEP				    ((uint8_t)0x01)
#define BNO055_MAG_POWER_MODE_SUSPEND			    ((uint8_t)0x02)
#define BNO055_MAG_POWER_MODE_FORCE_MODE		  ((uint8_t)0x03)
#define	BNO055_ACCEL_TEMP_EN	                ((uint8_t)0x00)
#define	BNO055_GYRO_TEMP_EN		                ((uint8_t)0x01)
#define	BNO055_MCU_TEMP_EN		                ((uint8_t)0x03)
#define BNO055_PAGE_ZERO		                  ((uint8_t)0x00)
#define BNO055_PAGE_ONE			                  ((uint8_t)0x01)
#define BNO055_USE_INTERNAL_OSCILLATOR        ((uint8_t)0x00)
#define BNO055_USE_EXTERNAL_CRYSTAL           ((uint8_t)0x80)


typedef struct
{
  uint8_t acceleration;
  uint8_t gyroscope;
  uint8_t euler;
  uint8_t temperature;
  uint8_t fusion_Format;
} BNO055_Unit_Config_t;

typedef struct
{
  uint8_t accelerometer_Range;
  uint8_t accelerometer_Bandwidth;
  uint8_t accelerometer_Mode;
  uint8_t magnetometer_Bandwidth;
  uint8_t magnetometer_Mode;
  uint8_t magnetometer_Power;
  uint8_t gyroscope_Range;
  uint8_t gyroscope_Bandwidth;
  uint8_t gyroscope_Mode;
} BNO055_Scale_Config_t;

typedef struct
{
  uint8_t magnetometer;
  uint8_t accelerometer;
  uint8_t gyroscope;
  uint8_t system;
} BNO055_Calibration_State_t;

typedef struct
{
  uint8_t accelerometer;
  uint8_t magnetometer;
  uint8_t gryroscope;
  uint8_t microcontroller;
} BNO055_Self_Test_Result_t;

typedef struct
{
  int16_t accelerometer[3];
  int16_t gyroscope[3];
  int16_t heading;
} BNO055_Imu_Raw_Data_t;

typedef struct
{
  float accelerometer[3];
  float gyroscope[3];
  float heading;
} BNO055_Imu_Converted_Data_t;

typedef struct
{
  float accelerometer[3];
  float magnetometer[3];
  float gyroscope[3];
  float euler[3];
  float quaterion[4];
  float liner_Acceleration[3];
  float gravitation_Acceleration[3];
  float temperature;
} BNO055_Imu_Converted_All_Data_t;


extern BNO055_Imu_Raw_Data_t raw_Data;
extern BNO055_Imu_Converted_Data_t converted_Data;
extern BNO055_Imu_Converted_All_Data_t converted_All_Data;
extern BNO055_Calibration_State_t calibration_State;
extern BNO055_Self_Test_Result_t self_Test_Result;
extern float accelerometer_Divisor;
extern float gyroscope_Divisor;
extern float euler_Divisor;
extern float temperature_Divisor;


extern void BNO055_Init(void);
extern inline void BNO055_Imu_Read_Data(void);
extern inline void BNO055_Imu_Read_All_Data(void);
extern inline void BNO055_Imu_Convert_Data(void);
extern inline void BNO055_Imu_Convert_All_Data(void);
extern inline void BNO055_Get_Calibration_Status(void);

static void BNO055_Deint(void);
static void BNO055_Set_External_Crystal_Use(uint8_t use_Crystal);
static void BNO055_Self_Test_Result(void);
static void BNO055_Unit_Config(BNO055_Unit_Config_t unit);
static void BNO055_Scale_Config(BNO055_Scale_Config_t scale);
static inline uint8_t BNO055_Read_Register(uint8_t reg);
static inline uint16_t BNO055_Read_Data(uint8_t reg);
static inline void BNO055_Read_Multiple_Register(uint8_t base_Reg, uint8_t *data, uint8_t n);
static inline void BNO055_Write_Register(const uint8_t reg, const uint8_t value);


#endif
