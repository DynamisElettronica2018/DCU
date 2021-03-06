#ifndef __DATA_H__
#define __DATA_H__

#include <stdint.h>
#include "user_defines.h"
#include "bno055.h"


/*****************LOAD CELLS OFFSET************************/

#define LOAD_CELL_FL_OFFSET     (0.0f)
#define LOAD_CELL_FR_OFFSET     (0.0f)
#define LOAD_CELL_RL_OFFSET     (0.0f)
#define LOAD_CELL_RR_OFFSET     (0.0f)


/***************DAU VOLTAGE REFERENCES*********************/

#define DAU_FL_SAMPLE_TO_VOLT   (5.052f / 4095.0f)
#define DAU_FR_SAMPLE_TO_VOLT   (5.116f / 4095.0f)
#define DAU_REAR_SAMPLE_TO_VOLT (5.118f / 4095.0f)


/********************CSV SYMBOLS***************************/

#define SEPARATOR               ((uint8_t)';')
#define END_LINE                ((uint8_t)'\n')
#define DECIMAL_SEPARATOR       ((uint8_t)'.')


/************************** EFI ***************************/

/*EFI_HALL_ID*/
#define HALL_EFFECT_FR          ((uint16_t)8)
#define HALL_EFFECT_FL          ((uint16_t)14)
#define HALL_EFFECT_RR          ((uint16_t)20)
#define HALL_EFFECT_RL          ((uint16_t)26)

/*EFI_WATER_TEMPERATURE_ID*/
#define T_H20_SX_IN             ((uint16_t)32)
#define T_H20_SX_OUT            ((uint16_t)36)
#define T_H20_DX_IN             ((uint16_t)40)
#define T_H20_DX_OUT            ((uint16_t)44)

/*EFI_OIL_T_ENGINE_BAT_ID*/
#define T_OIL_IN                ((uint16_t) 48)
#define T_OIL_OUT               ((uint16_t) 52)
#define T_H20_ENGINE            ((uint16_t) 56)
#define BATTERY_VOLTAGE         ((uint16_t) 60)

/*EFI_GEAR_RPM_TPS_APPS_ID*/
#define GEAR                    ((uint16_t)65)
#define RPM                     ((uint16_t)67)
#define TPS_1                   ((uint16_t)73)
#define PEDAL_POSITION_AVG      ((uint16_t)77)

/*EFI_TRACTION_CONTROL_ID*/
#define VH_SPEED                ((uint16_t)81)
#define SLIP_TARGET             ((uint16_t)87)
#define SLIP                    ((uint16_t)93)

/*EFI_FUEL_FAN_H2O_LAUNCH_ID*/
#define FUEL_PUMP               ((uint16_t)99)
#define FAN                     ((uint16_t)101)
#define H20_PUMP_DUTY_CYCLE     ((uint16_t)103)
#define LAUNCH_CONTROL_ACTIVE   ((uint16_t)107)

/*EFI_PRESSURES_LAMBDA_SMOT_ID*/
#define FUEL_PRESSURE           ((uint16_t)109)
#define OIL_PRESSURE            ((uint16_t)115)
#define LAMBDA                  ((uint16_t)121)
#define FLAG_SMOT               ((uint16_t)127)

/*EFI_DIAG_IGN_EXHAUST_ID*/
#define DIAG_IGN_1              ((uint16_t)133)
#define DIAG_IGN_2              ((uint16_t)135)
#define T_SCARICO_1             ((uint16_t)137)
#define T_SCARICO_2             ((uint16_t)141)


/************************** DAU ***************************/

/*DAU_FR_ID*/
#define LINEARE_FR              ((uint16_t)145)
#define LOAD_CELL_FR            ((uint16_t)151)
#define BPS_FRONT               ((uint16_t)157)

/*DAU_FL_ID*/
#define LINEARE_FL              ((uint16_t)163)
#define LOAD_CELL_FL            ((uint16_t)169)
#define BPS_REAR                ((uint16_t)175)
#define STEERING_WHEEL_ANGLE    ((uint16_t)181)

/*DAU_REAR_ID*/
#define LINEARE_RL              ((uint16_t)187)
#define LOAD_CELL_RL            ((uint16_t)193)
#define LINEARE_RR              ((uint16_t)199)
#define LOAD_CELL_RR            ((uint16_t)205)

/*DAU_FR_APPS_ID*/
#define APPS1                   ((uint16_t)211)
#define APPS2                   ((uint16_t)217)

/*IR_FL_ID*/
#define IR1_FL                  ((uint16_t)223)
#define IR2_FL                  ((uint16_t)229)
#define IR3_FL                  ((uint16_t)235)

/*IR_FR_ID*/
#define IR1_FR                  ((uint16_t)241)
#define IR2_FR                  ((uint16_t)247)
#define IR3_FR                  ((uint16_t)253)

/*IR_RL_ID*/
#define IR1_RL                  ((uint16_t)259)
#define IR2_RL                  ((uint16_t)265)
#define IR3_RL                  ((uint16_t)271)

/*IR_RR_ID*/
#define IR1_RR                  ((uint16_t)277)
#define IR2_RR                  ((uint16_t)283)
#define IR3_RR                  ((uint16_t)289)

/************************** IMU ***************************/

/*IMU_DATA_1_ID*/
#define ACC_X                   ((uint16_t)295)
#define ACC_Y                   ((uint16_t)303)
#define GYR_X                   ((uint16_t)311)
#define GYR_Z                   ((uint16_t)319)

/*IMU_DATA_2_ID*/
#define HEADING                 ((uint16_t)327)
#define ACC_Z                   ((uint16_t)331)
#define GYR_Y                   ((uint16_t)339)

/*IMU_DATA_3_ID*/
#define GPS_X                   ((uint16_t)347)
#define GPS_Y                   ((uint16_t)353)
#define VELOCITY                ((uint16_t)359)


/************************** EBB ***************************/

/*EBB_BIAS_ID*/
#define BIAS_POSITION           ((uint16_t)366)


/************************** IMU DCU ***************************/

#define DCU_ACC_X               ((uint16_t)372)
#define DCU_ACC_Y               ((uint16_t)380)
#define DCU_ACC_Z               ((uint16_t)388)
#define DCU_GYR_X               ((uint16_t)396)
#define DCU_GYR_Y               ((uint16_t)404)
#define DCU_GYR_Z               ((uint16_t)412)
#define DCU_HEADING             ((uint16_t)420)

/************************** DEBUG *************************/

#define GCU_TEMP             		((uint16_t)424)
#define FANS_CURRENT         		((uint16_t)428)
#define H2O_PUMP_CURRENT     		((uint16_t)434)
#define FUEL_PUMP_CURRENT    		((uint16_t)440)
#define GEARMOTOR_CURRENT				((uint16_t)446)
#define CLUTCH_CURRENT       		((uint16_t)452)
#define DRS_CURRENT          		((uint16_t)458)
#define DCU_TEMP             		((uint16_t)464)
#define DCU_CURRENT          		((uint16_t)468)
#define DAU_FR_TEMP          		((uint16_t)473)
#define DAU_FR_CURRENT       		((uint16_t)477)
#define DAU_FL_TEMP          		((uint16_t)482)
#define DAU_FL_CURRENT       		((uint16_t)486)
#define DAU_REAR_TEMP        		((uint16_t)491)
#define DAU_REAR_CURRENT     		((uint16_t)495)
#define SW_TEMP              		((uint16_t)500)
//#define EBB_TEMP             	
//#define EBB_BOARD_CURRENT    	
//#define EBB_MOTOR_CURRENT    

/************************** DEBUG *************************/

#define POSITION_DCU_TEMP             ((uint16_t)0)
#define POSITION_DCU_CURRENT          ((uint16_t)4)
#define POSITION_DAU_FR_TEMP          ((uint16_t)9)
#define POSITION_DAU_FR_CURRENT       ((uint16_t)13)
#define POSITION_DAU_FL_TEMP          ((uint16_t)18)
#define POSITION_DAU_FL_CURRENT       ((uint16_t)22)
#define POSITION_DAU_REAR_TEMP        ((uint16_t)27)
#define POSITION_DAU_REAR_CURRENT     ((uint16_t)31)
#define POSITION_SW_TEMP              ((uint16_t)36)
#define POSITION_EBB_TEMP             ((uint16_t)40)
#define POSITION_EBB_BOARD_CURRENT    ((uint16_t)44)
#define POSITION_EBB_MOTOR_CURRENT    ((uint16_t)49)
#define POSITION_GCU_TEMP             ((uint16_t)54)
#define POSITION_FANS_CURRENT         ((uint16_t)58)
#define POSITION_H2O_PUMP_CURRENT     ((uint16_t)64)
#define POSITION_FUEL_PUMP_CURRENT    ((uint16_t)70)
#define POSITION_GEARMOTOR_CURRENT    ((uint16_t)76)
#define POSITION_CLUTCH_CURRENT       ((uint16_t)82)
#define POSITION_DRS_CURRENT          ((uint16_t)88)


/******************* SEPARATORS POSITIONS ********************/

#define START_SEPARATOR_POSITION	((uint16_t)7)
#define END_SEPARATOR_POSITION  	((uint16_t)503)
#define END_LINE_POSITION       	((uint16_t)511)

#define SEP_DCU_CURRENT         (POSITION_DCU_CURRENT - 1)
#define SEP_DAU_FR_TEMP         (POSITION_DAU_FR_TEMP - 1)
#define SEP_DAU_FR_CURRENT      (POSITION_DAU_FR_CURRENT - 1)
#define SEP_DAU_FL_TEMP         (POSITION_DAU_FL_TEMP - 1)
#define SEP_DAU_FL_CURRENT      (POSITION_DAU_FL_CURRENT - 1)
#define SEP_DAU_REAR_TEMP       (POSITION_DAU_REAR_TEMP - 1)
#define SEP_DAU_REAR_CURRENT    (POSITION_DAU_REAR_CURRENT - 1)
#define SEP_SW_TEMP             (POSITION_SW_TEMP - 1)
#define SEP_EBB_TEMP            (POSITION_EBB_TEMP - 1)
#define SEP_EBB_BOARD_CURRENT   (POSITION_EBB_BOARD_CURRENT - 1)
#define SEP_EBB_MOTOR_CURRENT   (POSITION_EBB_MOTOR_CURRENT - 1)
#define SEP_GCU_TEMP            (POSITION_GCU_TEMP - 1)
#define SEP_FANS_CURRENT        (POSITION_FANS_CURRENT - 1)
#define SEP_H2O_PUMP_CURRENT    (POSITION_H2O_PUMP_CURRENT - 1)
#define SEP_FUEL_PUMP_CURRENT   (POSITION_FUEL_PUMP_CURRENT - 1)
#define SEP_GEARMOTOR_CURRENT   (POSITION_GEARMOTOR_CURRENT - 1)
#define SEP_CLUTCH_CURRENT      (POSITION_CLUTCH_CURRENT - 1)
#define SEP_DRS_CURRENT         (POSITION_DRS_CURRENT - 1)
#define DEBUG_PACKET_END_LINE   (POSITION_DRS_CURRENT + 5)

#define AVGBUF_WHEELSPEED1			((uint8_t)0)
#define AVGBUF_WHEELSPEED2			((uint8_t)1)
#define AVGBUF_WHEELSPEED3			((uint8_t)2)
#define AVGBUF_WHEELSPEED4			((uint8_t)3)
#define AVGBUF_BRAKEPRESSF			((uint8_t)4)
#define AVGBUF_BRAKEPRESSR			((uint8_t)5)
#define AVGBUF_TPS							((uint8_t)6)
#define AVGBUF_SW								((uint8_t)7)
#define AVGBUF_ACCX							((uint8_t)8)
#define AVGBUF_ACCY							((uint8_t)9)
#define AVGBUF_ACCZ							((uint8_t)10)
#define AVGBUF_GYROZ						((uint8_t)11)
#define AVGBUF_DCU_ACCX					((uint8_t)12)
#define AVGBUF_DCU_ACCY					((uint8_t)13)
#define AVGBUF_DCU_ACCZ					((uint8_t)14)
#define AVGBUF_DCU_GYROZ				((uint8_t)15)
#define AVGBUF_VHSPEED					((uint8_t)16)
#define AVGBUF_LINEARFL					((uint8_t)17)
#define AVGBUF_LINEARFR					((uint8_t)18)
#define AVGBUF_LINEARRR					((uint8_t)19)
#define AVGBUF_LINEARRL					((uint8_t)20)


extern uint8_t dcu_State_Packet[BUFFER_STATE_LEN];
extern uint8_t dcu_Debug_Packet[BUFFER_DEBUG_LEN];
extern uint8_t block_Buffer[BUFFER_LEN][BUFFER_BLOCK_LEN];
extern uint8_t header_Packet_Buffer[];
extern volatile uint8_t buffer_Write_Pointer;
extern volatile uint8_t buffer_Read_Pointer;
extern volatile uint8_t buffer_Telemetry_Pointer;
extern volatile uint32_t USB_Timestamp;
extern volatile uint8_t start_Acquisition_Request;


extern void initialize_Data(void);
extern inline void prepare_Next_Buffer_Block(uint8_t previous_Pointer, uint8_t next_Pointer);
extern inline void make_Data_Average(void);
extern inline void data_Conversion(uint16_t ID, uint8_t payload[8]);
extern inline void Imu_Dcu_Conversion_To_Buffer(void);
extern inline void debug_Data_Conversion(uint16_t ID, uint8_t payload[8]);

static inline float GYR_Conversion(uint16_t input);
static inline float HEADING_Conversion(uint16_t input);
static inline float temperature_Efi_Conversion(uint16_t input);
static inline float T_SCARICO_Conversion(uint16_t input);
static inline float T_OIL_IN_Conversion(uint16_t input);
static inline float T_H20_ENGINE_Conversion(uint16_t input);
static inline float BATT_VOLTAGE_Conversion(uint16_t input);
static inline float TPS_1_Conversion(uint16_t input);
static inline float PEDAL_POS_AVG_Conversion(uint16_t input);
static inline float LOAD_CELL_FL_Conversion(uint16_t input);
static inline float LOAD_CELL_FR_Conversion(uint16_t input);
static inline float LOAD_CELL_RL_Conversion(uint16_t input);
static inline float LOAD_CELL_RR_Conversion(uint16_t input);


#endif
