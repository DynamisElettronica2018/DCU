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
#define DIAG_IGN_1              ((uint16_t)130)
#define DIAG_IGN_2              ((uint16_t)132)
#define T_SCARICO_1             ((uint16_t)134)
#define T_SCARICO_2             ((uint16_t)138)


/************************** DAU ***************************/

/*DAU_FR_ID*/
#define LINEARE_FR              ((uint16_t)142)
#define LOAD_CELL_FR            ((uint16_t)148)
#define BPS_FRONT               ((uint16_t)154)

/*DAU_FL_ID*/
#define LINEARE_FL              ((uint16_t)160)
#define LOAD_CELL_FL            ((uint16_t)166)
#define BPS_REAR                ((uint16_t)172)
#define STEERING_WHEEL_ANGLE    ((uint16_t)178)

/*DAU_REAR_ID*/
#define LINEARE_RL              ((uint16_t)184)
#define LOAD_CELL_RL            ((uint16_t)190)
#define LINEARE_RR              ((uint16_t)196)
#define LOAD_CELL_RR            ((uint16_t)202)

/*DAU_FR_APPS_ID*/
#define APPS1                   ((uint16_t)208)
#define APPS2                   ((uint16_t)214)

/*IR_FL_ID*/
#define IR1_FL                  ((uint16_t)220)
#define IR2_FL                  ((uint16_t)226)
#define IR3_FL                  ((uint16_t)232)

/*IR_FR_ID*/
#define IR1_FR                  ((uint16_t)238)
#define IR2_FR                  ((uint16_t)244)
#define IR3_FR                  ((uint16_t)250)

/*IR_RL_ID*/
#define IR1_RL                  ((uint16_t)256)
#define IR2_RL                  ((uint16_t)262)
#define IR3_RL                  ((uint16_t)268)

/*IR_RR_ID*/
#define IR1_RR                  ((uint16_t)274)
#define IR2_RR                  ((uint16_t)280)
#define IR3_RR                  ((uint16_t)286)


/************************** IMU ***************************/

/*IMU_DATA_1_ID*/
#define ACC_X                   ((uint16_t)292)
#define ACC_Y                   ((uint16_t)300)
#define GYR_X                   ((uint16_t)308)
#define GYR_Z                   ((uint16_t)316)

/*IMU_DATA_2_ID*/
#define HEADING                 ((uint16_t)324)
#define ACC_Z                   ((uint16_t)328)
#define GYR_Y                   ((uint16_t)336)

/*IMU_DATA_3_ID*/
#define GPS_X                   ((uint16_t)344)
#define GPS_Y                   ((uint16_t)350)
#define VELOCITY                ((uint16_t)356)


/************************** EBB ***************************/

/*EBB_BIAS_ID*/
#define BIAS_POSITION           ((uint16_t)363)


/************************** IMU DCU ***************************/

#define DCU_ACC_X               ((uint16_t)369)
#define DCU_ACC_Y               ((uint16_t)377)
#define DCU_ACC_Z               ((uint16_t)385)
#define DCU_GYR_X               ((uint16_t)393)
#define DCU_GYR_Y               ((uint16_t)401)
#define DCU_GYR_Z               ((uint16_t)409)
#define DCU_HEADING             ((uint16_t)417)


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

#define SEP_EFI_START           ((uint16_t)7)
#define SEP_EFI_HALL_12         ((uint16_t)13)
#define SEP_EFI_HALL_23         ((uint16_t)19)
#define SEP_EFI_HALL_34         ((uint16_t)25)
#define SEP_EFI_HALL_H20        ((uint16_t)31)
#define SEP_EFI_H20_12          ((uint16_t)35)
#define SEP_EFI_H20_23          ((uint16_t)39)
#define SEP_EFI_H20_34          ((uint16_t)43)
#define SEP_EFI_H20_OIL         ((uint16_t)47)
#define SEP_EFI_OIL_12          ((uint16_t)51)
#define SEP_EFI_OIL_23          ((uint16_t)55)
#define SEP_EFI_OIL_34          ((uint16_t)59)
#define SEP_EFI_OIL_GEAR        ((uint16_t)64)
#define SEP_EFI_GEAR_12         ((uint16_t)66)
#define SEP_EFI_GEAR_23         ((uint16_t)72)
#define SEP_EFI_GEAR_34         ((uint16_t)76)
#define SEP_EFI_GEAR_TRACTION   ((uint16_t)80)
#define SEP_EFI_TRACTION_12     ((uint16_t)86)
#define SEP_EFI_TRACTION_23     ((uint16_t)92)
#define SEP_EFI_TRACTION_34     ((uint16_t)98)
#define SEP_EFI_TRACTION_FAN    ((uint16_t)100)
#define SEP_EFI_FAN_12          ((uint16_t)102)
#define SEP_EFI_FAN_23          ((uint16_t)106)
#define SEP_EFI_FAN_34          ((uint16_t)108)
#define SEP_EFI_FAN_LAMBDA      ((uint16_t)114)
#define SEP_EFI_LAMBDA_12       ((uint16_t)120)
#define SEP_EFI_LAMBDA_23       ((uint16_t)126)
#define SEP_EFI_LAMBDA_34       ((uint16_t)129)
#define SEP_EFI_LAMBDA_DIAG     ((uint16_t)131)
#define SEP_EFI_DIAG_12         ((uint16_t)133)
#define SEP_EFI_DIAG_23         ((uint16_t)137)
#define SEP_EFI_DIAG_34         ((uint16_t)141)
#define SEP_DAU_START           ((uint16_t)147)
#define SEP_DAU_FR_12           ((uint16_t)153)
#define SEP_DAU_FR_23           ((uint16_t)159)
#define SEP_DAU_FR_FL           ((uint16_t)165)
#define SEP_DAU_FL_12           ((uint16_t)171)
#define SEP_DAU_FL_23           ((uint16_t)177)
#define SEP_DAU_FL_34           ((uint16_t)183)
#define SEP_DAU_FL_RR           ((uint16_t)189)
#define SEP_DAU_RR_12           ((uint16_t)195)
#define SEP_DAU_RR_23           ((uint16_t)201)
#define SEP_DAU_RR_34           ((uint16_t)207)
#define SEP_DAU_RR_APPS         ((uint16_t)213)
#define SEP_DAU_APPS_12         ((uint16_t)219)
#define SEP_DAU_APPS_IR_FL      ((uint16_t)225)
#define SEP_DAU_IR_FL_12        ((uint16_t)231)
#define SEP_DAU_IR_FL_23        ((uint16_t)237)
#define SEP_DAU_IR_FL_FR        ((uint16_t)243)
#define SEP_DAU_IR_FR_12        ((uint16_t)249)
#define SEP_DAU_IR_FR_23        ((uint16_t)255)
#define SEP_DAU_IR_FR_RL        ((uint16_t)261)
#define SEP_DAU_IR_RL_12        ((uint16_t)267)
#define SEP_DAU_IR_RL_23        ((uint16_t)273)
#define SEP_DAU_IR_RL_RR        ((uint16_t)279)
#define SEP_DAU_IR_RR_12        ((uint16_t)285)
#define SEP_DAU_IR_RR_23        ((uint16_t)291)
#define SEP_IMU_START           ((uint16_t)299)
#define SEP_IMU_DATA1_12        ((uint16_t)307)
#define SEP_IMU_DATA1_23        ((uint16_t)315)
#define SEP_IMU_DATA1_34        ((uint16_t)323)
#define SEP_IMU_DATA_1_2        ((uint16_t)327)
#define SEP_IMU_DATA2_12        ((uint16_t)335)
#define SEP_IMU_DATA2_23        ((uint16_t)343)
#define SEP_IMU_DATA_2_3        ((uint16_t)349)
#define SEP_IMU_DATA3_12        ((uint16_t)355)
#define SEP_IMU_DATA3_23        ((uint16_t)362)
#define SEP_EBB_START           ((uint16_t)368)
#define SEP_EBB_IMU_DCU         ((uint16_t)376)
#define SEP_IMU_DCU_ACC_XY      ((uint16_t)384)
#define SEP_IMU_DCU_ACC_YZ      ((uint16_t)392)
#define SEP_IMU_DCU_ACC_GYR     ((uint16_t)400)
#define SEP_IMU_DCU_GYR_XY      ((uint16_t)408)
#define SEP_IMU_DCU_GYR_YZ      ((uint16_t)416)
#define SEP_END_FILE            ((uint16_t)420)
#define END_LINE_POSITION       ((uint16_t)511)

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

#define AVGBUF_WHEELSPEED1			0
#define AVGBUF_WHEELSPEED2			1
#define AVGBUF_WHEELSPEED3			2
#define AVGBUF_WHEELSPEED4			3
#define AVGBUF_BRAKEPRESSF			4
#define AVGBUF_BRAKEPRESSR			5
#define AVGBUF_TPS							6
#define AVGBUF_SW								7
#define AVGBUF_ACCX							8
#define AVGBUF_ACCY							9
#define AVGBUF_ACCZ							10
#define AVGBUF_GYROZ						11
#define AVGBUF_DCU_ACCX					12
#define AVGBUF_DCU_ACCY					13
#define AVGBUF_DCU_ACCZ					14
#define AVGBUF_DCU_GYROZ				15
#define AVGBUF_VHSPEED					16
#define AVGBUF_LINEARFL					17
#define AVGBUF_LINEARFR					18
#define AVGBUF_LINEARRR					19
#define AVGBUF_LINEARRL					20


extern uint8_t dcu_State_Packet[BUFFER_STATE_LEN];
extern uint8_t dcu_Debug_Packet[BUFFER_DEBUG_LEN];
extern uint8_t block_Buffer[BUFFER_LEN][BUFFER_BLOCK_LEN];
extern volatile uint8_t buffer_writePointer;
extern volatile uint8_t buffer_readPointer;
extern volatile uint8_t buffer_telemetryPointer;

extern volatile uint32_t USB_Timestamp;
extern volatile uint8_t start_Acquisition_Request;


extern void initialize_Data(void);
extern inline void makeDataAvg(void);
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
