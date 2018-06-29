#include <stdio.h>
#include "data.h"
#include "bno055.h"
#include "string_utility.h"
#include "user_defines.h"
#include "can_id_Defines.h"
#include "telemetry_command.h"


uint8_t dcu_State_Packet[BUFFER_STATE_LEN];
uint8_t dcu_Debug_Packet[BUFFER_DEBUG_LEN];
//uint8_t *buffer_Block_Write = NULL;
//uint8_t *buffer_Block = NULL;
volatile uint32_t USB_Timestamp = 0;
volatile uint8_t start_Acquisition_Request = 0;

//static uint8_t buffer_Block_1[BUFFER_BLOCK_LEN];
//static uint8_t buffer_Block_2[BUFFER_BLOCK_LEN];
uint8_t block_Buffer[BUFFER_LEN][BUFFER_BLOCK_LEN];
volatile uint8_t buffer_writePointer = 0;
volatile uint8_t buffer_readPointer = 0;
volatile uint8_t buffer_telemetryPointer = 0;

static float fTemp = 0.0;
static int16_t iTemp = 0;
static uint16_t uTemp = 0;
static uint16_t data0 = 0;
static uint16_t data1 = 0;
static uint16_t data2 = 0;
static uint16_t data3 = 0;
static uint16_t data4 = 0;
static uint16_t data5 = 0;
static uint16_t data6 = 0;
static uint16_t data7 = 0;

static float avg_Buffer[21][11];
static uint8_t avg_Buffer_Pointers[21];


// Funzione che inserisce i separatori di canali nel buffer.
// Da chiamare una sola volta prima del while, per inizializzare una riga del file CSV.
extern void initialize_Data(void)
{    
	for (uint8_t i = 0; i < 21; i++)
	{
		for(uint8_t j = 0; j < 11; j++)
		{
			avg_Buffer[i][j] = 0;
		}
		avg_Buffer_Pointers[i] = 0;
	}
	
	for(uint8_t i = 0; i < BUFFER_LEN; i++)
	{
		for(uint16_t j = 0; j < BUFFER_BLOCK_LEN; j++)
		{
			block_Buffer[i][j] = '0';
		}
		
		block_Buffer[i][SEP_EFI_START] = SEPARATOR;
		block_Buffer[i][SEP_EFI_HALL_12] = SEPARATOR;
		block_Buffer[i][SEP_EFI_HALL_23] = SEPARATOR;
		block_Buffer[i][SEP_EFI_HALL_34] = SEPARATOR;
		block_Buffer[i][SEP_EFI_HALL_H20] = SEPARATOR;
		block_Buffer[i][SEP_EFI_H20_12] = SEPARATOR;
		block_Buffer[i][SEP_EFI_H20_23] = SEPARATOR;
		block_Buffer[i][SEP_EFI_H20_34] = SEPARATOR;
		block_Buffer[i][SEP_EFI_H20_OIL] = SEPARATOR;
		block_Buffer[i][SEP_EFI_OIL_12] = SEPARATOR;
		block_Buffer[i][SEP_EFI_OIL_23] = SEPARATOR;
		block_Buffer[i][SEP_EFI_OIL_34] = SEPARATOR;
		block_Buffer[i][SEP_EFI_OIL_GEAR] = SEPARATOR;
		block_Buffer[i][SEP_EFI_GEAR_12] = SEPARATOR;
		block_Buffer[i][SEP_EFI_GEAR_23] = SEPARATOR;
		block_Buffer[i][SEP_EFI_GEAR_34] = SEPARATOR;
		block_Buffer[i][SEP_EFI_GEAR_TRACTION] = SEPARATOR;
		block_Buffer[i][SEP_EFI_TRACTION_12] = SEPARATOR;
		block_Buffer[i][SEP_EFI_TRACTION_23] = SEPARATOR;
		block_Buffer[i][SEP_EFI_TRACTION_34] = SEPARATOR;
		block_Buffer[i][SEP_EFI_TRACTION_FAN] = SEPARATOR;
		block_Buffer[i][SEP_EFI_FAN_12] = SEPARATOR;
		block_Buffer[i][SEP_EFI_FAN_23] = SEPARATOR;
		block_Buffer[i][SEP_EFI_FAN_34] = SEPARATOR;
		block_Buffer[i][SEP_EFI_FAN_LAMBDA] = SEPARATOR;
		block_Buffer[i][SEP_EFI_LAMBDA_12] = SEPARATOR;
		block_Buffer[i][SEP_EFI_LAMBDA_23] = SEPARATOR;
		block_Buffer[i][SEP_EFI_LAMBDA_34] = SEPARATOR;
		block_Buffer[i][SEP_EFI_LAMBDA_DIAG] = SEPARATOR;
		block_Buffer[i][SEP_EFI_DIAG_12] = SEPARATOR;
		block_Buffer[i][SEP_EFI_DIAG_23] = SEPARATOR;
		block_Buffer[i][SEP_EFI_DIAG_34] = SEPARATOR;
		block_Buffer[i][SEP_DAU_START] = SEPARATOR;
		block_Buffer[i][SEP_DAU_FR_12] = SEPARATOR;
		block_Buffer[i][SEP_DAU_FR_23] = SEPARATOR;
		block_Buffer[i][SEP_DAU_FR_FL] = SEPARATOR;
		block_Buffer[i][SEP_DAU_FL_12] = SEPARATOR;
		block_Buffer[i][SEP_DAU_FL_23] = SEPARATOR;
		block_Buffer[i][SEP_DAU_FL_34] = SEPARATOR;
		block_Buffer[i][SEP_DAU_FL_RR] = SEPARATOR;
		block_Buffer[i][SEP_DAU_RR_12] = SEPARATOR;
		block_Buffer[i][SEP_DAU_RR_23] = SEPARATOR;
		block_Buffer[i][SEP_DAU_RR_34] = SEPARATOR;
		block_Buffer[i][SEP_DAU_RR_APPS] = SEPARATOR;
		block_Buffer[i][SEP_DAU_APPS_12] = SEPARATOR;
		block_Buffer[i][SEP_DAU_APPS_IR_FL] = SEPARATOR;
		block_Buffer[i][SEP_DAU_IR_FL_12] = SEPARATOR;
		block_Buffer[i][SEP_DAU_IR_FL_23] = SEPARATOR;
		block_Buffer[i][SEP_DAU_IR_FL_FR] = SEPARATOR;
		block_Buffer[i][SEP_DAU_IR_FR_12] = SEPARATOR;
		block_Buffer[i][SEP_DAU_IR_FR_23] = SEPARATOR;
		block_Buffer[i][SEP_DAU_IR_FR_RL] = SEPARATOR;
		block_Buffer[i][SEP_DAU_IR_RL_12] = SEPARATOR;
		block_Buffer[i][SEP_DAU_IR_RL_23] = SEPARATOR;
		block_Buffer[i][SEP_DAU_IR_RL_RR] = SEPARATOR;
		block_Buffer[i][SEP_DAU_IR_RR_12] = SEPARATOR;
		block_Buffer[i][SEP_DAU_IR_RR_23] = SEPARATOR;
		block_Buffer[i][SEP_IMU_START] = SEPARATOR;
		block_Buffer[i][SEP_IMU_DATA1_12] = SEPARATOR;
		block_Buffer[i][SEP_IMU_DATA1_23] = SEPARATOR;
		block_Buffer[i][SEP_IMU_DATA1_34] = SEPARATOR;
		block_Buffer[i][SEP_IMU_DATA_1_2] = SEPARATOR;
		block_Buffer[i][SEP_IMU_DATA2_12] = SEPARATOR;
		block_Buffer[i][SEP_IMU_DATA2_23] = SEPARATOR;
		block_Buffer[i][SEP_IMU_DATA_2_3] = SEPARATOR;
		block_Buffer[i][SEP_IMU_DATA3_12] = SEPARATOR;
		block_Buffer[i][SEP_IMU_DATA3_23] = SEPARATOR;
		block_Buffer[i][SEP_EBB_START] = SEPARATOR;
		block_Buffer[i][SEP_EBB_IMU_DCU] = SEPARATOR;
		block_Buffer[i][SEP_IMU_DCU_ACC_XY] = SEPARATOR;
		block_Buffer[i][SEP_IMU_DCU_ACC_YZ] = SEPARATOR;
		block_Buffer[i][SEP_IMU_DCU_ACC_GYR] = SEPARATOR;
		block_Buffer[i][SEP_IMU_DCU_GYR_XY] = SEPARATOR;
		block_Buffer[i][SEP_IMU_DCU_GYR_YZ] = SEPARATOR;
		block_Buffer[i][SEP_END_FILE] = SEPARATOR;
		block_Buffer[i][END_LINE_POSITION] = END_LINE;
	}
	
  for(uint16_t i = 0; i < BUFFER_DEBUG_LEN ; i++)
  {
    dcu_Debug_Packet[i] = '0';
  }
	
	// Set all separators between channels for dcu_Debug_Packet.
  dcu_Debug_Packet[SEP_DCU_CURRENT] = SEPARATOR;
  dcu_Debug_Packet[SEP_DAU_FR_TEMP] = SEPARATOR;
  dcu_Debug_Packet[SEP_DAU_FR_CURRENT] = SEPARATOR;
  dcu_Debug_Packet[SEP_DAU_FL_TEMP] = SEPARATOR;
  dcu_Debug_Packet[SEP_DAU_FL_CURRENT] = SEPARATOR;
  dcu_Debug_Packet[SEP_DAU_REAR_TEMP] = SEPARATOR;
  dcu_Debug_Packet[SEP_DAU_REAR_CURRENT] = SEPARATOR;
  dcu_Debug_Packet[SEP_SW_TEMP] = SEPARATOR;
  dcu_Debug_Packet[SEP_EBB_TEMP] = SEPARATOR;
  dcu_Debug_Packet[SEP_EBB_BOARD_CURRENT] = SEPARATOR;
  dcu_Debug_Packet[SEP_EBB_MOTOR_CURRENT] = SEPARATOR;
  dcu_Debug_Packet[SEP_GCU_TEMP] = SEPARATOR;
  dcu_Debug_Packet[SEP_FANS_CURRENT] = SEPARATOR;
  dcu_Debug_Packet[SEP_H2O_PUMP_CURRENT] = SEPARATOR;
  dcu_Debug_Packet[SEP_FUEL_PUMP_CURRENT] = SEPARATOR;
  dcu_Debug_Packet[SEP_GEARMOTOR_CURRENT] = SEPARATOR;
  dcu_Debug_Packet[SEP_CLUTCH_CURRENT] = SEPARATOR;
  dcu_Debug_Packet[SEP_DRS_CURRENT] = SEPARATOR;
  dcu_Debug_Packet[DEBUG_PACKET_END_LINE] = END_LINE;
  
  // Set all separators between channels for dcu_State_Packet.
  dcu_State_Packet[0] = 'M';
  dcu_State_Packet[DCU_STATE_PACKET_USB_PRESENT] = UDP_DCU_STATE_ERROR;
  dcu_State_Packet[DCU_STATE_PACKET_USB_READY] = UDP_DCU_STATE_ERROR;
  dcu_State_Packet[DCU_STATE_PACKET_ACQUISITION_ON] = UDP_DCU_STATE_ERROR;
  dcu_State_Packet[DCU_STATE_PACKET_TELEMETRY_ON] = UDP_DCU_STATE_ERROR;
  dcu_State_Packet[DCU_STATE_PACKET_SD_PRESENT] = UDP_DCU_STATE_ERROR;
  dcu_State_Packet[DCU_STATE_PACKET_SD_READY] = UDP_DCU_STATE_ERROR;
  dcu_State_Packet[BUFFER_STATE_LEN - 1] = '\0';
}



extern inline void makeDataAvg(void)
{
	for(uint8_t j = 0; j < 10; j++)
	{
		avg_Buffer[AVGBUF_WHEELSPEED1][10] += avg_Buffer[AVGBUF_WHEELSPEED1][j];
		avg_Buffer[AVGBUF_WHEELSPEED2][10] += avg_Buffer[AVGBUF_WHEELSPEED2][j];
		avg_Buffer[AVGBUF_WHEELSPEED3][10] += avg_Buffer[AVGBUF_WHEELSPEED3][j];
		avg_Buffer[AVGBUF_WHEELSPEED4][10] += avg_Buffer[AVGBUF_WHEELSPEED4][j];
		
		avg_Buffer[AVGBUF_BRAKEPRESSF][10] += avg_Buffer[AVGBUF_BRAKEPRESSF][j];
		avg_Buffer[AVGBUF_BRAKEPRESSR][10] += avg_Buffer[AVGBUF_BRAKEPRESSR][j];
		
		avg_Buffer[AVGBUF_TPS][10] += avg_Buffer[AVGBUF_TPS][j];
		avg_Buffer[AVGBUF_SW][10] += avg_Buffer[AVGBUF_SW][j];
		
		avg_Buffer[AVGBUF_ACCX][10] += avg_Buffer[AVGBUF_ACCX][j];
		avg_Buffer[AVGBUF_ACCY][10] += avg_Buffer[AVGBUF_ACCY][j];
		avg_Buffer[AVGBUF_ACCZ][10] += avg_Buffer[AVGBUF_ACCZ][j];
		avg_Buffer[AVGBUF_GYROZ][10] += avg_Buffer[AVGBUF_GYROZ][j];
		
		avg_Buffer[AVGBUF_DCU_ACCX][10] += avg_Buffer[AVGBUF_DCU_ACCX][j];
		avg_Buffer[AVGBUF_DCU_ACCY][10] += avg_Buffer[AVGBUF_DCU_ACCY][j];
		avg_Buffer[AVGBUF_DCU_ACCZ][10] += avg_Buffer[AVGBUF_DCU_ACCZ][j];
		avg_Buffer[AVGBUF_DCU_GYROZ][10] += avg_Buffer[AVGBUF_DCU_GYROZ][j];
		
		avg_Buffer[AVGBUF_VHSPEED][10] += avg_Buffer[AVGBUF_VHSPEED][j];
		
		avg_Buffer[AVGBUF_LINEARFL][10] += avg_Buffer[AVGBUF_LINEARFL][j];
		avg_Buffer[AVGBUF_LINEARFR][10] += avg_Buffer[AVGBUF_LINEARFR][j];
		avg_Buffer[AVGBUF_LINEARRR][10] += avg_Buffer[AVGBUF_LINEARRR][j];
		avg_Buffer[AVGBUF_LINEARRL][10] += avg_Buffer[AVGBUF_LINEARRL][j];
	}
	
	buffer_telemetryPointer = (buffer_telemetryPointer + BUFFER_LEN - 1) % BUFFER_LEN;
	
	decimal_To_String_Unsigned((uint16_t)(avg_Buffer[AVGBUF_WHEELSPEED1][10] / 10.0), &block_Buffer[buffer_telemetryPointer][HALL_EFFECT_FR], 3, 1);
	decimal_To_String_Unsigned((uint16_t)(avg_Buffer[AVGBUF_WHEELSPEED2][10] / 10.0), &block_Buffer[buffer_telemetryPointer][HALL_EFFECT_FL], 3, 1);
	decimal_To_String_Unsigned((uint16_t)(avg_Buffer[AVGBUF_WHEELSPEED3][10] / 10.0), &block_Buffer[buffer_telemetryPointer][HALL_EFFECT_RR], 3, 1);
	decimal_To_String_Unsigned((uint16_t)(avg_Buffer[AVGBUF_WHEELSPEED4][10] / 10.0), &block_Buffer[buffer_telemetryPointer][HALL_EFFECT_RL], 3, 1);
	
	int_To_String_Unsigned((uint16_t)(avg_Buffer[AVGBUF_BRAKEPRESSF][10] / 10.0), &block_Buffer[buffer_telemetryPointer][BPS_FRONT], 5);
	int_To_String_Unsigned((uint16_t)(avg_Buffer[AVGBUF_BRAKEPRESSR][10] / 10.0), &block_Buffer[buffer_telemetryPointer][BPS_FRONT], 5);
	
	int_To_String_Unsigned((uint16_t)(avg_Buffer[AVGBUF_TPS][10] / 10.0), &block_Buffer[buffer_telemetryPointer][TPS_1], 3);
	int_To_String_Unsigned((uint16_t)(avg_Buffer[AVGBUF_SW][10] / 10.0), &block_Buffer[buffer_telemetryPointer][STEERING_WHEEL_ANGLE], 5);
	
	decimal_To_String((int16_t)(avg_Buffer[AVGBUF_ACCX][10] / 10.0), &block_Buffer[buffer_telemetryPointer][ACC_X], 3, 2);
	decimal_To_String((int16_t)(avg_Buffer[AVGBUF_ACCY][10] / 10.0), &block_Buffer[buffer_telemetryPointer][ACC_Y], 3, 2);
	decimal_To_String((int16_t)(avg_Buffer[AVGBUF_ACCZ][10] / 10.0), &block_Buffer[buffer_telemetryPointer][ACC_Z], 3, 2);
	decimal_To_String((int16_t)(avg_Buffer[AVGBUF_GYROZ][10] / 10.0), &block_Buffer[buffer_telemetryPointer][GYR_Z], 3, 2);
	
  decimal_To_String((int16_t)(avg_Buffer[AVGBUF_DCU_ACCX][10] / 10.0), &block_Buffer[buffer_telemetryPointer][DCU_ACC_X], 3, 2);
  decimal_To_String((int16_t)(avg_Buffer[AVGBUF_DCU_ACCY][10] / 10.0), &block_Buffer[buffer_telemetryPointer][DCU_ACC_Y], 3, 2);
  decimal_To_String((int16_t)(avg_Buffer[AVGBUF_DCU_ACCZ][10] / 10.0), &block_Buffer[buffer_telemetryPointer][DCU_ACC_Z], 3, 2);
  decimal_To_String((int16_t)(avg_Buffer[AVGBUF_DCU_GYROZ][10] / 10.0), &block_Buffer[buffer_telemetryPointer][DCU_GYR_Z], 3, 2);
	
	decimal_To_String_Unsigned((uint16_t)(avg_Buffer[AVGBUF_VHSPEED][10] / 10.0), &block_Buffer[buffer_telemetryPointer][VH_SPEED], 3, 1);
	
  int_To_String_Unsigned((uint16_t)(avg_Buffer[AVGBUF_LINEARFL][10] / 10.0), &block_Buffer[buffer_telemetryPointer][LINEARE_FL], 5);
  int_To_String_Unsigned((uint16_t)(avg_Buffer[AVGBUF_LINEARFR][10] / 10.0), &block_Buffer[buffer_telemetryPointer][LINEARE_FR], 5);
  int_To_String_Unsigned((uint16_t)(avg_Buffer[AVGBUF_LINEARRL][10] / 10.0), &block_Buffer[buffer_telemetryPointer][LINEARE_RL], 5);
  int_To_String_Unsigned((uint16_t)(avg_Buffer[AVGBUF_LINEARRR][10] / 10.0), &block_Buffer[buffer_telemetryPointer][LINEARE_RR], 5);	
}

// Funzione di conversione dati, identificati in base all'ID del relativo pacchetto CAN
// La funzione va richiamata nella callback di ricezione pacchetti CAN, con gli opportuni parametri
// Il primo parametro è l'ID del pacchetto da convertire e il secondo è l'array degli 8 byte di dato
extern inline void data_Conversion(uint16_t ID, uint8_t payload[8])
{
  data0 = payload[0] & 0x00FF;
  data1 = payload[1] & 0x00FF;
  data2 = payload[2] & 0x00FF;
  data3 = payload[3] & 0x00FF;
  data4 = payload[4] & 0x00FF;
  data5 = payload[5] & 0x00FF;
  data6 = payload[6] & 0x00FF;
  data7 = payload[7] & 0x00FF;
    
  switch(ID)
  {          
    
    /**********************EFI**********************/
        
    case EFI_HALL_ID:

      // HALL_EFFECT_FR byte 0-1
      uTemp = ((data0 << 8 ) & 0xFF00) | data1;
		
			avg_Buffer[AVGBUF_WHEELSPEED1][avg_Buffer_Pointers[AVGBUF_WHEELSPEED1]] = (float)uTemp;
			avg_Buffer_Pointers[AVGBUF_WHEELSPEED1] = (avg_Buffer_Pointers[AVGBUF_WHEELSPEED1] + 1) % 10;
		
      decimal_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][HALL_EFFECT_FR], 3, 1);
      
      // HALL_EFFECT_FL byte 2-3
      uTemp = ((data2 << 8 ) & 0xFF00) | data3;
		
			avg_Buffer[AVGBUF_WHEELSPEED2][avg_Buffer_Pointers[AVGBUF_WHEELSPEED2]] = (float)uTemp;
			avg_Buffer_Pointers[AVGBUF_WHEELSPEED2] = (avg_Buffer_Pointers[AVGBUF_WHEELSPEED2] + 1) % 10;
		
      decimal_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][HALL_EFFECT_FL], 3, 1);
      
      // HALL_EFFECT_RR byte 4-5
      uTemp = ((data4 << 8 ) & 0xFF00) | data5;
		
			avg_Buffer[AVGBUF_WHEELSPEED3][avg_Buffer_Pointers[AVGBUF_WHEELSPEED3]] = (float)uTemp;
			avg_Buffer_Pointers[AVGBUF_WHEELSPEED3] = (avg_Buffer_Pointers[AVGBUF_WHEELSPEED3] + 1) % 10;
			
      decimal_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][HALL_EFFECT_RR], 3, 1);
      
      // HALL_EFFECT_RL byte 6-7
      uTemp = ((data6 << 8 ) & 0xFF00) | data7;
			
			avg_Buffer[AVGBUF_WHEELSPEED4][avg_Buffer_Pointers[AVGBUF_WHEELSPEED4]] = (float)uTemp;
			avg_Buffer_Pointers[AVGBUF_WHEELSPEED4] = (avg_Buffer_Pointers[AVGBUF_WHEELSPEED4] + 1) % 10;
			
      decimal_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][HALL_EFFECT_RL], 3, 1);
      break;
        
    case EFI_WATER_TEMPERATURE_ID:
        
      // T_H20_SX_IN byte 0-1
      uTemp = ((data0 << 8 ) & 0xFF00) | data1;
      fTemp = temperature_Efi_Conversion(uTemp);
      int_To_String_Unsigned((uint16_t)fTemp, &block_Buffer[buffer_writePointer][T_H20_SX_IN], 3);
      
      // T_H20_SX_OUT byte 2-3
      uTemp = ((data2 << 8 ) & 0xFF00) | data3;
      fTemp = temperature_Efi_Conversion(uTemp);
      int_To_String_Unsigned((uint16_t)fTemp, &block_Buffer[buffer_writePointer][T_H20_SX_OUT], 3);
      
      // T_H20_DX_IN byte 4-5
      uTemp = ((data4 << 8 ) & 0xFF00) | data5;
      fTemp = temperature_Efi_Conversion(uTemp);
      int_To_String_Unsigned((uint16_t)fTemp, &block_Buffer[buffer_writePointer][T_H20_DX_IN], 3);
      
      // T_H20_DX_OUT byte 6-7
      uTemp = ((data6 << 8 ) & 0xFF00) | data7;
      fTemp = temperature_Efi_Conversion(uTemp);
      int_To_String_Unsigned((uint16_t)fTemp, &block_Buffer[buffer_writePointer][T_H20_DX_OUT], 3);
      break;

    case EFI_OIL_T_ENGINE_BAT_ID:
        
      // T_OIL_IN byte 0-1
      uTemp = ((data0 << 8 ) & 0xFF00) | data1;
      fTemp = T_OIL_IN_Conversion(uTemp);
      int_To_String_Unsigned((uint16_t)fTemp, &block_Buffer[buffer_writePointer][T_OIL_IN], 3);
      
      // T_OIL_OUT byte 2-3
      uTemp = ((data2 << 8 ) & 0xFF00) | data3;
      fTemp = temperature_Efi_Conversion(uTemp);
      int_To_String_Unsigned((uint16_t)fTemp, &block_Buffer[buffer_writePointer][T_OIL_OUT], 3);
   
      // T_H20_ENGINE byte 4-5
      uTemp = ((data4 << 8 ) & 0xFF00) | data5;
      fTemp = T_H20_ENGINE_Conversion(uTemp);
      int_To_String_Unsigned((uint16_t)fTemp, &block_Buffer[buffer_writePointer][T_H20_ENGINE], 3);
      
      // BATTERY_VOLTAGE byte 6-7
      uTemp = ((data6 << 8 ) & 0xFF00) | data7;
      fTemp = BATT_VOLTAGE_Conversion(uTemp);
      decimal_To_String_Unsigned((uint16_t)fTemp, &block_Buffer[buffer_writePointer][BATTERY_VOLTAGE], 2, 1);
      break;

    case EFI_GEAR_RPM_TPS_APPS_ID:
        
      // GEAR byte 0-1
      uTemp = ((data0 << 8 ) & 0xFF00) | data1;
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][GEAR], 1);
    
      // RPM byte 2-3
      uTemp = ((data2 << 8 ) & 0xFF00) | data3;
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][RPM], 5);
    
      // TPS_1 byte 4-5
      uTemp = ((data4 << 8 ) & 0xFF00) | data5;
      fTemp = TPS_1_Conversion(uTemp);
					
			avg_Buffer[AVGBUF_TPS][avg_Buffer_Pointers[AVGBUF_TPS]] = fTemp;
			avg_Buffer_Pointers[AVGBUF_TPS] = (avg_Buffer_Pointers[AVGBUF_TPS] + 1) % 10;
		
      int_To_String_Unsigned((uint16_t)fTemp, &block_Buffer[buffer_writePointer][TPS_1], 3);
    
      // PEDAL_POSITION_AVG byte 6-7
      uTemp = ((data6 << 8 ) & 0xFF00) | data7;
      fTemp = PEDAL_POS_AVG_Conversion(uTemp);
      int_To_String_Unsigned((uint16_t)fTemp, &block_Buffer[buffer_writePointer][PEDAL_POSITION_AVG], 3);
      break;
        
    case EFI_TRACTION_CONTROL_ID:
        
      // VH_SPEED byte 0-1
      uTemp = ((data0 << 8 ) & 0xFF00) | data1;
		
			avg_Buffer[AVGBUF_VHSPEED][avg_Buffer_Pointers[AVGBUF_VHSPEED]] = (float)uTemp;
			avg_Buffer_Pointers[AVGBUF_VHSPEED] = (avg_Buffer_Pointers[AVGBUF_VHSPEED] + 1) % 10;
		
      decimal_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][VH_SPEED], 3, 1);
      
      // SLIP_TARGET byte 2-3
      uTemp = ((data2 << 8 ) & 0xFF00) | data3;
      decimal_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][SLIP_TARGET], 3, 1);
      
      // SLIP byte 4-5
      uTemp = ((data4 << 8 ) & 0xFF00) | data5;
      decimal_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][SLIP], 3, 1);
      break;
        
    case EFI_FUEL_FAN_H2O_LAUNCH_ID:
        
      // FUEL_PUMP byte 0-1
      uTemp = ((data0 << 8 ) & 0xFF00) | data1;
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][FUEL_PUMP], 1);
  
      // FAN byte 2-3
      uTemp = ((data2 << 8 ) & 0xFF00) | data3;
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][FAN], 1);
      
      // H20_PUMP_DUTY_CYCLE byte 4-5
      uTemp = ((data4 << 8 ) & 0xFF00) | data5;
      uTemp = uTemp * 100;
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][H20_PUMP_DUTY_CYCLE], 3);
      
      // LAUNCH_CONTROL_ACTIVE byte 6-7
      uTemp = ((data6 << 8 ) & 0xFF00) | data7;
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][LAUNCH_CONTROL_ACTIVE], 1);
      break;
        
    case EFI_PRESSURES_LAMBDA_SMOT_ID:
        
      // FUEL_PRESSURE byte 0-1
      uTemp = ((data0 << 8 ) & 0xFF00) | data1;
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][FUEL_PRESSURE], 5);
    
      // OIL_PRESSURE byte 2-3
      uTemp = ((data2 << 8 ) & 0xFF00) | data3;
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][OIL_PRESSURE], 5);
    
      // LAMBDA byte 4-5
      uTemp = ((data4 << 8 ) & 0xFF00) | data5;
      decimal_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][LAMBDA], 1, 3);
      
      // FLAG_SMOT byte 6-7
      uTemp = ((data6 << 8 ) & 0xFF00) | data7;
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][FLAG_SMOT], 2);
      break;
    
    case EFI_DIAG_IGN_EXHAUST_ID:
        
      // DIAG_IGN_1 byte 0-1
      uTemp = ((data0 << 8 ) & 0xFF00) | data1;
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][DIAG_IGN_1], 1);
      
      // DIAG_IGN_2 byte 2-3
      uTemp = ((data2 << 8 ) & 0xFF00) | data3;
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][DIAG_IGN_2], 1);
      
      // T_SCARICO_1 byte 4-5
      uTemp = ((data4 << 8 ) & 0xFF00) | data5;
      fTemp = T_SCARICO_Conversion(uTemp);
      int_To_String_Unsigned((uint16_t)fTemp, &block_Buffer[buffer_writePointer][T_SCARICO_1], 3);
      
      // T_SCARICO_2 byte 6-7
      uTemp = ((data6 << 8 ) & 0xFF00) | data7;
      fTemp = T_SCARICO_Conversion(uTemp);
      int_To_String_Unsigned((uint16_t)fTemp, &block_Buffer[buffer_writePointer][T_SCARICO_2], 3);
      break;
    
    
    /**********************DAU**********************/
        
    case DAU_FR_ID:
      
      // LINEARE_FR byte 0-1
      uTemp = ((data0 << 8 ) & 0xFF00) | data1;
		
			avg_Buffer[AVGBUF_LINEARFR][avg_Buffer_Pointers[AVGBUF_LINEARFR]] = (float)uTemp;
			avg_Buffer_Pointers[AVGBUF_LINEARFR] = (avg_Buffer_Pointers[AVGBUF_LINEARFR] + 1) % 10;
		
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][LINEARE_FR], 5);
      
      // LOAD_CELL_FR byte 2-3
      uTemp = ((data2 << 8 ) & 0xFF00) | data3;
      fTemp = LOAD_CELL_FR_Conversion(uTemp);
      int_To_String((uint16_t)fTemp, &block_Buffer[buffer_writePointer][LOAD_CELL_FR], 5);
      
      // BPS_FRONT byte 3-4
      uTemp = ((data4 << 8 ) & 0xFF00) | data5;
		
			avg_Buffer[AVGBUF_BRAKEPRESSF][avg_Buffer_Pointers[AVGBUF_BRAKEPRESSF]] = (float)uTemp;
			avg_Buffer_Pointers[AVGBUF_BRAKEPRESSF] = (avg_Buffer_Pointers[AVGBUF_BRAKEPRESSF] + 1) % 10;
		
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][BPS_FRONT], 5);
      break;
        
    case DAU_FL_ID:
        
      // LINEARE_FL byte 0-1
      uTemp = ((data0 << 8 ) & 0xFF00) | data1;
		
			avg_Buffer[AVGBUF_LINEARFL][avg_Buffer_Pointers[AVGBUF_LINEARFL]] = (float)uTemp;
			avg_Buffer_Pointers[AVGBUF_LINEARFL] = (avg_Buffer_Pointers[AVGBUF_LINEARFL] + 1) % 10;
		
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][LINEARE_FL], 5);
      
      // LOAD_CELL_FL byte 2-3
      uTemp = ((data2 << 8 ) & 0xFF00) | data3;
      fTemp = LOAD_CELL_FL_Conversion(uTemp);
      int_To_String((uint16_t)fTemp, &block_Buffer[buffer_writePointer][LOAD_CELL_FL], 5);
      
      // BPS_REAR byte 3-4
      uTemp = ((data4 << 8 ) & 0xFF00) | data5;
		
			avg_Buffer[AVGBUF_BRAKEPRESSR][avg_Buffer_Pointers[AVGBUF_BRAKEPRESSR]] = (float)uTemp;
			avg_Buffer_Pointers[AVGBUF_BRAKEPRESSR] = (avg_Buffer_Pointers[AVGBUF_BRAKEPRESSR] + 1) % 10;
		
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][BPS_REAR], 5);
      
      // STEERING_WHEEL_ANGLE byte 5-6
      uTemp = ((data6 << 8 ) & 0xFF00) | data7;
		
			avg_Buffer[AVGBUF_SW][avg_Buffer_Pointers[AVGBUF_SW]] = (float)uTemp;
			avg_Buffer_Pointers[AVGBUF_SW] = (avg_Buffer_Pointers[AVGBUF_SW] + 1) % 10;
		
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][STEERING_WHEEL_ANGLE], 5);
      break;
        
    case DAU_REAR_ID:
        
      // LINEARE_RL byte 0-1
      uTemp = ((data0 << 8 ) & 0xFF00) | data1;
		
			avg_Buffer[AVGBUF_LINEARRL][avg_Buffer_Pointers[AVGBUF_LINEARRL]] = (float)uTemp;
			avg_Buffer_Pointers[AVGBUF_LINEARRL] = (avg_Buffer_Pointers[AVGBUF_LINEARRL] + 1) % 10;
		
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][LINEARE_RL], 5);
      
      // LOAD_CELL_RL byte 2-3
      uTemp = ((data2 << 8 ) & 0xFF00) | data3;
      fTemp = LOAD_CELL_RL_Conversion(uTemp);
      int_To_String((uint16_t)fTemp, &block_Buffer[buffer_writePointer][LOAD_CELL_RL], 5);
      
      // LINEARE_RR byte 3-4
      uTemp = ((data4 << 8 ) & 0xFF00) | data5;
		
			avg_Buffer[AVGBUF_LINEARRR][avg_Buffer_Pointers[AVGBUF_LINEARRR]] = (float)uTemp;
			avg_Buffer_Pointers[AVGBUF_LINEARRR] = (avg_Buffer_Pointers[AVGBUF_LINEARRR] + 1) % 10;
		
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][LINEARE_RR], 5);
      
      // LOAD_CELL_RR byte 5-6
      uTemp = ((data6 << 8 ) & 0xFF00) | data7;
      fTemp = LOAD_CELL_RR_Conversion(uTemp);
      int_To_String((uint16_t)fTemp, &block_Buffer[buffer_writePointer][LOAD_CELL_RR], 5);
      break;
        
    case DAU_FR_APPS_ID:
        
      // APPS1 byte 0-1
      uTemp = ((data0 << 8 ) & 0xFF00) | data1;
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][APPS1], 5);
      
      // APPS2 byte 2-3
      uTemp = ((data2 << 8 ) & 0xFF00) | data3;
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][APPS2], 5);
      break;
        
    case IR_FL_ID:
        
      // IR1_FL byte 0-1
      uTemp = ((data0 << 8 ) & 0xFF00) | data1;
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][IR1_FL], 5);
      
      // IR2_FL byte 2-3
      uTemp = ((data2 << 8 ) & 0xFF00) | data3;
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][IR2_FL], 5);
      
      // IR3_FL byte 3-4
      uTemp = ((data4 << 8 ) & 0xFF00) | data5;
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][IR3_FL], 5);
      break;
        
    case IR_FR_ID:
        
      // IR1_FR byte 0-1
      uTemp = ((data0 << 8 ) & 0xFF00) | data1;
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][IR1_FR], 5);
      
      // IR2_FR byte 2-3
      uTemp = ((data2 << 8 ) & 0xFF00) | data3;
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][IR2_FR], 5);

      // IR3_FR byte 3-4
      uTemp = ((data4 << 8 ) & 0xFF00) | data5;
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][IR3_FR], 5);       
      break;
        
    case IR_RL_ID:
        
      // IR1_RL byte 0-1
      uTemp = ((data0 << 8 ) & 0xFF00) | data1;
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][IR1_RL], 5);
      
      // IR2_RL byte 2-3
      uTemp = ((data2 << 8 ) & 0xFF00) | data3;
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][IR2_RL], 5);
      
      // IR3_RL byte 3-4
      uTemp = ((data4 << 8 ) & 0xFF00) | data5;
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][IR3_RL], 5);
      break;
        
    case IR_RR_ID:
        
      // IR1_RR byte 0-1
      uTemp = ((data0 << 8 ) & 0xFF00) | data1;
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][IR1_RR], 5);
      
      // IR2_RR byte 2-3
      uTemp = ((data2 << 8 ) & 0xFF00) | data3;
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][IR2_RR], 5);
      
      // IR3_RR byte 3-4
      uTemp = ((data4 << 8 ) & 0xFF00) | data5;
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][IR3_RR], 5);
      break;
        
        
    /**********************IMU**********************/
    
    case IMU_DATA_1_ID:
        
      // ACC_X X byte 0-1
      iTemp = ((data0 << 8 ) & 0xFF00) | data1;
		
			avg_Buffer[AVGBUF_ACCX][avg_Buffer_Pointers[AVGBUF_ACCX]] = (float)iTemp;
			avg_Buffer_Pointers[AVGBUF_ACCX] = (avg_Buffer_Pointers[AVGBUF_ACCX] + 1) % 10;
		
      decimal_To_String((int16_t)iTemp, &block_Buffer[buffer_writePointer][ACC_X], 3, 2);
      
      // ACC_Y byte 2-3
      iTemp = ((data2 << 8 ) & 0xFF00) | data3;
		
			avg_Buffer[AVGBUF_ACCY][avg_Buffer_Pointers[AVGBUF_ACCY]] = (float)iTemp;
			avg_Buffer_Pointers[AVGBUF_ACCY] = (avg_Buffer_Pointers[AVGBUF_ACCY] + 1) % 10;
		
      decimal_To_String((int16_t)iTemp, &block_Buffer[buffer_writePointer][ACC_Y], 3, 2);
      
      // GYR_X byte 4-5
      iTemp = ((data4 << 8 ) & 0xFF00) | data5;
      fTemp = GYR_Conversion(iTemp);
      decimal_To_String((int16_t)fTemp, &block_Buffer[buffer_writePointer][GYR_X], 3, 2);
      
      // GYR_Z byte 6-7
      iTemp = ((data6 << 8 ) & 0xFF00) | data7;
      fTemp = GYR_Conversion(iTemp);
		
			avg_Buffer[AVGBUF_GYROZ][avg_Buffer_Pointers[AVGBUF_GYROZ]] = fTemp;
			avg_Buffer_Pointers[AVGBUF_GYROZ] = (avg_Buffer_Pointers[AVGBUF_GYROZ] + 1) % 10;
		
      decimal_To_String((int16_t)fTemp, &block_Buffer[buffer_writePointer][GYR_Z], 3, 2);
      break;
        
    case IMU_DATA_2_ID:
        
      // HEADING byte 0-1
      uTemp = ((data0 << 8 ) & 0xFF00) | data1;
      fTemp = HEADING_Conversion(uTemp);
      int_To_String_Unsigned((uint16_t)fTemp, &block_Buffer[buffer_writePointer][HEADING], 3);
      
      // ACC_Z byte 2-3
      iTemp = ((data2 << 8 ) & 0xFF00) | data3;
		
			avg_Buffer[AVGBUF_ACCZ][avg_Buffer_Pointers[AVGBUF_ACCZ]] = (float)iTemp;
			avg_Buffer_Pointers[AVGBUF_ACCZ] = (avg_Buffer_Pointers[AVGBUF_ACCZ] + 1) % 10;
		
      decimal_To_String((int16_t)iTemp, &block_Buffer[buffer_writePointer][ACC_Z], 3, 2);
      
      // GYR_Y byte 4-5
      iTemp = ((data4 << 8 ) & 0xFF00) | data5;
      fTemp = GYR_Conversion(iTemp);
      decimal_To_String((int16_t)fTemp, &block_Buffer[buffer_writePointer][GYR_Y], 3, 2);
      break;
        
    case IMU_DATA_3_ID:
        
      // GPS_X byte 0-1
      uTemp = ((data0 << 8 ) & 0xFF00) | data1;
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][GPS_X], 5);
      
      // GPS_Y byte 2-3
      uTemp = ((data2 << 8 ) & 0xFF00) | data3;
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][GPS_Y], 5);
      
      // VELOCITY byte 4-5
      uTemp = ((data4 << 8 ) & 0xFF00) | data5;
      decimal_To_String((uint16_t)uTemp, &block_Buffer[buffer_writePointer][VELOCITY], 3, 2);
      break;
        
    
    /**********************EBB**********************/
        
    case EBB_BIAS_ID:
        
      // BIAS_POSITION byte 0-1
      uTemp = ((data0 << 8 ) & 0xFF00) | data1;
      int_To_String_Unsigned((uint16_t)uTemp, &block_Buffer[buffer_writePointer][BIAS_POSITION], 5);
      break;
  }
}


extern inline void Imu_Dcu_Conversion_To_Buffer(void)
{
	//ACC X			
	avg_Buffer[AVGBUF_DCU_ACCX][avg_Buffer_Pointers[AVGBUF_DCU_ACCX]] = (float)raw_Data.accelerometer[0];
	avg_Buffer_Pointers[AVGBUF_DCU_ACCX] = (avg_Buffer_Pointers[AVGBUF_DCU_ACCX] + 1) % 10;
			
  decimal_To_String((int16_t)raw_Data.accelerometer[0], &block_Buffer[buffer_writePointer][DCU_ACC_X], 3, 2);
	
	//ACC Y
	avg_Buffer[AVGBUF_DCU_ACCY][avg_Buffer_Pointers[AVGBUF_DCU_ACCY]] = (float)raw_Data.accelerometer[0];
	avg_Buffer_Pointers[AVGBUF_DCU_ACCY] = (avg_Buffer_Pointers[AVGBUF_DCU_ACCY] + 1) % 10;
			
	decimal_To_String((int16_t)raw_Data.accelerometer[1], &block_Buffer[buffer_writePointer][DCU_ACC_Y], 3, 2);
	
	//ACC Z
	avg_Buffer[AVGBUF_DCU_ACCZ][avg_Buffer_Pointers[AVGBUF_DCU_ACCZ]] = (float)raw_Data.accelerometer[0];
	avg_Buffer_Pointers[AVGBUF_DCU_ACCZ] = (avg_Buffer_Pointers[AVGBUF_DCU_ACCZ] + 1) % 10;
			
	decimal_To_String((int16_t)raw_Data.accelerometer[2], &block_Buffer[buffer_writePointer][DCU_ACC_Z], 3, 2);
	
	//GYR X
  fTemp = GYR_Conversion(raw_Data.gyroscope[0]);
  decimal_To_String((int16_t)raw_Data.gyroscope[0], &block_Buffer[buffer_writePointer][DCU_GYR_X], 3, 2);
	
	//GYR Y
  fTemp = GYR_Conversion(raw_Data.gyroscope[1]);
  decimal_To_String((int16_t)raw_Data.gyroscope[1], &block_Buffer[buffer_writePointer][DCU_GYR_Y], 3, 2);
	
	//GYR Z
  fTemp = GYR_Conversion(raw_Data.gyroscope[2]);
	
	avg_Buffer[AVGBUF_DCU_GYROZ][avg_Buffer_Pointers[AVGBUF_DCU_ACCX]] = fTemp;
	avg_Buffer_Pointers[AVGBUF_DCU_GYROZ] = (avg_Buffer_Pointers[AVGBUF_DCU_GYROZ] + 1) % 10;
			
  decimal_To_String((int16_t)raw_Data.gyroscope[2], &block_Buffer[buffer_writePointer][DCU_GYR_Z], 3, 2);

	//HEADING
  fTemp = HEADING_Conversion(raw_Data.heading);
  int_To_String_Unsigned((uint16_t)fTemp, &block_Buffer[buffer_writePointer][DCU_HEADING], 3);
}


extern inline void debug_Data_Conversion(uint16_t ID, uint8_t payload[8])
{
  data0 = payload[0] & 0x00FF;
  data1 = payload[1] & 0x00FF;
  data2 = payload[2] & 0x00FF;
  data3 = payload[3] & 0x00FF;
  data4 = payload[4] & 0x00FF;
  data5 = payload[5] & 0x00FF;
  data6 = payload[6] & 0x00FF;
  data7 = payload[7] & 0x00FF;
    
  switch(ID)
  {
    case DAU_FR_DEBUG_ID:
      
      // DAU_FR_TEMP byte 0-1
      uTemp = ((data0 << 8 ) & 0xFF00) | data1;
      int_To_String_Unsigned((uint16_t)uTemp, &dcu_Debug_Packet[POSITION_DAU_FR_TEMP], 3);
      
      // DAU_FR_CURRENT byte 2-3
      uTemp = ((data2 << 8 ) & 0xFF00) | data3;
      int_To_String_Unsigned((uint16_t)uTemp, &dcu_Debug_Packet[POSITION_DAU_FR_CURRENT], 4);
      break;
    
    case DAU_FL_DEBUG_ID:
      
      // DAU_FL_TEMP byte 0-1
      uTemp = ((data0 << 8 ) & 0xFF00) | data1;
      int_To_String_Unsigned((uint16_t)uTemp, &dcu_Debug_Packet[POSITION_DAU_FL_TEMP], 3);
      
      // DAU_FL_CURRENT byte 2-3
      uTemp = ((data2 << 8 ) & 0xFF00) | data3;
      int_To_String_Unsigned((uint16_t)uTemp, &dcu_Debug_Packet[POSITION_DAU_FL_CURRENT], 4);
      break;
        
    case DAU_REAR_DEBUG_ID:
      
      // DAU_REAR_TEMP byte 0-1
      uTemp = ((data0 << 8 ) & 0xFF00) | data1;
      int_To_String_Unsigned((uint16_t)uTemp, &dcu_Debug_Packet[POSITION_DAU_REAR_TEMP], 3);
      
      // DAU_REAR_CURRENT byte 2-3
      uTemp = ((data2 << 8 ) & 0xFF00) | data3;
      int_To_String_Unsigned((uint16_t)uTemp, &dcu_Debug_Packet[POSITION_DAU_REAR_CURRENT], 4);
      break;
    
    case SW_DEBUG_ID:
      
      // SW_TEMP byte 0-1
      uTemp = ((data0 << 8 ) & 0xFF00) | data1;
      int_To_String_Unsigned((uint16_t)uTemp, &dcu_Debug_Packet[POSITION_SW_TEMP], 3);
      break;
    
    case EBB_DEBUG_ID:
      
      // EBB_TEMP byte 0-1
      uTemp = ((data0 << 8 ) & 0xFF00) | data1;
      int_To_String_Unsigned((uint16_t)uTemp, &dcu_Debug_Packet[POSITION_EBB_TEMP], 3);
      
      // EBB_BOARD_CURRENT byte 2-3
      uTemp = ((data2 << 8 ) & 0xFF00) | data3;
      int_To_String_Unsigned((uint16_t)uTemp, &dcu_Debug_Packet[POSITION_EBB_BOARD_CURRENT], 4);
      
      // EBB_MOTOR_CURRENT byte 4-5
      uTemp = ((data4 << 8 ) & 0xFF00) | data5;
      int_To_String_Unsigned((uint16_t)uTemp, &dcu_Debug_Packet[POSITION_EBB_MOTOR_CURRENT], 4);
      break;
    
    case GCU_DEBUG_1_ID:
      
      // GCU_TEMP byte 0-1
      uTemp = ((data0 << 8 ) & 0xFF00) | data1;
      int_To_String_Unsigned((uint16_t)uTemp, &dcu_Debug_Packet[POSITION_GCU_TEMP], 3);
      
      // FANS_CURRENT byte 2-3
      uTemp = ((data2 << 8 ) & 0xFF00) | data3;
      int_To_String_Unsigned((uint16_t)uTemp, &dcu_Debug_Packet[POSITION_FANS_CURRENT], 5);
      
      // H2O_PUMP_CURRENT byte 4-5
      uTemp = ((data4 << 8 ) & 0xFF00) | data5;
      int_To_String_Unsigned((uint16_t)uTemp, &dcu_Debug_Packet[POSITION_H2O_PUMP_CURRENT], 5);
      
      // FUEL_PUMP_CURRENT byte 6-7
      uTemp = ((data6 << 8 ) & 0xFF00) | data7;
      int_To_String_Unsigned((uint16_t)uTemp, &dcu_Debug_Packet[POSITION_FUEL_PUMP_CURRENT], 5);
      break;
    
    case GCU_DEBUG_2_ID:
      
      // GEARMOTOR_CURRENT byte 0-1
      uTemp = ((data0 << 8 ) & 0xFF00) | data1;
      int_To_String_Unsigned((uint16_t)uTemp, &dcu_Debug_Packet[POSITION_GEARMOTOR_CURRENT], 5);
      
      // CLUTCH_CURRENT byte 2-3
      uTemp = ((data2 << 8 ) & 0xFF00) | data3;
      int_To_String_Unsigned((uint16_t)uTemp, &dcu_Debug_Packet[POSITION_CLUTCH_CURRENT], 5);
      
      // DRS_CURRENT byte 4-5
      uTemp = ((data4 << 8 ) & 0xFF00) | data5;
      int_To_String_Unsigned((uint16_t)uTemp, &dcu_Debug_Packet[POSITION_DRS_CURRENT], 5);
      break;
  }
}


static inline float GYR_Conversion(uint16_t input)
{
  // Conversione dato: input * 0,0625
  // Tengo già conto di moltiplicare per 100 la conversione, per la formattazione della stringa
  return (((float)input / (float)16) * (float)100);
}


static inline float HEADING_Conversion(uint16_t input)
{
  // Conversione dato: input / 16
  return (((float)input / (float)16));
}


static inline float temperature_Efi_Conversion(uint16_t input)
{
  // La conversione è quella corretta
  // NON per essere usata con le funzioni di conversione decimale in string_utility
  return (((float)(-0.35572) * (float)input) + (float)190.95);
}


static inline float T_SCARICO_Conversion(uint16_t input)
{
  // La conversione è quella corretta
  // NON per essere usata con le funzioni di conversione decimale in string_utility
  return (((float)1.24626 * (float)input) - (float)24.925);
}


static inline float T_OIL_IN_Conversion(uint16_t input)
{
  // La conversione è quella corretta
  // NON per essere usata con le funzioni di conversione decimale in string_utility
  return (((float)(-0.36094) * (float)input) + (float)196.36);
}


static inline float T_H20_ENGINE_Conversion(uint16_t input)
{
  // La conversione è quella corretta
  // NON per essere usata con le funzioni di conversione decimale in string_utility
  return (((float)0.625 * (float)input) - (float)10);
}


static inline float BATT_VOLTAGE_Conversion(uint16_t input)
{
  // Conversione dato: input * 0,01758
  // Tengo già conto di moltiplicare per 10 la conversione, per la formattazione della stringa
  return (((float)0.01758 * (float)input) * (float)10);
}


static inline float TPS_1_Conversion(uint16_t input)
{
  return ((float)0.39216 * input);
}


static inline float PEDAL_POS_AVG_Conversion(uint16_t input)
{
  return ((float)0.09775 * (float)input);
}


static inline float LOAD_CELL_FL_Conversion(uint16_t input)
{  
  // Da sottrarre l'offsett a vuoto
  // Modificare la define, con il valore espresso in Newton
  return ((((float)input / (float)4095 - (float)0.5) * ((float)4448 / ((float)0.002 * (float)50.4))) - LOAD_CELL_FL_OFFSET);
}


static inline float LOAD_CELL_FR_Conversion(uint16_t input)
{
  // Da sottrarre l'offsett a vuoto
  // Modificare la define, con il valore espresso in Newton
  return ((((float)input / (float)4095 - (float)0.5) * ((float)4448 / ((float)0.002 * (float)50.4))) - LOAD_CELL_FR_OFFSET);
}


static inline float LOAD_CELL_RL_Conversion(uint16_t input)
{
  // Da sottrarre l'offsett a vuoto
  // Modificare la define, con il valore espresso in Newton
  return ((((float)input / (float)4095 - (float)0.5) * ((float)4448 / ((float)0.002 * (float)50.4))) - LOAD_CELL_RL_OFFSET);
}


static inline float LOAD_CELL_RR_Conversion(uint16_t input)
{
  // Da sottrarre l'offsett a vuoto
  // Modificare la define, con il valore espresso in Newton
  return ((((float)input / (float)4095 - (float)0.5) * ((float)4448 / ((float)0.002 * (float)50.4))) - LOAD_CELL_RR_OFFSET);
}
