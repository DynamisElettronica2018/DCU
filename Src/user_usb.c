#include "user_usb.h"
#include "telemetry_command.h"
#include "user_ethernet_udp.h"
#include "fatfs.h"
#include "data.h"
#include "tim.h"
#include "rtc.h"


static RTC_DateTypeDef RTC_Date;
static RTC_TimeTypeDef RTC_Time;
static char USB_Filename[13];
static uint8_t header_Packet_Buffer[] = "TIMESTAMP;HALL EFFECT FR;HALL EFFECT FL;HALL EFFECT RR;HALL EFFECT RL;T H20 SX IN;T H20 SX OUT;T H20 DX IN;T H20 DX OUT;T OIL IN;T OIL OUT;T H20 ENGINE;BATTERY VOLTAGE;GEAR;RPM;TPS 1;PEDAL POSITION AVG;VH SPEED;SLIP TARGET;SLIP;FUEL PUMP;FAN;H20 PUMP DUTY CYCLE;LAUNCH CONTROL ACTIVE;FUEL PRESSURE;OIL PRESSURE;LAMBDA;FLAG SMOT;DIAG IGN 1;DIAG IGN 2;T SCARICO 1;T SCARICO 2;LINEARE FR;LOAD CELL FR;BPS FRONT;LINEARE FL;LOAD CELL FL;BPS REAR;STEERING WHEEL ANGLE;LINEARE RL;LOAD CELL RL;LINEARE RR;LOAD CELL RR;APPS1;APPS2;IR1 FL;IR2 FL;IR3 FL;IR1 FR;IR2 FR;IR3 FR;IR1 RL;IR2 RL;IR3 RL;IR1 RR;IR2 RR;IR3 RR;ACC X;ACC Y;GYR X;GYR Z;HEADING;ACC Z;GYR Y;GPS X;GPS Y;VELOCITY;BIAS POSITION;DCU_ACC X;DCU_ACC Y;DCU_ACC Z;DCU_GYR X;DCU_GYR Y;DCU_GYR Z;DCU_HEADING;VUOTO;\n";


extern inline void USB_Open_File(void)
{
  USB_Get_File_Name(USB_Filename);
  
  if(f_open(&USBHFile, USB_Filename, FA_CREATE_ALWAYS | FA_WRITE)!= FR_OK)
  {
    UDP_Send_Error(OPEN_FILE_ERROR);
  }
  
  else
  {
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    UDP_Send_Error(OPEN_FILE_OK);
    USB_Write_Len(header_Packet_Buffer);
    dcu_State_Packet[DCU_STATE_PACKET_ACQUISITION_ON] = UDP_DCU_STATE_OK;
    USB_Timestamp = 0;
  }
}


extern inline void USB_Close_File(void)
{  
  if(f_close(&USBHFile) == FR_OK)
  { 
    dcu_State_Packet[DCU_STATE_PACKET_ACQUISITION_ON] = UDP_DCU_STATE_ERROR;
    UDP_Send_Error(CLOSE_FILE_OK);
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
  }
  
  else
  {
    UDP_Send_Error(CLOSE_FILE_ERROR);
  }
}


extern inline void USB_Write_Block(uint8_t *buffer)
{
  uint32_t bytes_Written;
  FRESULT file_Error;

  file_Error = f_write(&USBHFile, buffer, BUFFER_BLOCK_LEN, (void *)&bytes_Written);
    
  if(file_Error != FR_OK)
  {
    UDP_Send_Error(WRITE_FILE_ERROR);
  }
}


extern inline void USB_Write_Len(uint8_t *buffer)
{
  uint32_t bytes_Written;
  FRESULT file_Error;
  uint32_t len;

  len = strlen((char *)buffer);
  file_Error = f_write(&USBHFile, buffer, len, (void *)&bytes_Written);
    
  if((file_Error != FR_OK) || (bytes_Written != len))
  {
    UDP_Send_Error(WRITE_FILE_ERROR);
  }
}


static inline void USB_Get_File_Name(char *filename)
{
  RTC_Get_Value(&RTC_Date, &RTC_Time);
  sprintf(filename, "%d_%d_%d.csv", RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds);
}
