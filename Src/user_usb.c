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


extern inline void USB_Open_File(void)
{
  USB_Get_File_Name(USB_Filename);
  
  if(f_open(&USBHFile, USB_Filename, FA_OPEN_APPEND | FA_WRITE) != FR_OK)
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
  if(f_close(&USBHFile) != FR_OK)
  { 
    UDP_Send_Error(CLOSE_FILE_ERROR);
  }
  
  else
  {
    dcu_State_Packet[DCU_STATE_PACKET_ACQUISITION_ON] = UDP_DCU_STATE_ERROR;
    UDP_Send_Error(CLOSE_FILE_OK);
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
  }
}



extern inline void USB_Close_And_Open_File(void)
{
	if(f_close(&USBHFile) != FR_OK)
  { 
    dcu_State_Packet[DCU_STATE_PACKET_ACQUISITION_ON] = UDP_DCU_STATE_ERROR;
    UDP_Send_Error(CLOSE_FILE_ERROR);
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
  }

	if(f_open(&USBHFile, USB_Filename, FA_OPEN_APPEND | FA_WRITE) != FR_OK)
  {
    UDP_Send_Error(OPEN_FILE_ERROR);
    dcu_State_Packet[DCU_STATE_PACKET_ACQUISITION_ON] = UDP_DCU_STATE_ERROR;
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
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
