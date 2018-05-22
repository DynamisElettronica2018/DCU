#include "user_usb.h"
#include "user_defines.h"
#include "user_externalVariables.h"
#include "telemetry_command.h"
#include "fatfs.h"
#include "tim.h"
#include "rtc.h"
#include "user_ethernet_udp.h"


RTC_DateTypeDef RTC_Date;
RTC_TimeTypeDef RTC_Time;


extern inline void USB_User_Start(const char *filename)
{  
  timestamp = 0;
  
  if(f_open(&USBHFile, filename, FA_CREATE_ALWAYS | FA_WRITE)!= FR_OK)
  {
    UDP_Send_Error(OPEN_FILE_ERROR);
  }
  
  else
  {
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    HAL_TIM_Base_Start_IT(&htim6);
    acquisition_On = UDP_DCU_STATE_OK;
    UDP_Send_Error(OPEN_FILE_OK);
  }
}


extern inline void USB_User_Stop(void)
{  
  HAL_TIM_Base_Stop_IT(&htim6);
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
  
  if(f_close(&USBHFile) != FR_OK)
  {
    UDP_Send_Error(CLOSE_FILE_ERROR);
  }
  
  else
  {
    acquisition_On = UDP_DCU_STATE_ERROR;
    UDP_Send_Error(CLOSE_FILE_OK);
  }
}


extern inline void USB_Write(uint8_t *buffer)
{
  uint32_t bytes_Written;
  FRESULT file_Error;

  file_Error = f_write(&USBHFile, buffer, BUFFER_BLOCK_LEN, (void *)&bytes_Written);
    
  if((file_Error != FR_OK) || (bytes_Written != BUFFER_BLOCK_LEN))
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


extern inline void USB_Get_File_Name(char *filename)
{
  RTC_Get_Value(&RTC_Date, &RTC_Time);
  sprintf(filename, "%2d_%2d_%2d.csv", RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds);
}
