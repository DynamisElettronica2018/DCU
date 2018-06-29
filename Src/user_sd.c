//#include "user_sd.h"
//#include "sdmmc.h"
//#include "fatfs.h"
//#include "data.h"
//#include "tim.h"
//#include "rtc.h"
//#include "telemetry_command.h"
//#include "user_ethernet_udp.h"


//static RTC_DateTypeDef RTC_Date;
//static RTC_TimeTypeDef RTC_Time;
//static char SD_Filename[13];


/*extern void SD_Init(void)
{	
  if(f_mount(&SDFatFS, SDPath, 1) != FR_OK)
  {
    UDP_Send_Error(SD_DRIVER_ERROR);
    dcu_State_Packet[DCU_STATE_PACKET_SD_READY] = UDP_DCU_STATE_ERROR;
    dcu_State_Packet[DCU_STATE_PACKET_SD_PRESENT] = UDP_DCU_STATE_ERROR;
  }
  
  else
  {
    dcu_State_Packet[DCU_STATE_PACKET_SD_PRESENT] = UDP_DCU_STATE_OK;
    dcu_State_Packet[DCU_STATE_PACKET_SD_READY] = UDP_DCU_STATE_OK;
  }
}


extern void SD_Check_And_Init(void)
{	
  uint8_t SD_Is_Detected;
  FRESULT SD_Status;
  
  SD_Is_Detected = BSP_SD_IsDetected();
  
  // BSP_SD_IsDetected setta SD_PRESENT se legge 1
  // Il pin invece è in pull-up e viene portato a 0 se l'SD è presente
  // La logica è quindi al contrario
  if(SD_Is_Detected == SD_NOT_PRESENT)
  {
    dcu_State_Packet[DCU_STATE_PACKET_SD_PRESENT] = UDP_DCU_STATE_OK;
    SD_Status = f_mount(&SDFatFS, SDPath, 1);
	
    if(SD_Status != FR_OK)
    {
      dcu_State_Packet[DCU_STATE_PACKET_SD_READY] = UDP_DCU_STATE_ERROR;
      UDP_Send_Error(SD_DRIVER_ERROR);
    }
    
    else
    {
      dcu_State_Packet[DCU_STATE_PACKET_SD_READY] = UDP_DCU_STATE_OK;		
    }
  }
  
  else
  {
    dcu_State_Packet[DCU_STATE_PACKET_SD_PRESENT] = UDP_DCU_STATE_ERROR;
  }
}


extern inline void SD_Open_File(void)
{
  FRESULT SD_Status;
  
  SD_Get_File_Name(SD_Filename);
	SD_Status = f_open(&SDFile, (const TCHAR*)SD_Filename, FA_OPEN_APPEND|FA_WRITE|FA_READ);
	
  if(SD_Status != FR_OK)
	{	
		UDP_Send_Error(OPEN_FILE_ERROR);
	}
  
  else
  {
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    UDP_Send_Error(OPEN_FILE_OK);
  }
}


extern inline void SD_Write_Len(uint8_t *write_Buffer)
{
  uint32_t written_Bytes;
  uint32_t bytes;
  FRESULT SD_Status;
  
  bytes = strlen((char *)write_Buffer);
	SD_Status = f_write(&SDFile, write_Buffer, bytes, (void*)&written_Bytes);
		
	if(SD_Status != FR_OK)
	{
		UDP_Send_Error(WRITE_FILE_ERROR);
	}
}


extern inline void SD_write_Block(uint8_t *write_Buffer)
{
  uint32_t written_Bytes;
  FRESULT SD_Status;
  
	SD_Status = f_write(&SDFile, write_Buffer, BUFFER_BLOCK_LEN, (void*)&written_Bytes);
		
	if(SD_Status != FR_OK)
	{
		UDP_Send_Error(WRITE_FILE_ERROR);
	}
}


extern inline void SD_Close_File(void)
{
  if(f_close(&SDFile) != FR_OK)
	{
		UDP_Send_Error(CLOSE_FILE_ERROR);
	}
  
  else
  {
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
    UDP_Send_Error(CLOSE_FILE_OK);
  }
}


extern inline void SD_Get_File_Name(char *filename)
{
  RTC_Get_Value(&RTC_Date, &RTC_Time);
  sprintf(filename, "%2d_%2d_%2d.csv", RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds);
}*/
