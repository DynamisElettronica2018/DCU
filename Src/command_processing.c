#include "command_processing.h"
#include "user_ethernet_udp.h"
#include "telemetry_command.h"
#include "data.h"


static uint8_t telemetry_Is_Alive_First = 1;


/* Funzione che gestisce i comandi ricevuto su ethernet, inviati da telemetry
Viene chiamata nella funzione di gestione della coda di pacchetti ricevuti
Il pacchetto di comando deve assolutamente avere il carattare di terminazione '\0' */
extern inline void UDP_Packet_Comand(char *data)
{
	if(data != NULL)
	{
    
		// Uso il primo carattere del pacchetto ethernet ricevuto come identificativo della board
    // a cui è indirizzato il comando. Il seconod carattere è il condice del comando, seguito
    // da eventuali parametri.
    switch(data[0])
		{
      case DCU_BOARD_CODE:
        dcu_Board_Command(data);        
        break;
      
      /*case DAU_ALL_BOARD_CODE:
        break;
      
      case DAU_FL_BOARD_CODE:
        break;
      
      case DAU_FR_BOARD_CODE:
        break;
      
      case DAU_REAR_BOARD_CODE:
        break;

      case GCU_BOARD_CODE:
        break;
      
      case EBB_BOARD_CODE:
        break;
      
      case IMU_BOARD_CODE:
        break;*/
    }
	}
}


static inline void dcu_Board_Command(char *command)
{
  switch(command[1])
  {
    case TELEMETRY_IS_ALIVE:
      
      if(telemetry_Is_Alive_First == 1)
      {
        telemetry_Is_Alive_First = 0;
        dcu_State_Packet[DCU_STATE_PACKET_TELEMETRY_ON] = UDP_DCU_STATE_OK;
      }
      
      break;
      
    case START_TELEMETRY_COMMAND:
      if(dcu_State_Packet[DCU_STATE_PACKET_TELEMETRY_ON] == UDP_DCU_STATE_ERROR)
      {
        dcu_State_Packet[DCU_STATE_PACKET_TELEMETRY_ON] = UDP_DCU_STATE_OK;
      }
      
      break;
    
    case STOP_TELEMETRY_COMMAND:
      if(dcu_State_Packet[DCU_STATE_PACKET_TELEMETRY_ON] == UDP_DCU_STATE_OK)
      {
        dcu_State_Packet[DCU_STATE_PACKET_TELEMETRY_ON] = UDP_DCU_STATE_ERROR;
      }
      
      break;
    
    case START_ACQUISITION_COMMAND:
      if(dcu_State_Packet[DCU_STATE_PACKET_ACQUISITION_ON] == UDP_DCU_STATE_ERROR)
      {
        start_Acquisition_Request = START_ACQUISITION_REQUEST;
      }
      
      break;
    
    case STOP_ACQUISITION_COMMAND:
      if(dcu_State_Packet[DCU_STATE_PACKET_ACQUISITION_ON] == UDP_DCU_STATE_OK)
		  {
        start_Acquisition_Request = STOP_ACQUISITION_REQUEST;
      }
      
      break;
    
    case START_ACQUISITION_AND_TELEMETRY:
      if(dcu_State_Packet[DCU_STATE_PACKET_ACQUISITION_ON] == UDP_DCU_STATE_ERROR)
      {
        start_Acquisition_Request = START_ACQUISITION_REQUEST;
      }
      
      if(dcu_State_Packet[DCU_STATE_PACKET_TELEMETRY_ON] == UDP_DCU_STATE_ERROR)
      {
        dcu_State_Packet[DCU_STATE_PACKET_TELEMETRY_ON] = UDP_DCU_STATE_OK;
      }
      
      break;
    
    case STOP_ACQUISITION_AND_TELEMETRY:
      if(dcu_State_Packet[DCU_STATE_PACKET_ACQUISITION_ON] == UDP_DCU_STATE_OK)
		  {
        start_Acquisition_Request = STOP_ACQUISITION_REQUEST;
      }
      
      if(dcu_State_Packet[DCU_STATE_PACKET_TELEMETRY_ON] == UDP_DCU_STATE_OK)
      {
        dcu_State_Packet[DCU_STATE_PACKET_TELEMETRY_ON] = UDP_DCU_STATE_ERROR;
      }
      
      break;
        
    /*case SET_RTC_TIME: 
      break;
    
    case SET_RTC_DATA:
      break;*/
  }
  
  UDP_Send_Queue(UDP_DCU_STATE_PORT, dcu_State_Packet, BUFFER_STATE_LEN);
}
