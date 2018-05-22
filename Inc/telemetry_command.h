#ifndef TELEMETRY_COMMAND_H
#define	TELEMETRY_COMMAND_H


// UDP_DCU_STATE_PORT
#define DCU_STATE_PACKET_PERIOD   ((uint8_t)10)
#define UDP_DCU_STATE_PACKET   		((uint8_t)'M')
#define UDP_DCU_STATE_OK   				((uint8_t)'Y')
#define UDP_DCU_STATE_ERROR				((uint8_t)'N')

// UDP_DCU_ERROR_PORT
#define UDP_DCU_ERROR_PACKET   		((uint8_t)'E')
#define OPEN_FILE_ERROR						((uint8_t)'Q')
#define WRITE_FILE_ERROR					((uint8_t)'W')
#define CLOSE_FILE_ERROR					((uint8_t)'S')
#define OPEN_FILE_OK							((uint8_t)'R')
#define CLOSE_FILE_OK							((uint8_t)'T')
#define USB_DRIVER_ERROR					((uint8_t)'Y')
#define USB_OVERCURRENT					  ((uint8_t)'D')
#define FATFS_LINK_ERROR					((uint8_t)'U')
#define ETHERNET_TX_ERROR					((uint8_t)'O')
#define ETHERNET_RX_ERROR					((uint8_t)'I')
#define ETHERNET_RX_QUEUE_ERROR		((uint8_t)'P')
#define ETHERNET_TX_QUEUE_ERROR		((uint8_t)'A')

//UDP_TELEMETRY_COMAND_PORT
#define DCU_BOARD_CODE										((uint8_t)'D')
#define DAU_ALL_BOARD_CODE								((uint8_t)'A')
#define DAU_FL_BOARD_CODE									((uint8_t)'B')
#define DAU_FR_BOARD_CODE									((uint8_t)'C')
#define DAU_REAR_BOARD_CODE								((uint8_t)'E')
#define GCU_BOARD_CODE										((uint8_t)'G')
#define START_TELEMETRY_COMMAND 					((uint8_t)'M')
#define STOP_TELEMETRY_COMMAND						((uint8_t)'N')
#define START_ACQUISITION_COMMAND					((uint8_t)'B')
#define STOP_ACQUISITION_COMMAND 					((uint8_t)'V')
#define START_ACQUISITION_AND_TELEMETRY 	((uint8_t)'X')
#define STOP_ACQUISITION_AND_TELEMETRY 		((uint8_t)'Z')
#define START_DEBUG_COMMAND 							((uint8_t)'L')
#define STOP_DEBUG_COMMAND 								((uint8_t)'K')
#define SET_RTC_TIME 											((uint8_t)'J')
#define SET_RTC_DATA 											((uint8_t)'H')


#endif	/* TELEMETRY_COMMAND_H */
