#ifndef USER_DEFINES
#define	USER_DEFINES

#include <stdint.h>
#include "user_ethernet_udp.h"


extern char USB_Filename[20];
extern volatile uint32_t timestamp;
extern volatile uint8_t canStartAcquisitionRequest;
extern volatile uint8_t usbBlockWriteFlag;
extern volatile uint8_t USB_Logger_State;
extern uint8_t *bufferBlockWrite;
extern uint32_t LED_Actual_Time;
extern uint32_t LED_Toogle_Time;

extern UDP_Send_Buffer_t UDP_Send_Buffer[UDP_SEND_BUFFER_LEN];
extern UDP_Receive_Buffer_t UDP_Receive_Buffer[UDP_RECEIVE_BUFFER_LEN];
extern uint8_t IP_ADDR0;
extern uint8_t IP_ADDR1;
extern uint8_t IP_ADDR2;
extern uint8_t IP_ADDR3;
extern uint8_t NETMASK_ADDR0;
extern uint8_t NETMASK_ADDR1;
extern uint8_t NETMASK_ADDR2;
extern uint8_t NETMASK_ADDR3;
extern uint8_t GW_ADDR0;
extern uint8_t GW_ADDR1;
extern uint8_t GW_ADDR2;
extern uint8_t GW_ADDR3;
extern uint16_t UDP_Send_Buffer_Index;
extern uint16_t UDP_Read_Buffer_Index;
extern volatile uint16_t UDP_Queue_Buffer_Index;
extern volatile uint16_t UDP_Receive_Buffer_Index;

extern uint8_t usb_Present;
extern uint8_t usb_Ready;
extern volatile uint8_t acquisition_On;
extern volatile uint8_t telemetry_On;


#endif	/* USER_DEFINES */
