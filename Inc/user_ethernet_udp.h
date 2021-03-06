#ifndef __USER_ETHERNET_UDP_H__
#define __USER_ETHERNET_UDP_H__

#include "lwip/udp.h"
#include "user_defines.h"


#define UDP_TELEMETRY_DATA_PORT       ((uint16_t)50505)
#define UDP_DCU_STATE_PORT            ((uint16_t)52901)
#define UDP_DCU_ERROR_PORT            ((uint16_t)54321)
#define UDP_TELEMETRY_COMMAND_PORT    ((uint16_t)61000)
#define UDP_TELEMETRY_DEBUG_PORT      ((uint16_t)65301)
#define DEST_IP_ADDR0                 ((uint8_t)255)
#define DEST_IP_ADDR1                 ((uint8_t)255)
#define DEST_IP_ADDR2                 ((uint8_t)255)
#define DEST_IP_ADDR3                 ((uint8_t)255)
#define UDP_SEND_BUFFER_LEN           ((uint16_t)2048)
#define UDP_RECEIVE_BUFFER_LEN        ((uint16_t)128)


typedef struct
{
  uint16_t port;
  uint8_t *packet_Data;
  uint16_t len;
  uint8_t to_Write;
} UDP_Send_Buffer_t;

typedef struct
{
  char *buffer;
  uint8_t to_Read;
} UDP_Receive_Buffer_t;


extern void UDP_Init(void);
extern void UDP_Deinit(void);
extern inline void UDP_Send(uint8_t *msg, struct udp_pcb *pcb);
extern inline void UDP_Send_Len(uint8_t *msg, struct udp_pcb *pcb, uint16_t len);
extern inline void UDP_Send_Error(uint8_t error_Code);
extern inline void UDP_Send_Date_Time(void);
extern inline void UDP_Send_Queue(const uint16_t port, uint8_t *data, const uint16_t len);
extern inline void UDP_Send_Processig(void);
extern inline void UDP_Receive_Processig(void);
extern void netif_status_callback(struct netif *netif);
void UDP_Receive_Telemetry_Command(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
static inline struct udp_pcb *get_Udp_Pcb(const uint16_t port);


#endif
