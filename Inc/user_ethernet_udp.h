#ifndef __USER_ETHERNET_UDP_H__
#define __USER_ETHERNET_UDP_H__

#include "lwip/udp.h"

#define UDP_TELEMETRY_DATA_PORT       ((uint16_t)50505)
#define UDP_DCU_STATE_PORT            ((uint16_t)52901)
#define UDP_DCU_ERROR_PORT            ((uint16_t)54321)
#define UDP_TELEMETRY_COMMAND_PORT    ((uint16_t)61000)
#define UDP_TELEMETRY_DEBUG_PORT      ((uint16_t)65301)
#define DEST_IP_ADDR0                 ((uint8_t)255)
#define DEST_IP_ADDR1                 ((uint8_t)255)
#define DEST_IP_ADDR2                 ((uint8_t)255)
#define DEST_IP_ADDR3                 ((uint8_t)255)
#define UDP_SEND_BUFFER_LEN           ((uint16_t)4096)
#define UDP_RECEIVE_BUFFER_LEN        ((uint16_t)128)


typedef struct
{
  struct pbuf buffer;
  struct udp_pcb *pcb;
  uint8_t to_Write;
} UDP_Send_Buffer_t;


typedef struct
{
  char *buffer;
  uint8_t to_Read;
} UDP_Receive_Buffer_t;


extern void UDP_Init(void);
extern inline void UDP_Send(uint8_t *msg, struct udp_pcb *pcb);
extern inline void UDP_Send_Error(uint8_t error_Code);
extern inline void UDP_Send_Queue(uint8_t *msg, uint16_t len, uint16_t port);
extern inline void UDP_Send_Processig(void);
extern inline void UDP_Receive_Processig(void);
static inline err_t UDP_Queue_Put(struct pbuf UDP_Send_packet, uint16_t port);
static inline struct udp_pcb *UDP_Get_Pcb(uint16_t port);
static inline void UDP_Packet_Comand(char *data);
static inline void dcu_Board_Command(char *command);
void UDP_Receive_Callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);


#endif
