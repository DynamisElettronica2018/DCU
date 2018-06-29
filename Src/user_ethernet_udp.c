#include <string.h>
#include "user_ethernet_udp.h"
#include "command_processing.h"
#include "lwip.h"
#include "tim.h"
#include "user_defines.h"
#include "telemetry_command.h"


static struct udp_pcb *telemetry_Data_Port;
static struct udp_pcb *dcu_State_Port;
static struct udp_pcb *dcu_Error_Port;
static struct udp_pcb *telemetry_Command_port;
static struct udp_pcb *telemetry_Debug_Port;
static struct pbuf *local_Buffer = NULL;
static struct udp_pcb *send_Processing_Tx_Pcb;
static struct pbuf *send_Processing_Tx_Packet;
static uint16_t send_Processing_Len;
static const void *rx_Dataptr;
static char *rx_Data;
static UDP_Send_Buffer_t UDP_Send_Buffer[UDP_SEND_BUFFER_LEN];
static UDP_Receive_Buffer_t UDP_Receive_Buffer[UDP_RECEIVE_BUFFER_LEN];
static volatile uint16_t UDP_Send_Buffer_Index = 0;
static volatile uint16_t UDP_Read_Buffer_Index = 0;
static volatile uint16_t UDP_Queue_Buffer_Index = 0;
static volatile uint16_t UDP_Receive_Buffer_Index = 0;
static uint8_t dcu_Error_Buffer[BUFFER_ERROR_LEN];


extern void UDP_Init(void)
{
  ip_addr_t DestIPaddr;
  err_t err;
  
  // TX buffer initialization
  for(uint16_t i = 0; i < UDP_SEND_BUFFER_LEN; i++)
  {
    UDP_Send_Buffer[i].port = 0;
    UDP_Send_Buffer[i].packet_Data = NULL;
    UDP_Send_Buffer[i].len = 0;
    UDP_Send_Buffer[i].to_Write = DIRTY;
  }
  
  // RX buffer initialization 
  for(uint16_t i = 0; i < UDP_RECEIVE_BUFFER_LEN; i++)
  {
    UDP_Receive_Buffer[i].buffer = NULL;
    UDP_Receive_Buffer[i].to_Read = DIRTY;
  }

  // Error buffer initialization
  dcu_Error_Buffer[0] = 'E';
  dcu_Error_Buffer[2] = '\0';
  
  UDP_Deinit();
  telemetry_Data_Port = udp_new();
  dcu_State_Port = udp_new();
  dcu_Error_Port = udp_new();
  telemetry_Command_port = udp_new();
  telemetry_Debug_Port = udp_new();
  IP4_ADDR(&DestIPaddr, DEST_IP_ADDR0, DEST_IP_ADDR1, DEST_IP_ADDR2, DEST_IP_ADDR3);
  
  if(telemetry_Command_port != NULL)
  {
    // Binds the pcb to a local address.
    // The IP-address argument can be IP_ADDR_ANY to indicate that it should listen to any local IP address. 
    // The function currently always return ERR_OK.
    err = udp_bind(telemetry_Command_port, IP_ADDR_ANY, UDP_TELEMETRY_COMMAND_PORT);
    
    if(err == ERR_OK)
    {       
      // Specifies a callback function that should be called when a UDP datagram is received.
      udp_recv(telemetry_Command_port, UDP_Receive_Telemetry_Command, NULL);
    }
    
    else
    {
      udp_remove(telemetry_Command_port);
    }
  }

  if(telemetry_Data_Port != NULL)
  {
    // Sets the remote end of the pcb.
    // This function does not generate any network traffic, but set the remote address of the pcb.
    err = udp_connect(telemetry_Data_Port, &DestIPaddr, UDP_TELEMETRY_DATA_PORT);
       
    if(err != ERR_OK)
    {
      udp_remove(telemetry_Data_Port);
    }
  }

  if(dcu_State_Port != NULL)
  {
    err = udp_connect(dcu_State_Port, &DestIPaddr, UDP_DCU_STATE_PORT);
       
    if(err != ERR_OK)
    {
      udp_remove(dcu_State_Port);
    }
  }

  if(dcu_Error_Port != NULL)
  {
    err = udp_connect(dcu_Error_Port, &DestIPaddr, UDP_DCU_ERROR_PORT);
       
    if(err != ERR_OK)
    {
      udp_remove(dcu_Error_Port);
    }
  }
  
  if(telemetry_Debug_Port != NULL)
  {
    err = udp_connect(telemetry_Debug_Port, &DestIPaddr, UDP_TELEMETRY_DEBUG_PORT);
       
    if(err != ERR_OK)
    {
      udp_remove(telemetry_Debug_Port);
    }
  }
}


extern inline void UDP_Deinit(void)
{
  if(telemetry_Data_Port != NULL)
  {
    udp_disconnect(telemetry_Data_Port);
    udp_remove(telemetry_Data_Port);
    telemetry_Data_Port = NULL;
  }
  
  if(dcu_State_Port != NULL)
  {
    udp_disconnect(dcu_State_Port);
    udp_remove(dcu_State_Port);
    dcu_State_Port = NULL;
  }
  
  if(dcu_Error_Port != NULL)
  {
    udp_disconnect(dcu_Error_Port);
    udp_remove(dcu_Error_Port);
    dcu_Error_Port = NULL;
  }
  
  if(telemetry_Command_port != NULL)
  {
    udp_disconnect(telemetry_Command_port);
    udp_remove(telemetry_Command_port);
    telemetry_Command_port = NULL;
  }
  
  if(telemetry_Debug_Port != NULL)
  {
    udp_disconnect(telemetry_Debug_Port);
    udp_remove(telemetry_Debug_Port);
    telemetry_Debug_Port = NULL;
  }
}


/* Usare questa funzione per mandare pochi pacchetti, perché non implementa nessuna gestione logica
di una coda. Quindi, per assicurarsi che non venga perso nessun pacchetto, bisgona assicurarsi che
i dati già presenti nella DMA (buffer ethernet) siano già stati inviato (non implementato). */
extern inline void UDP_Send(uint8_t *msg, struct udp_pcb *pcb)
{
  uint32_t len;
  
  len = strlen((char *)msg);
  
  // pbuf data is stored in RAM, used for TX mostly.
  // Struct pbuf and its payload are allocated in one piece of contiguous memory. 
  // pbuf_alloc() allocates PBUF_RAM pbufs as unchained pbufs. This should be used for all OUTGOING packets (TX).
  local_Buffer = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
  
  if(local_Buffer != NULL)
  {
    // Copy application supplied data into a pbuf.
    // This function can only be used to copy the equivalent of buf->tot_len data.    
    if(pbuf_take(local_Buffer, msg, len) != ERR_OK)
    {
      UDP_Send_Error(ETHERNET_TX_ERROR);
    }
    
    else
    {
      if(HAL_ETH_GetState(&heth) == HAL_ETH_STATE_READY)
      {
        if(udp_send(pcb, local_Buffer) != ERR_OK)
        {
          UDP_Send_Error(ETHERNET_TX_ERROR);  
        }
      }
      
      // Dereference a pbuf chain or queue and deallocate any no-longer-used pbufs at the head of this chain or queue.
      // Decrements the pbuf reference count. If it reaches zero, the pbuf is deallocated.
      // For a pbuf chain, this is repeated for each pbuf in the chain, up to the first pbuf which has a non-zero reference 
      // count after decrementing. So, when all reference counts are one, the whole chain is free'd.
      // Returns the number of pbufs that were de-allocated from the head of the chain.
      // MUST NOT be called on a packet queue.
      pbuf_free(local_Buffer);
      local_Buffer = NULL;
    }
  }
  
  else
  {
    UDP_Send_Error(ETHERNET_TX_ERROR);
  }
}


extern inline void UDP_Send_Len(uint8_t *msg, struct udp_pcb *pcb, uint16_t len)
{
  local_Buffer = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
  
  if(local_Buffer != NULL)
  { 
    if(pbuf_take(local_Buffer, msg, len) != ERR_OK)
    {
      UDP_Send_Error(ETHERNET_TX_ERROR);
    }
    
    else
    {
      if(HAL_ETH_GetState(&heth) == HAL_ETH_STATE_READY)
      {
        if(udp_send(pcb, local_Buffer) != ERR_OK)
        {
          UDP_Send_Error(ETHERNET_TX_ERROR);  
        }
      }
      
      pbuf_free(local_Buffer);
      local_Buffer = NULL;
    }
  }
  
  else
  {
    UDP_Send_Error(ETHERNET_TX_ERROR);
  }
}


extern inline void UDP_Send_Error(uint8_t error_Code)
{
  dcu_Error_Buffer[1] = error_Code;
  UDP_Send_Queue(UDP_DCU_ERROR_PORT, dcu_Error_Buffer, BUFFER_ERROR_LEN);
}


/* Usare questa funzione per mettere un pacchetto nella coda di trasmissione, che viene gestita
in modalità polling dalla relativa funzione richiamata nel main. Usare questa modalità quando
bisogna mandare molti pacchetti. */
extern inline void UDP_Send_Queue(const uint16_t port, uint8_t *data, const uint16_t len)
{  
  if(UDP_Queue_Buffer_Index > (UDP_SEND_BUFFER_LEN - 1))
  {
    UDP_Queue_Buffer_Index = 0;
    UDP_Send_Error(ETHERNET_TX_QUEUE_ERROR);
  }

  if(UDP_Queue_Buffer_Index == (UDP_SEND_BUFFER_LEN - 1))
  {
    UDP_Queue_Buffer_Index = 0;
  }

  UDP_Send_Buffer[UDP_Queue_Buffer_Index].port = port;
  UDP_Send_Buffer[UDP_Queue_Buffer_Index].packet_Data = data;
  UDP_Send_Buffer[UDP_Queue_Buffer_Index].len = len;
  UDP_Send_Buffer[UDP_Queue_Buffer_Index].to_Write = TO_WRITE;
  UDP_Queue_Buffer_Index++;
}


/* Funzione di gestione dell'invio dei pacchetti. Bisgona richiamarla continuamente nel main,
all'interno del while: la logica della coda è basata sul pooling continuo della coda di
pacchetti, per inviare quelli rimasti in sospeso. La funzione invia effettivamente un pacchetto, 
prendendolo dalla coda e valuntando il flag di scrittura del relativo pacchetto. Ogni volta che 
viene richiamata la funzione, invia il pacchetto successivo nella coda. L'array di strutture che 
rappresentana i pacchetti esiste sempre in memoria, anche se la coda è vuota (indice di send nullo). */
extern inline void UDP_Send_Processig(void)
{
  if(UDP_Send_Buffer_Index > (UDP_SEND_BUFFER_LEN - 1))
  {
    UDP_Send_Error(ETHERNET_TX_QUEUE_ERROR);
    UDP_Send_Buffer_Index = 0;
    return;
  }
  
  if(UDP_Send_Buffer_Index == (UDP_SEND_BUFFER_LEN - 1))
  {
    UDP_Send_Buffer_Index = 0;
  }

  if(HAL_ETH_GetState(&heth) == HAL_ETH_STATE_READY)
  {
    if(UDP_Send_Buffer[UDP_Send_Buffer_Index].to_Write == TO_WRITE)
    {
      send_Processing_Len = UDP_Send_Buffer[UDP_Send_Buffer_Index].len;
      send_Processing_Tx_Packet = pbuf_alloc(PBUF_TRANSPORT, send_Processing_Len, PBUF_RAM);
      
      if(send_Processing_Tx_Packet != NULL)
      {
        if(pbuf_take(send_Processing_Tx_Packet, UDP_Send_Buffer[UDP_Send_Buffer_Index].packet_Data, send_Processing_Len) != ERR_OK)
        {
          UDP_Send_Error(ETHERNET_TX_ERROR);
        }
        
        else
        {
          send_Processing_Tx_Pcb = get_Udp_Pcb(UDP_Send_Buffer[UDP_Send_Buffer_Index].port);

          if(udp_send(send_Processing_Tx_Pcb, send_Processing_Tx_Packet) != ERR_OK)
          {
            UDP_Send_Error(ETHERNET_TX_ERROR);
          }
          
          UDP_Send_Buffer[UDP_Send_Buffer_Index].port = 0;
          UDP_Send_Buffer[UDP_Send_Buffer_Index].to_Write = DIRTY;
          pbuf_free(send_Processing_Tx_Packet);
          send_Processing_Tx_Pcb = NULL;
          send_Processing_Tx_Packet = NULL;
          UDP_Send_Buffer_Index++;
        }
      }
      
      else
      {
        UDP_Send_Error(ETHERNET_TX_ERROR);
      }
    }
  }
}


/* Funzione di gestione della ricezione dei pacchetti. Bisgona richiamarla continuamente 
nel main, all'interno del while: la logica della coda è basata sul pooling continuo della 
coda di pacchetti, per elaborare quelli ricevuti e rimasti in sospeso. */
extern inline void UDP_Receive_Processig(void)
{
  char *data_Received = NULL;
  
  if(UDP_Read_Buffer_Index > (UDP_RECEIVE_BUFFER_LEN - 1))
  {
    UDP_Read_Buffer_Index = 0;
    UDP_Send_Error(ETHERNET_RX_QUEUE_ERROR);
    return;
  }
  
  if(UDP_Read_Buffer_Index == (UDP_RECEIVE_BUFFER_LEN - 1))
  {
    UDP_Read_Buffer_Index = 0;
  }
  
  if(UDP_Receive_Buffer[UDP_Read_Buffer_Index].to_Read == TO_READ)
  {
    data_Received = UDP_Receive_Buffer[UDP_Read_Buffer_Index].buffer;
    UDP_Packet_Comand(data_Received);
    UDP_Receive_Buffer[UDP_Read_Buffer_Index].to_Read = DIRTY;
    UDP_Read_Buffer_Index++;
  }
}


extern void netif_status_callback(struct netif *netif)
{
  UDP_Deinit();
  UDP_Init();
}


/* Funzione di callback, richiamata dalla libreria quando viene ricevuto un pacchetto. La ricezione
di un pacchetto avviene mediante polling, con una funzione di libreria richiamata nel while del main.
Questa funzione si occupa di mettere nells coda di ricezione il pacchetto ricevuto, in attesa di essere
successivamente elaborato nel main. */
void UDP_Receive_Telemetry_Command(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
  if(UDP_Receive_Buffer_Index > (UDP_RECEIVE_BUFFER_LEN - 1))
  {
    UDP_Receive_Buffer_Index = 0;
    pbuf_free(p);
    UDP_Send_Error(ETHERNET_RX_QUEUE_ERROR);
    return;
  }

  if(UDP_Receive_Buffer_Index == (UDP_RECEIVE_BUFFER_LEN - 1))
  {
    UDP_Receive_Buffer_Index = 0;
  }
  
  rx_Dataptr = p->payload;
  rx_Data = &(((char*)rx_Dataptr)[0]);
  UDP_Receive_Buffer[UDP_Receive_Buffer_Index].buffer = rx_Data;
  UDP_Receive_Buffer[UDP_Receive_Buffer_Index].to_Read = TO_READ;
  UDP_Receive_Buffer_Index++;
  pbuf_free(p);
  p = NULL;
}


void ethernetif_notify_conn_changed(struct netif *netif)
{
  ip_addr_t ipaddr;
  ip_addr_t netmask;
  ip_addr_t gw;

  if(netif_is_link_up(netif))
  {
    IP_ADDR4(&ipaddr, IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
    IP_ADDR4(&netmask, NETMASK_ADDRESS[0], NETMASK_ADDRESS[1] , NETMASK_ADDRESS[2], NETMASK_ADDRESS[3]);
    IP_ADDR4(&gw, GATEWAY_ADDRESS[0], GATEWAY_ADDRESS[1], GATEWAY_ADDRESS[2], GATEWAY_ADDRESS[3]); 
    netif_set_addr(netif, &ipaddr , &netmask, &gw);
    netif_set_up(netif);
  }
  
  else
  {
    netif_set_down(netif);
  }
}


static inline struct udp_pcb *get_Udp_Pcb(const uint16_t port)
{  
  switch(port)
  {
    case UDP_TELEMETRY_DATA_PORT:
      return telemetry_Data_Port;
    
    case UDP_DCU_STATE_PORT:
      return dcu_State_Port;
    
    case UDP_DCU_ERROR_PORT:
      return dcu_Error_Port;
    
    case UDP_TELEMETRY_DEBUG_PORT:
      return telemetry_Debug_Port;
  }
  
  return NULL;
}

