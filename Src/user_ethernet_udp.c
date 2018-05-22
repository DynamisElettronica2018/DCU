#include <string.h>
#include "user_defines.h"
#include "user_externalVariables.h"
#include "telemetry_command.h"
#include "user_ethernet_udp.h"
#include "user_usb.h"


struct udp_pcb *telemetry_Data_Port;
struct udp_pcb *dcu_State_Port;
struct udp_pcb *dcu_Error_Port;
struct udp_pcb *telemetry_Command_port;
struct udp_pcb *telemetry_Debug_Port;
struct udp_pcb *pcb_Temp = NULL;
struct udp_pcb *to_Return = NULL;
struct pbuf *local_Buffer = NULL;
struct pbuf *UDP_Send_packet = NULL;


extern void UDP_Init(void)
{
  ip_addr_t DestIPaddr;
  err_t err;
  
  // TX buffer initialization
  for(uint16_t i = 0; i < UDP_SEND_BUFFER_LEN; i++)
  {
    UDP_Send_Buffer[i].pcb = telemetry_Data_Port;
    UDP_Send_Buffer[i].to_Write = DIRTY;
  }
  
  // RX buffer initialization 
  for(uint16_t i = 0; i < UDP_RECEIVE_BUFFER_LEN; i++)
  {
    UDP_Receive_Buffer[i].buffer = NULL;
    UDP_Receive_Buffer[i].to_Read = DIRTY;
  }

  
  // Creates a new UDP pcb for specific IP type, which can be used for UDP communication.
  // The pcb is not active until it has either been bound to a local address or connected to a remote address.
  telemetry_Data_Port = udp_new();
  dcu_State_Port = udp_new();
  dcu_Error_Port = udp_new();
  telemetry_Command_port = udp_new();
  telemetry_Debug_Port = udp_new();
  
  // telemetry_Command_port
  if(telemetry_Command_port != NULL)
  {
    // Binds the pcb to a local address.
    // The IP-address argument can be IP_ADDR_ANY to indicate that it should listen to any local IP address. 
    // The function currently always return ERR_OK.
    err = udp_bind(telemetry_Command_port, IP_ADDR_ANY, UDP_TELEMETRY_COMMAND_PORT);
    
    if(err == ERR_OK)
    {       
      // Specifies a callback function that should be called when a UDP datagram is received.
      udp_recv(telemetry_Command_port, UDP_Receive_Callback, NULL);
    }
    
    else
    {
      udp_remove(telemetry_Command_port);
    }
  }

  
  // telemetry_Data_Port
  if(telemetry_Data_Port != NULL)
  {
    // Sets the remote end of the pcb.
    // This function does not generate any network traffic, but set the remote address of the pcb.
    IP4_ADDR(&DestIPaddr, DEST_IP_ADDR0, DEST_IP_ADDR1, DEST_IP_ADDR2, DEST_IP_ADDR3);
    err = udp_connect(telemetry_Data_Port, &DestIPaddr, UDP_TELEMETRY_DATA_PORT);
       
    if(err != ERR_OK)
    {
      udp_remove(telemetry_Data_Port);
    }
  }

  
  // dcu_State_Port
  if(dcu_State_Port != NULL)
  {
    IP4_ADDR(&DestIPaddr, DEST_IP_ADDR0, DEST_IP_ADDR1, DEST_IP_ADDR2, DEST_IP_ADDR3);
    err = udp_connect(dcu_State_Port, &DestIPaddr, UDP_DCU_STATE_PORT);
       
    if(err != ERR_OK)
    {
      udp_remove(dcu_State_Port);
    }
  }

  
  // dcu_Error_Port
  if(dcu_Error_Port != NULL)
  {
    IP4_ADDR(&DestIPaddr, DEST_IP_ADDR0, DEST_IP_ADDR1, DEST_IP_ADDR2, DEST_IP_ADDR3);
    err = udp_connect(dcu_Error_Port, &DestIPaddr, UDP_DCU_ERROR_PORT);
       
    if(err != ERR_OK)
    {
      udp_remove(dcu_Error_Port);
    }
  }
  
  
  // telemetry_Debug_Port
  if(telemetry_Debug_Port != NULL)
  {
    IP4_ADDR(&DestIPaddr, DEST_IP_ADDR0, DEST_IP_ADDR1, DEST_IP_ADDR2, DEST_IP_ADDR3);
    err = udp_connect(telemetry_Debug_Port, &DestIPaddr, UDP_TELEMETRY_DEBUG_PORT);
       
    if(err != ERR_OK)
    {
      udp_remove(telemetry_Debug_Port);
    }
  }
}


/* Usare questa funzione per mandare pochi pacchetti, perché non implementa nessuna gestione logica
di una coda. Quindi, per assicurarsi che non venga perso nessun pacchetto, bisgona assicurarsi che
i dati già presenti nella DMA (buffer ethernet) siano già stati inviato (non implementato). */
extern inline void UDP_Send(uint8_t *msg, struct udp_pcb *pcb)
{
  uint16_t len;
  err_t pbuf_Error_Code;
  err_t send_Error_Code;
  
  len = strlen((char *)msg);
  
  // pbuf data is stored in RAM, used for TX mostly.
  // Struct pbuf and its payload are allocated in one piece of contiguous memory. 
  // pbuf_alloc() allocates PBUF_RAM pbufs as unchained pbufs. This should be used for all OUTGOING packets (TX).
  // Don't use PBUF_POOL for TX. Do not use heap in interrupt functions, so use PBUF_POOL.
  local_Buffer = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
  
  // TX packet
  if(local_Buffer != NULL)
  {    
    // Copy application supplied data into a pbuf.
    // This function can only be used to copy the equivalent of buf->tot_len data.
    pbuf_Error_Code = pbuf_take(local_Buffer, msg, len);
    
    if(pbuf_Error_Code != ERR_OK)
    {
      UDP_Send_Error(ETHERNET_TX_ERROR);
    }
    
    else
    {
      send_Error_Code = udp_send(pcb, local_Buffer);
      
      if(send_Error_Code != ERR_OK)
      {
        UDP_Send_Error(ETHERNET_TX_ERROR);  
      }
      
      // Dereference a pbuf chain or queue and deallocate any no-longer-used pbufs at the head of this chain or queue.
      // Decrements the pbuf reference count. If it reaches zero, the pbuf is deallocated.
      // For a pbuf chain, this is repeated for each pbuf in the chain, up to the first pbuf which has a non-zero reference 
      // count after decrementing. So, when all reference counts are one, the whole chain is free'd.
      // Returns the number of pbufs that were de-allocated from the head of the chain.
      // MUST NOT be called on a packet queue.
      pbuf_free(local_Buffer);
    }
  }
  
  else
  {
    UDP_Send_Error(ETHERNET_TX_ERROR);
  }
}


extern inline void UDP_Send_Error(uint8_t error_Code)
{
  uint8_t dcu_Error_Buffer[BUFFER_ERROR_LEN];
  
  dcu_Error_Buffer[0] = 'E';
  dcu_Error_Buffer[1] = error_Code;
  dcu_Error_Buffer[2] = '\0';
  UDP_Send_Queue(dcu_Error_Buffer, BUFFER_ERROR_LEN, UDP_DCU_ERROR_PORT);
}


/* Usare questa funzione per mettere un pacchetto nella coda di trasmissione, che viene gestita
in modalità polling dalla relativa funzione richiamata nel main. Usare questa modalità quando
bisogna mandare molti pacchetti. */
// Gestisce solo pacchetti di lenghezza fissata, pari a UDP_TELEMETRY_DATA_LEN e usa upcb_Telemetry_Data.
extern inline void UDP_Send_Queue(uint8_t *msg, uint16_t len, uint16_t port)
{
  err_t queue_Error;
  
  UDP_Send_packet = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
  
  // Pacchetto nella coda
  if(UDP_Send_packet != NULL)
  {
    queue_Error = pbuf_take(UDP_Send_packet, msg, len);
    
    if(queue_Error != ERR_OK)
    {
      UDP_Send_Error(ETHERNET_TX_ERROR);
    }
    
    else
    {
      // Passo direttamente una copia del pacchetto, non il suo puntatore.
      // Così lo posso distruggere senza rischiare di perdere il messaggio.
      queue_Error = UDP_Queue_Put(*UDP_Send_packet, port);

      if(queue_Error != ERR_OK)
      {
        UDP_Send_Error(ETHERNET_TX_QUEUE_ERROR);
      }

      pbuf_free(UDP_Send_packet);
    }
  }
  
  else
  {
    UDP_Send_Error(ETHERNET_TX_ERROR);
  }
}


/* Funzione di gestione dell'invio dei pacchetti. Bisgona richiamarla continuamente nel main,
all'interno del while: la logica della coda è basata sul pooling continuo della coda di
pacchetti, per inviare quelli rimasti in sospeso. La funzione invia effettivamente un pacchetto, 
prendendolo dalla coda e valuntando il flag di scrittura del relativo pacchetto. Ogni volta che 
viene richiamata la funzione, invia il pacchetto successivo nella coda. L'array di strutture che 
rappresentana i pacchetti esiste sempre in memoria, anche se la coda è vuota (indice di send nullo). */
extern inline void UDP_Send_Processig(void)
{
  err_t send_Processing_Error;
  
  if(UDP_Send_Buffer_Index > (UDP_SEND_BUFFER_LEN - 1))
  {
    UDP_Send_Error(ETHERNET_TX_QUEUE_ERROR);
    UDP_Send_Buffer_Index = 0;
  }
  
  if(UDP_Send_Buffer_Index == (UDP_SEND_BUFFER_LEN - 1))
  {
    UDP_Send_Buffer_Index = 0;
  }

  if(UDP_Send_Buffer[UDP_Send_Buffer_Index].to_Write == TO_WRITE)
  {
    pcb_Temp = UDP_Send_Buffer[UDP_Send_Buffer_Index].pcb;
    send_Processing_Error = udp_send(pcb_Temp, &(UDP_Send_Buffer[UDP_Send_Buffer_Index].buffer));
    UDP_Send_Buffer[UDP_Send_Buffer_Index].to_Write = DIRTY;
    UDP_Send_Buffer_Index++;
    
    if(send_Processing_Error != ERR_OK)
    {
      UDP_Send_Error(ETHERNET_TX_ERROR);
    }
  }
}


/* Funzione di gestione della ricezione dei pacchetti. Bisgona richiamarla continuamente 
nel main, all'interno del while: la logica della coda è basata sul pooling continuo della 
coda di pacchetti, per elaborare quelli ricevuti e rimasti in sospeso. */
extern inline void UDP_Receive_Processig(void)
{
  if(UDP_Read_Buffer_Index == (UDP_RECEIVE_BUFFER_LEN - 1))
  {
    UDP_Read_Buffer_Index = 0;
  }

  if(UDP_Read_Buffer_Index > (UDP_RECEIVE_BUFFER_LEN - 1))
  {
    UDP_Read_Buffer_Index = 0;
    UDP_Send_Error(ETHERNET_RX_QUEUE_ERROR);
    return;
  }

  if(UDP_Receive_Buffer[UDP_Read_Buffer_Index].to_Read == TO_READ)
  {
    char *data = UDP_Receive_Buffer[UDP_Read_Buffer_Index].buffer;
    UDP_Packet_Comand(data);
    UDP_Receive_Buffer[UDP_Read_Buffer_Index].to_Read = DIRTY;
    UDP_Read_Buffer_Index++;
  }
}


static inline err_t UDP_Queue_Put(struct pbuf UDP_Send_packet, uint16_t port)
{
  err_t queue_Put_Error;

  if(UDP_Queue_Buffer_Index == (UDP_SEND_BUFFER_LEN - 1))
  {
    UDP_Queue_Buffer_Index = 0;
  }

  if(UDP_Queue_Buffer_Index > (UDP_SEND_BUFFER_LEN - 1))
  {
    UDP_Queue_Buffer_Index = 0;
    queue_Put_Error = ERR_MEM;
    return queue_Put_Error;
  }

  UDP_Send_Buffer[UDP_Queue_Buffer_Index].buffer = UDP_Send_packet;
  UDP_Send_Buffer[UDP_Queue_Buffer_Index].pcb = UDP_Get_Pcb(port);
  UDP_Send_Buffer[UDP_Queue_Buffer_Index].to_Write = TO_WRITE;
  UDP_Queue_Buffer_Index++;
  queue_Put_Error = ERR_OK;
  return queue_Put_Error;
}


static inline struct udp_pcb *UDP_Get_Pcb(uint16_t port)
{
  to_Return = NULL;
  
  switch(port)
  {
    case UDP_TELEMETRY_DATA_PORT:
      to_Return = telemetry_Data_Port;
      break;
    
    case UDP_DCU_STATE_PORT:
      to_Return = dcu_State_Port;
      break;
    
    case UDP_DCU_ERROR_PORT:
      to_Return = dcu_Error_Port;
      break;
    
    case UDP_TELEMETRY_DEBUG_PORT:
      to_Return = telemetry_Debug_Port;
      break;
  }
  
  return to_Return;
}


/* Funzione che gestisce i comandi ricevuto su ethernet, inviati da telemetry
Viene chiamata nella funzione di gestione della coda di pacchetti ricevuti
Il pacchetto di comando deve assolutamente avere il carattare di terminazione '\0' */
static inline void UDP_Packet_Comand(char *data)
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
      
      case DAU_ALL_BOARD_CODE:
        break;
      
      case DAU_FL_BOARD_CODE:
        break;
      
      case DAU_FR_BOARD_CODE:
        break;
      
      case DAU_REAR_BOARD_CODE:
        break;

      case GCU_BOARD_CODE:
        break;
    }
	}
}


static inline void dcu_Board_Command(char *command)
{
  switch(command[1])
  {
    case START_TELEMETRY_COMMAND:
      telemetry_On = UDP_DCU_STATE_OK;
      break;
    
    case STOP_TELEMETRY_COMMAND:
      telemetry_On = UDP_DCU_STATE_ERROR;
      break;
    
    case START_ACQUISITION_COMMAND:
      if(acquisition_On == UDP_DCU_STATE_ERROR)
      {
        USB_Get_File_Name(USB_Filename);
        USB_User_Start(USB_Filename);
        acquisition_On = UDP_DCU_STATE_OK;
      }
      
      break;
    
    case STOP_ACQUISITION_COMMAND:
      if(acquisition_On == UDP_DCU_STATE_OK)
		  {
        USB_User_Stop();
        acquisition_On = UDP_DCU_STATE_ERROR;
      }
      
      break;
    
    case START_ACQUISITION_AND_TELEMETRY:
      if(acquisition_On == UDP_DCU_STATE_ERROR)
      {
        USB_Get_File_Name(USB_Filename);
        USB_User_Start(USB_Filename);
        acquisition_On = UDP_DCU_STATE_OK;
      }

      telemetry_On = UDP_DCU_STATE_OK;
      break;
    
    case STOP_ACQUISITION_AND_TELEMETRY:
      if(acquisition_On == UDP_DCU_STATE_OK)
		  {
        USB_User_Stop();
        acquisition_On = UDP_DCU_STATE_ERROR;
      }
      
      telemetry_On = UDP_DCU_STATE_ERROR;
      break;
    
    case START_DEBUG_COMMAND:
      break;
    
    case STOP_DEBUG_COMMAND:
      break;
    
    case SET_RTC_TIME:
      break;
    
    case SET_RTC_DATA:
      break;
  }
}


/* Funzione di callback, richiamata dalla libreria quando viene ricevuto un pacchetto. La ricezione
di un pacchetto avviene mediante polling, con una funzione di libreria richiamata nel while del main.
Questa funzione si occupa di mettere nells coda di ricezione il pacchetto ricevuto, in attesa di essere
successivamente elaborato nel main. */
void UDP_Receive_Callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
  const void *dataptr;
  char *rx_data;

  if(UDP_Receive_Buffer_Index == (UDP_RECEIVE_BUFFER_LEN - 1))
  {
    UDP_Receive_Buffer_Index = 0;
  }

  if(UDP_Receive_Buffer_Index > (UDP_RECEIVE_BUFFER_LEN - 1))
  {
    UDP_Send_Error(ETHERNET_RX_QUEUE_ERROR);
    UDP_Receive_Buffer_Index = 0;
    pbuf_free(p);
  }

  dataptr = p->payload;
  rx_data = &(((char*)dataptr)[0]);
  UDP_Receive_Buffer[UDP_Receive_Buffer_Index].buffer = rx_data;
  UDP_Receive_Buffer[UDP_Receive_Buffer_Index].to_Read = TO_READ;
  UDP_Receive_Buffer_Index++;
  pbuf_free(p);
}


void ethernetif_notify_conn_changed(struct netif *netif)
{
  ip_addr_t ipaddr;
  ip_addr_t netmask;
  ip_addr_t gw;

  if(netif_is_link_up(netif))
  {
    IP_ADDR4(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
    IP_ADDR4(&netmask, NETMASK_ADDR0, NETMASK_ADDR1 , NETMASK_ADDR2, NETMASK_ADDR3);
    IP_ADDR4(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3); 
    netif_set_addr(netif, &ipaddr , &netmask, &gw);
    netif_set_up(netif);
  }
  
  else
  {
    netif_set_down(netif);
  }
}
