#include <stdio.h>
#include "user_defines.h"
#include "user_externalVariables.h"
#include "CAN_ID_Defines.h"
#include "telemetry_command.h"
#include "data.h"
#include "user_ethernet_udp.h"


uint8_t bufferBlock1[BUFFER_BLOCK_LEN];
uint8_t bufferBlock2[BUFFER_BLOCK_LEN];
uint8_t dcu_State_Packet[BUFFER_STATE_LEN];
uint8_t *bufferBlock = NULL;
uint8_t dcu_State_Packet_Count = 0;
float fTemp = 0.0;
int16_t intTemp = 0;
uint16_t uTemp = 0;
uint16_t data0 = 0;
uint16_t data1 = 0;
uint16_t data2 = 0;
uint16_t data3 = 0;
uint16_t data4 = 0;
uint16_t data5 = 0;
uint16_t data6 = 0;
uint16_t data7 = 0;


// Funzione che inserisce i 21 separatori di canali nel buffer
// Da chiamare una sola volta per inizializzare una riga del file CSV
extern void initializeData(void)
{
  usbBlockWriteFlag = 0;
  timestamp = 0;
  bufferBlock = bufferBlock1;
	bufferBlockWrite = bufferBlock2;

  for(uint16_t i = 0; i < BUFFER_BLOCK_LEN; i++)
  {
    bufferBlock1[i] = '0';
    bufferBlock2[i] = '0';
  }

  // Inizializzazione buffer USB
  // DA FARE CON DEFINES
}


// Funzione di conversione dati, identificati in base all'ID del relativo pacchetto CAN
// La funzione va richiamata nella callback di ricezione pacchetti CAN, con gli opportuni parametri
// Il primo parametro è l'ID del pacchetto da convertire e il secondo è l'array degli 8 byte di dato
extern inline void dataConversion(uint16_t ID, uint8_t payload[8])
{
	data0 = payload[0] & 0x00FF;
  data1 = payload[1] & 0x00FF;
  data2 = payload[2] & 0x00FF;
  data3 = payload[3] & 0x00FF;
  data4 = payload[4] & 0x00FF;
  data5 = payload[5] & 0x00FF;
  data6 = payload[6] & 0x00FF;
  data7 = payload[7] & 0x00FF;
  
  switch(ID)
	{
		case IMU_DATA_1:

			// ACC X byte 0-1 (matrix byte da 8 a 14)
      uTemp = ((data0 << 8 ) & 0xFF00) | data1;
      decimalToString((int16_t)uTemp, &bufferBlock[8], 3, 2);
    
			// ACC Y byte 2-3 (matrix da 16 a 22)
      uTemp = ((data2 << 8 ) & 0xFF00) | data3;
      decimalToString((int16_t)uTemp, &bufferBlock[16], 3, 2);
							
			// GYR X byte 4-5 (matrix da 24 a 30)
			uTemp = ((data4 << 8 ) & 0xFF00) | data5;
      decimalToString((int16_t)uTemp, &bufferBlock[24], 3, 2);
							
			// GYR Y byte 6-7 (payload da 32 a 38)
			uTemp = ((data6 << 8 ) & 0xFF00) | data7;
      decimalToString((int16_t)uTemp, &bufferBlock[32], 3, 2);

			break;

		case IMU_DATA_2:
			
			// HEADING byte 0-1 (matrix byte da 40 a 46)
			uTemp = ((data0 << 8 ) & 0xFF00) | data1;
      fTemp = (float)uTemp;
      fTemp = fTemp / 16;
      intToStringUnsigned((int16_t)fTemp, &bufferBlock[40], 4);
						
			// ACC Z byte 2-3 (matrix byte da 48 a 54)
      uTemp = ((data2 << 8 ) & 0xFF00) | data3;
      decimalToString((int16_t)uTemp, &bufferBlock[45], 3, 2);
						
			// GYR Y byte 4-5 (matrix byte da 56 a 62)
			uTemp = ((data4 << 8 ) & 0xFF00) | data5;
      decimalToString((int16_t)uTemp, &bufferBlock[53], 3, 2);
			
			break;
						
		case IMU_DATA_3:

			// GPS X byte 0-1 (matrix byte da 64 a 68)
			uTemp = ((data0 << 8 ) & 0xFF00) | data1;
      intToStringUnsigned((uint16_t)uTemp, &bufferBlock[61], 5);		

			// GPS Y byte 2-3 (matrix byte da 70 a 74)
			uTemp = ((data2 << 8 ) & 0xFF00) | data3;
      intToStringUnsigned((uint16_t)uTemp, &bufferBlock[67], 5);
			
			// VELOCITY byte 4-5 (matrix byte da 76 a 81)
			uTemp = ((data4 << 8 ) & 0xFF00) | data5;
      decimalToStringUnsigned((uint16_t)uTemp, &bufferBlock[73], 3, 2);
						
			break;

		case GCU_DATA_1:
      
      // H20 DUTY CYCLE byte 0-1 (matrix byte da 83 a 85)
      uTemp = ((data0 << 8 ) & 0xFF00) | data1;
			intToStringUnsigned((int16_t)uTemp, &bufferBlock[80], 3);
			
			//H20 ENGINE byte 2-3 (matrix byte da 87 a 89)
			uTemp = ((data2 << 8 ) & 0xFF00) | data3;
			fTemp = (float)uTemp;
      fTemp = (fTemp / ((float)1.6)) - 10;
      intToStringUnsigned((int16_t)fTemp, &bufferBlock[84], 3);
      
			//H20 IN byte 4-5 (matrix byte da 91 a 93)
			uTemp = ((data4 << 8 ) & 0xFF00) | data5;
			fTemp = (float)uTemp;
			fTemp = (fTemp * (-0.359381)) + 192.396;
			intToString((int16_t)fTemp, &bufferBlock[88], 3);
						
			// H20 OUT byte 6-7 (matrix byte da 95 a 97)
			uTemp = ((data6 << 8 ) & 0xFF00) | data7;
			fTemp = (float)uTemp;
			fTemp = (fTemp * (-0.359381)) + 192.396;
			intToStringUnsigned((int16_t)fTemp, &bufferBlock[92], 3);
						
			break;
						
		case GCU_DATA_2:

      // OIL PRESSURE byte 0-1 (matrix byte da 99 a 103)
			uTemp = ((data0 << 8 ) & 0xFF00) | data1;
			intToStringUnsigned((uint16_t)uTemp, &bufferBlock[96], 5);
		
			// T OIL IN byte 2-3 (matrix byte da 105 a 107)
			uTemp = ((data2 << 8 ) & 0xFF00) | data3;
			fTemp = (float)uTemp;
			fTemp = (fTemp * (-0.359381)) + 192.396;
			intToStringUnsigned((uint16_t)fTemp, &bufferBlock[102], 3);
						
			// T OIL OUT byte 4-5 (matrix byte da 109 a 111)
			uTemp = ((data4 << 8 ) & 0xFF00) | data5;
			fTemp = (float)uTemp;
			fTemp = (fTemp * (-0.359381)) + 192.396;
			intToStringUnsigned((uint16_t)fTemp, &bufferBlock[106], 3);
					
			// BATTERY byte 6-7 (matrix byte da 113 a 114)
			uTemp = ((data6 << 8 ) & 0xFF00) | data7;
			fTemp = (float)uTemp;
      uTemp = ((uint16_t)(((fTemp * 18) / 1024) * 100));
			decimalToStringUnsigned(uTemp, &bufferBlock[110], 2, 2);
				
			break;
						
		case GCU_DATA_3:

			// FUEL PRESSURE byte 0-1 (matrix byte da 116 a 120)
			uTemp = ((data0 << 8 ) & 0xFF00) | data1;
      intToStringUnsigned((uint16_t)uTemp, &bufferBlock[116], 5);
						
			// FAN byte 2-3 (matrix byte 122)
			uTemp = ((data2 << 8 ) & 0xFF00) | data3;
			intToStringUnsigned((uint16_t)uTemp, &bufferBlock[122], 1);
						
			// INJ 1 DIAG byte 4-5 (matrix byte 124)
			uTemp = ((data4 << 8 ) & 0xFF00) | data5;
			intToStringUnsigned((uint16_t)uTemp, &bufferBlock[124], 1);
						
			// INJ 2 DIAG byte 6-7 (matrix byte 126)
			uTemp = ((data6 << 8 ) & 0xFF00) | data7;
			intToStringUnsigned((uint16_t)uTemp, &bufferBlock[126], 1);

			break;
  }
}


// Funzione da richiamare dentro a un timer con periodo di 10 ms
// Gestisce la scrittura su chiavetta USB
extern inline void data10msTimerBackground(void)
{
	uint8_t *temp;
  
  uint32ToString(timestamp, &bufferBlock[0], 7);  
  timestamp += 10;
  
  // Scambio il blocco da scrivere con i nuovi dati con quello appena riempito
  // Questo è necessario per non disturbare la funzioni di scrittura su USB
	// Se comincio a scrivere lo stesso buffer con nuovi dati, perdo quelli precedenti
  temp = bufferBlockWrite;
	bufferBlockWrite = bufferBlock;
	bufferBlock = temp;
  
  // Il buffer che contiene un blocco da 512 byte è pieno, quindi lo salvo
	// per avviare il salvataggio su chiavetta USB, setto un flag che viene controllato nel main 
	// La funzioni di salvataggio è infatti bloccante e non può essere utilizzata nell'interrupt
	usbBlockWriteFlag = 1;
}


// Funzione da richiamare dentro a un timer con periodo di 100 ms
// Gestisce l'invio del pacchetto ethernet
extern inline void data100msTimerBackground(void)
{
  UDP_Send_Queue(bufferBlock, BUFFER_BLOCK_LEN, UDP_TELEMETRY_DATA_PORT);
  dcu_State_Packet_Count++;
  
  if(dcu_State_Packet_Count >= DCU_STATE_PACKET_PERIOD)
  {
    dcu_State_Packet[0] = 'M';
    dcu_State_Packet[1] = usb_Present;
    dcu_State_Packet[2] = usb_Ready;
    dcu_State_Packet[3] = acquisition_On;
    dcu_State_Packet[4] = telemetry_On;
    dcu_State_Packet[5] = '\0';
    UDP_Send_Queue(dcu_State_Packet, BUFFER_STATE_LEN, UDP_DCU_STATE_PORT);
    dcu_State_Packet_Count = 0;
  }
}


// Funzione per convertire in stringa i numeri che sono nella forma 3 cifre intere + 2 decimali + segno
// Il formato è fisso e non può essere usata per altri tipi di conversioni
// Il punto decimale è direttamente inserio dalla funzione, quindi non scirverlo prima nel buffer
// Gestisce anche l'eventuale segno del numero
static inline void decimalToString(int16_t i, uint8_t *s, uint8_t nInt, uint8_t nDec)
{
	int16_t divResult;
	int16_t integer;
	int16_t decimal;
	uint8_t charSizeCount = 0;

  divResult = i;

	// Divido l'intero per 10 fino a quando non è nullo, ovvero conto le cifre del numero da covertire
	while(divResult != 0)
	{
		divResult /= 10;
		charSizeCount++;
	}

	if(charSizeCount > (nInt + nDec))
	{
		for(uint8_t j = 0; j < (nInt + nDec); j++)
		{
			s[j] = '*';
		}
  }

  else
  {
    integer = i / power(10, nDec);

    if(i < 0)
    {
      i *= -1;
    }

    decimal = i % power(10, nDec);
    intToString((int16_t)integer, &s[0], nInt+1);
    s[nInt+1] = DECIMAL_SEPARATOR;
    intToString((int16_t)decimal, &s[nInt+2], nDec);
  }
}


// Funzione per convertire in stringa i numeri che sono nella forma 3 cifre intere + 2 decimali
// Il formato è fisso e non può essere usata per altri tipi di conversioni
// Il punto decimale è direttamente inserio dalla funzione, quindi non scirverlo prima nel buffer
// La funzione gestisce correttamente soltanto numeri positivi, nel formato indicato
static inline void decimalToStringUnsigned(uint16_t i, uint8_t *s, uint8_t nInt, uint8_t nDec)
{
	uint16_t divResult;
	uint16_t integer;
	uint16_t decimal;
	uint8_t charSizeCount = 0;

  divResult = i;

	// Divido l'intero per 10 fino a quando non è nullo, ovvero conto le cifre del numero da covertire
	while(divResult != 0)
	{
		divResult /= 10;
		charSizeCount++;
	}

	if(charSizeCount > (nInt + nDec))
	{
		for(uint8_t j = 0; j < (nInt + nDec); j++)
		{
			s[j] = '*';
		}
  }

  else
  {
    integer = i / power(10, nDec);
    decimal = i % power(10, nDec);
    intToString((int16_t)integer, &s[0], nInt);
    s[nInt] = DECIMAL_SEPARATOR;
    intToString((int16_t)decimal, &s[nInt+1], nDec);
  }
}


// Funzione di conversione da int a stringa di caratteri
// Se il numero richiede un numero di caratteri maggiore di size, scrive '*'
static inline void intToString(int16_t i, uint8_t *s, uint8_t size)
{
	uint8_t count = 0;
	uint8_t charSizeCount = 0;
	uint8_t j = 0;
	int16_t divResult;

	// Se il numero è negatico, lo trasformo in uno positivo mettendo il meno davanti
	if(i < 0)
	{
		s[0] = '-';
		i *= -1;
		charSizeCount = 1;
    j=1;
	}

	divResult = i;

	// Divido l'intero per 10 fino a quando non è nullo, ovvero conto le cifre del numero da covertire
	while((divResult != 0) || (count == 0))
	{
		divResult = divResult / 10;
		count++;
		charSizeCount++;
	}

	divResult = i;

	// Controllo se il nuemro da convertire richiede più spazio di quello voluto (parametro size)
	// Se lo richiede scrivo sempre '*', alrimenti lo converto nella stringa
	if(charSizeCount > size)
  {
		for(j = 0; j < size; j++)
		{
			s[j] = '*';
		}
	}

	else
	{
    
		// Se l'intero richiede un numero di cifre inferiore a quello voluto lo completo con zeri
		while(charSizeCount < size)
		{
			s[j] = '0';
			j++;
			charSizeCount++;
		}

		// Include nella stringa il resto della divisione intera del numero per 10
		// Riduco in numero di un ordine di grandezza e il resto lo includo nell'iterazione successiva
		for(j = 0; j < count; j++)
		{
			s[(charSizeCount - 1) - j] = (divResult % 10) + '0';
			divResult /= 10;
		}
	}
}


static inline void intToStringUnsigned(uint16_t i, uint8_t *s, uint8_t size)
{
	uint8_t count = 0;
	uint8_t charSizeCount = 0;
	uint8_t j = 0;
	uint16_t divResult;

  divResult = i;

	// Divido l'intero per 10 fino a quando non è nullo, ovvero conto le cifre del numero da covertire
	while((divResult != 0) || (count == 0))
	{
		divResult = divResult / 10;
		count++;
		charSizeCount++;
	}

	divResult = i;

	// Controllo se il nuemro da convertire richiede più spazio di quello voluto (parametro size)
	// Se lo richiede scrivo sempre '*', alrimenti lo converto nella stringa
	if(charSizeCount > size)
  {
		for(j = 0; j < size; j++)
		{
			s[j] = '*';
		}
	}

	else
	{
    
		// Se l'intero richiede un numero di cifre inferiore a quello voluto lo completo con zeri
		while(charSizeCount < size)
		{
			s[j] = '0';
			j++;
			charSizeCount++;
		}

		// Include nella stringa il resto della divisione intera del numero per 10
		// Riduco in numero di un ordine di grandezza e il resto lo includo nell'iterazione successiva
		for(j = 0; j < count; j++)
		{
			s[(charSizeCount - 1) - j] = (divResult % 10) + '0';
			divResult /= 10;
		}
	}
}


static inline void uint32ToString(uint32_t i, uint8_t *s, uint8_t size)
{
  uint8_t count = 0;
	uint8_t charSizeCount = 0;
	uint8_t j = 0;
	uint32_t divResult;

	divResult = i;

	// Divido l'intero per 10 fino a quando non è nullo, ovvero conto le cifre del numero da covertire
	while((divResult != 0) || (count == 0))
	{
		divResult = divResult / 10;
		count++;
		charSizeCount++;
	}

	divResult = i;

	// Controllo se il nuemro da convertire richiede più spazio di quello voluto (parametro size)
	// Se lo richiede scrivo sempre '*', alrimenti lo converto nella stringa
	if(charSizeCount > size)
  {
		for(j = 0; j < size; j++)
		{
			s[j] = '*';
		}
	}

	else
	{
    
		// Se l'intero richiede un numero di cifre inferiore a quello voluto lo completo con zeri
		while(charSizeCount < size)
		{
			s[j] = '0';
			j++;
			charSizeCount++;
		}

		// Include nella stringa il resto della divisione intera del numero per 10
		// Riduco in numero di un ordine di grandezza e il resto lo includo nell'iterazione successiva
		for(j = 0; j < count; j++)
		{
			s[(charSizeCount - 1) - j] = (divResult % 10) + '0';
			divResult /= 10;
		}
	}
}


static inline uint16_t power(uint16_t base, uint16_t num)
{
  uint16_t result = base;
  
  for(uint16_t i = 1; i < num; i++)
  {
      result = result*base;
  }
  return result;
}
