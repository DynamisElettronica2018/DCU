#include "string_utility.h"
#include "data.h"


// Funzione per convertire in stringa i numeri che sono nella forma 3 cifre intere + 2 decimali + segno
// Il formato è fisso e non può essere usata per altri tipi di conversioni
// Il punto decimale è direttamente inserio dalla funzione, quindi non scirverlo prima nel buffer
// Gestisce anche l'eventuale segno del numero
extern inline void decimal_To_String(int16_t i, uint8_t *s, uint8_t num_Int, uint8_t nun_Dec)
{
	int16_t div_Result;
	int16_t integer;
	int16_t decimal;
	uint8_t char_Size_Count = 0;

  div_Result = i;

	// Divido l'intero per 10 fino a quando non è nullo, ovvero conto le cifre del numero da covertire
	while(div_Result != 0)
	{
		div_Result /= 10;
		char_Size_Count++;
	}

	if(char_Size_Count > (num_Int + nun_Dec))
	{
		for(uint8_t j = 0; j < (num_Int + nun_Dec + 1); j++)
		{
			s[j] = '*';
		}
  }

  else
  {
    integer = i / power(10, nun_Dec);

    if(i < 0)
    {
      i *= -1;
    }

    decimal = i % power(10, nun_Dec);
    int_To_String((int16_t)integer, &s[0], num_Int + 1);
    s[num_Int+1] = DECIMAL_SEPARATOR;
    int_To_String((int16_t)decimal, &s[num_Int + 2], nun_Dec);
  }
}


// Funzione per convertire in stringa i numeri che sono nella forma 3 cifre intere + 2 decimali
// Il formato è fisso e non può essere usata per altri tipi di conversioni
// Il punto decimale è direttamente inserio dalla funzione, quindi non scirverlo prima nel buffer
// La funzione gestisce correttamente soltanto numeri positivi, nel formato indicato
extern inline void decimal_To_String_Unsigned(uint16_t i, uint8_t *s, uint8_t num_Int, uint8_t nun_Dec)
{
	uint16_t div_Result;
	uint16_t integer;
	uint16_t decimal;
	uint8_t char_Size_Count = 0;

  div_Result = i;

	// Divido l'intero per 10 fino a quando non è nullo, ovvero conto le cifre del numero da covertire
	while(div_Result != 0)
	{
		div_Result /= 10;
		char_Size_Count++;
	}

	if(char_Size_Count > (num_Int + nun_Dec))
	{
		for(uint8_t j = 0; j < (num_Int + nun_Dec + 1); j++)
		{
			s[j] = '*';
		}
  }

  else
  {
    integer = i / power(10, nun_Dec);
    decimal = i % power(10, nun_Dec);
    int_To_String_Unsigned((int16_t)integer, &s[0], num_Int);
    s[num_Int] = DECIMAL_SEPARATOR;
    int_To_String_Unsigned((int16_t)decimal, &s[num_Int + 1], nun_Dec);
  }
}


// Funzione di conversione da int a stringa di caratteri
// Se il numero richiede un numero di caratteri maggiore di size, scrive '*'
extern inline void int_To_String(int16_t i, uint8_t *s, uint8_t size)
{
	uint8_t count = 0;
	uint8_t char_Size_Count = 0;
	uint8_t j = 0;
	int16_t div_Result;

	// Se il numero è negatico, lo trasformo in uno positivo mettendo il meno davanti
	if(i < 0)
  {
		s[0] = '-';
		i = i * (-1);
		char_Size_Count = 1;
    j = 1;
	}
  
	div_Result = i;

	// Divido l'intero per 10 fino a quando non è nullo, ovvero conto le cifre del numero da covertire
	while((div_Result != 0) || (count == 0))
	{
		div_Result = div_Result / 10;
		count++;
		char_Size_Count++;
	}

	div_Result = i
  ;

	// Controllo se il nuemro da convertire richiede più spazio di quello voluto (parametro size)
	// Se lo richiede scrivo sempre '*', alrimenti lo converto nella stringa
	if(char_Size_Count > size)
  {
		for(j = 0; j < size; j++)
		{
			s[j] = '*';
		}
	}

	else
	{
    
		// Se l'intero richiede un numero di cifre inferiore a quello voluto lo completo con zeri
		while(char_Size_Count < size)
		{
			s[j] = '0';
			j++;
			char_Size_Count++;
		}

		// Include nella stringa il resto della divisione intera del numero per 10
		// Riduco in numero di un ordine di grandezza e il resto lo includo nell'iterazione successiva
		for(j = 0; j < count; j++)
		{
			s[(char_Size_Count - 1) - j] = (div_Result % 10) + '0';
			div_Result /= 10;
		}
	}
}


extern inline void int_To_String_Unsigned(uint16_t i, uint8_t *s, uint8_t size)
{
	uint8_t count = 0;
	uint8_t char_Size_Count = 0;
	uint8_t j = 0;
	uint16_t div_Result;

  div_Result = i;

	// Divido l'intero per 10 fino a quando non è nullo, ovvero conto le cifre del numero da covertire
	while((div_Result != 0) || (count == 0))
	{
		div_Result = div_Result / 10;
		count++;
		char_Size_Count++;
	}

	div_Result = i;

	// Controllo se il nuemro da convertire richiede più spazio di quello voluto (parametro size)
	// Se lo richiede scrivo sempre '*', alrimenti lo converto nella stringa
	if(char_Size_Count > size)
  {
		for(j = 0; j < size; j++)
		{
			s[j] = '*';
		}
	}

	else
	{
    
		// Se l'intero richiede un numero di cifre inferiore a quello voluto lo completo con zeri
		while(char_Size_Count < size)
		{
			s[j] = '0';
			j++;
			char_Size_Count++;
		}

		// Include nella stringa il resto della divisione intera del numero per 10
		// Riduco in numero di un ordine di grandezza e il resto lo includo nell'iterazione successiva
		for(j = 0; j < count; j++)
		{
			s[(char_Size_Count - 1) - j] = (div_Result % 10) + '0';
			div_Result /= 10;
		}
	}
}


extern inline void uint32_To_String(uint32_t i, uint8_t *s, uint8_t size)
{
  uint8_t count = 0;
	uint8_t char_Size_Count = 0;
	uint8_t j = 0;
	uint32_t div_Result;

	div_Result = i;

	// Divido l'intero per 10 fino a quando non è nullo, ovvero conto le cifre del numero da covertire
	while((div_Result != 0) || (count == 0))
	{
		div_Result = div_Result / 10;
		count++;
		char_Size_Count++;
	}

	div_Result = i;

	// Controllo se il nuemro da convertire richiede più spazio di quello voluto (parametro size)
	// Se lo richiede scrivo sempre '*', alrimenti lo converto nella stringa
	if(char_Size_Count > size)
  {
		for(j = 0; j < size; j++)
		{
			s[j] = '*';
		}
	}

	else
	{
    
		// Se l'intero richiede un numero di cifre inferiore a quello voluto lo completo con zeri
		while(char_Size_Count < size)
		{
			s[j] = '0';
			j++;
			char_Size_Count++;
		}

		// Include nella stringa il resto della divisione intera del numero per 10
		// Riduco in numero di un ordine di grandezza e il resto lo includo nell'iterazione successiva
		for(j = 0; j < count; j++)
		{
			s[(char_Size_Count - 1) - j] = (div_Result % 10) + '0';
			div_Result /= 10;
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
