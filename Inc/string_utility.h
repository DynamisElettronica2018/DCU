#ifndef __STRING_UTILITY_H__
#define __STRING_UTILITY_H__

#include <stdint.h>


extern inline void decimal_To_String(int16_t i, uint8_t *s, uint8_t num_Int, uint8_t num_Dec);
extern inline void decimal_To_String_Unsigned(uint16_t i, uint8_t *s, uint8_t num_Int, uint8_t num_Dec);
extern inline void int_To_String(int16_t i, uint8_t *s, uint8_t size);
extern inline void int_To_String_Unsigned(uint16_t i, uint8_t *s, uint8_t size);
extern inline void uint32_To_String(uint32_t i, uint8_t *s, uint8_t size);
static inline uint16_t power(uint16_t base, uint16_t num);


#endif
