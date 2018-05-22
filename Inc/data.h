#ifndef __DATA_H__
#define __DATA_H__

#include <stdint.h>


extern void initializeData(void);
extern inline void dataConversion(uint16_t ID, uint8_t payload[8]);
extern inline void data10msTimerBackground(void);
extern inline void data100msTimerBackground(void);
static inline void decimalToString(int16_t i, uint8_t *s, uint8_t nInt, uint8_t nDec);
static inline void decimalToStringUnsigned(uint16_t i, uint8_t *s, uint8_t nInt, uint8_t nDec);
static inline void intToString(int16_t i, uint8_t *s, uint8_t size);
static inline void intToStringUnsigned(uint16_t i, uint8_t *s, uint8_t size);
static inline void uint32ToString(uint32_t i, uint8_t *s, uint8_t size);
static inline uint16_t power(uint16_t base, uint16_t num);


#endif
