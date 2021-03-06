#ifndef __USER_USB_H__
#define __USER_USB_H__

#include <stdint.h>


extern inline void USB_Open_File(void);
extern inline void USB_Close_File(void);
extern inline void USB_Close_And_Open_File(void);
extern inline void USB_Write_Block(uint8_t *buffer);
extern inline void USB_Write_Len(uint8_t *buffer);
static inline void USB_Get_File_Name(char *filename);


#endif
