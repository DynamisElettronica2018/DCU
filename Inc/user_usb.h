#ifndef __USER_USB_H__
#define __USER_USB_H__

#include <stdint.h>


extern inline void USB_User_Start(const char *filename);
extern inline void USB_User_Stop(void);
extern inline void USB_Write(uint8_t *buffer);
extern inline void USB_Write_Len(uint8_t *buffer);
extern inline void USB_Get_File_Name(char *filename);


#endif
