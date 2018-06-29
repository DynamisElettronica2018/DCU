#ifndef USER_DEFINES_H
#define	USER_DEFINES_H


#define USB_VBUS_ENABLE   		      ((uint8_t)0)
#define USB_VBUS_DISABLE  		      ((uint8_t)1)
#define TO_WRITE                    ((uint8_t)1)
#define TO_READ                     ((uint8_t)1)
#define DIRTY                       ((uint8_t)0)
#define BUFFER_BLOCK_LEN            ((uint16_t)512)
#define BUFFER_LEN            			((uint8_t)3)
#define BUFFER_STATE_LEN            ((uint16_t)8)
#define BUFFER_DEBUG_LEN            ((uint16_t)94)
#define BUFFER_ERROR_LEN            ((uint16_t)3)
#define START_ACQUISITION_REQUEST   ((uint8_t)1)
#define STOP_ACQUISITION_REQUEST    ((uint8_t)2)
#define START_ACQUISITION_DONE      ((uint8_t)0)


#endif	/* USER_DEFINES_H */
