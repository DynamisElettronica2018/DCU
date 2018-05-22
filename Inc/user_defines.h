#ifndef USER_DEFINES_H
#define	USER_DEFINES_H


#define USB_VBUS_ENABLE   		  ((uint8_t)0)
#define USB_VBUS_DISABLE  		  ((uint8_t)1)
#define USB_OK           		 	  ((uint8_t)1)
#define USB_ERROR         		  ((uint8_t)0)

#define TO_WRITE                ((uint8_t)1)
#define TO_READ                 ((uint8_t)1)
#define DIRTY                   ((uint8_t)0)
#define FATFS_OK                ((uint8_t)0)
#define FATFS_ERROR             ((uint8_t)1)
#define SEPARATOR               ((uint8_t)';')
#define END_LINE                ((uint8_t)'\n')
#define DECIMAL_SEPARATOR       ((uint8_t)'.')
#define BUFFER_BLOCK_LEN        ((uint16_t)512)
#define BUFFER_STATE_LEN        ((uint16_t)6)
#define BUFFER_ERROR_LEN        ((uint16_t)3)

#define LSB_12_BIT_MV           (0.8056640625f)
#define V_AMBIENTE_MV           (760f)
#define TEMP_SLOP_MV            (2.5f)
#define ADC_DATA_BUFFER_LENGTH  ((uint8_t)11)

#define LED_TOOGLE_PERIOD       ((uint32_t)250)


#endif	/* USER_DEFINES_H */
