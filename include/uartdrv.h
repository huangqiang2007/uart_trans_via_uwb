#ifndef UARTDRV_H_
#define UARTDRV_H_

#include <stdint.h>

#define UARTFRAME_LEN_12B 12
/*
 * convert AD samples into RS422 frame
 * */
typedef struct {
	/*
	 * the frame of 12 bytes
	 * */
	uint8_t uartFrame[UARTFRAME_LEN_12B];
} UartFrame;

/* Function prototypes */
extern void uartSetup(void);
extern void uartPutData(volatile uint8_t * dataPtr, uint32_t dataLen);
extern uint32_t uartGetData(uint8_t * dataPtr, uint32_t dataLen);
extern void    uartPutChar(uint8_t charPtr);
extern uint8_t uartGetChar(void);
extern uint32_t uartReadChar(uint8_t *data);
extern int UartFrameEnqueue(UartFrame *uFrame);
extern UartFrame* UartFrameDequeue(void);

extern uint32_t USART0_BaudRate;

void DMAInit(void);
//void DMAConfig(void);

#endif /* UARTDRV_H_ */
