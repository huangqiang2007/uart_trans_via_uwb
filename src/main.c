#include <string.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_adc.h"
#include "em_cmu.h"
#include "timer.h"
#include "udelay.h"
#include "hal-config.h"
#include "main.h"
#include "mainctrl.h"
#include "uartdrv.h"
#include "spidrv.h"
#include "Typedefs.h"
#include "libdw1000.h"
#include "em_dma.h"
#include "em_timer.h"

//extern volatile int8_t g_slaveWkup;

void clockConfig(void)
{
	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_TIMER0, true);
	CMU_ClockEnable(cmuClock_TIMER1, true);
	timer_init();
	Delay_ms(5);
	GPIO_PinModeSet(gpioPortA, 1, gpioModePushPull, 1);
	Delay_ms(5);

	SystemCoreClockUpdate();

	/*
	 * chose external crystal oscillator as clock source.
	 * */
	CMU_OscillatorEnable(cmuOsc_HFXO, true, true);
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
	CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);

	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);
	CMU_OscillatorEnable(cmuSelect_ULFRCO, true, true);
	CMU_ClockEnable(cmuClock_HFLE, true);

	/*
	 * Enable clocks required
	 * */
	//CMU_ClockDivSet(cmuClock_HF, cmuClkDiv_2);
	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_USART0, true);
	CMU_ClockEnable(cmuClock_TIMER0, true);
	CMU_ClockEnable(cmuClock_TIMER1, true);
}

#if 0

int main(void)
{
	bool recvDone = false;

	// Initialize chip
	CHIP_Init();

	powerADandUWB(1);
//	spiDMA_test();
	globalInit();
	clockConfig();
	SPIDMAInit();
	//uart_test();
	dwSpiRead(&g_dwDev, 0x00, 0x00, g_dwDev.networkAndAddress, 4);
	g_dwDev.networkAndAddress[0]= 01;
	g_dwDev.networkAndAddress[1]= 02;
	g_dwDev.networkAndAddress[2]= 03;
	g_dwDev.networkAndAddress[3]= 04;

	dwWriteNetworkIdAndDeviceAddress(&g_dwDev);
	memset(g_dwDev.networkAndAddress, 0x00, sizeof(g_dwDev.networkAndAddress));
	dwReadNetworkIdAndDeviceAddress(&g_dwDev);

	dwDeviceInit(&g_dwDev);
  	UDELAY_Calibrate();
  	Delay_ms(500);

	// Place breakpoint here and observe RxBuffer
	// RxBuffer should contain 0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9
	while(1) {
		recvDone = g_spiTransDes.recvDone;
	}
}
#elif 1

void spiDMA_test(dwDevice_t *dev)
{
	uint8_t buf[4] = {0};

	while(1) {
		dwSpiRead(dev, 0x00, 0x00, buf, 4);
		buf[0] = 0x01;
		buf[1] = 0x02;
		buf[2] = 0x03;
		buf[3] = 0x04;
//		dwSpiWrite(dev, 0x03, 0x00, buf, 4);
//		buf[0] = 0x05;
//		buf[1] = 0x06;
//		buf[2] = 0x07;
//		buf[3] = 0x08;
//		dwSpiWrite(dev, 0x03, 0x00, buf, 4);
//		buf[0] = 9;
//		buf[1] = 10;
//		buf[2] = 11;
//		buf[3] = 12;
//		dwSpiWrite(dev, 0x03, 0x00, buf, 4);
		memset(buf, 0x00, 4);
		Delay_ms(1);
		dwSpiRead(dev, 0x03, 0x00, buf, 4);
		dwSpiRead(dev, 0x04, 0x00, buf, 4);
	}
}

extern volatile uint32_t g_Ticks;

void uwb_send_and_recv_test(void)
{
	uint8_t temp[4] = {0};

	g_cmd_feedback_timeout = g_Ticks + CMD_FEEDBACK_TIMEOUT;
	dwSendData(&g_dwDev, "test\n", 5);
	delayms(20);
	dwNewReceive(&g_dwDev);
	dwStartReceive(&g_dwDev);

	while (!g_dataRecvDone && g_cmd_feedback_timeout > g_Ticks);
	g_dataRecvDone = false;
	memcpy(temp, (void *)&g_recvSlaveFr, 4);
}

volatile int g_send_cnt = 0, g_recv_cnt = 0, g_cnt2 = 0;

void uwb_send_and_try_resend(uint8_t *data, int len)
{
	uint8_t temp[4] = {0}, timeout = 0;

//	uint32_t cnt1 = 0, cnt2 = 0;
//
//	TIMER_CounterSet(TIMER1, 0);
//	TIMER_TopSet(TIMER1, 3125 * 10);
//	TIMER_Enable(TIMER1, true);
//
//	cnt1 = TIMER_CounterGet(TIMER1);

	g_send_cnt++;
	dwSendData(&g_dwDev, data, len);
//	delayms(1);
//	dwNewReceive(&g_dwDev);
//	dwStartReceive(&g_dwDev);
//
//	while (!g_dataRecvDone) {
//		delayms(1);
//		if (timeout++ > 2)
//			break;
//	}
//	g_dataRecvDone = false;
//	memcpy(temp, (void *)&g_recvSlaveFr, 4);

//	cnt2 = TIMER_CounterGet(TIMER1);
//	TIMER_Enable(TIMER1, false);
//	g_cnt2 = cnt2 - cnt1;
}

void fill_rxbuf(struct circularBuffer *rxBuf)
{
	int8_t i = 0, j = 0;

	for (i = 0; i < 40; i++)
		for (j = 0; j < 25; j++) {
			if (j == 24)
				rxBuf->data[25 * i + j] = '\n';
			else
				rxBuf->data[25 * i + j] = j;
		}

	rxBuf->pendingBytes = BUFFERSIZE;
}

typedef void (*func)(void);

void hardfault_test(void)
{
	func pc;

	pc = 0xFFFFFFF9;

	pc();
}

int main(void)
{
	/*
	 * Chip errata
	 * */
	CHIP_Init();

	/*
	 * global var init
	 * */
	globalInit();

	/*
	 * config needed clock
	 * */
	clockConfig();

	/*
	 * power down AD
	 * */
	powerADandUWB(0);

	/*
	 * Timer init
	 * */
	timer_init();

	/*
	 * DMA common init
	 * */
	DMAInit();

	/*
	 * RS422 Uart init for delivering converted data
	 * */
	uartSetup();

	/*
	 * SPI master config
	 * */
	//SPIConfig(SPI_CLK);
	SPIDMAInit();

	//spiDMA_test(&g_dwDev);

	/*
	 * DW100 wireless device init
	 * */
	GPIO_PinModeSet(gpioPortC, 13, gpioModePushPull, 1);
	Delay_ms(2);
	GPIO_PinModeSet(gpioPortC, 13, gpioModePushPull, 0);

	GPIO_PinModeSet(gpioPortA, 2, gpioModePushPull, 0);
	Delay_ms(5);
	GPIO_PinModeSet(gpioPortA, 2, gpioModePushPull, 1);

	dwDeviceInit(&g_dwDev);

  	UDELAY_Calibrate();
  	Delay_ms(500);

	//dwNewReceive(&g_dwDev);
	//dwStartReceive(&g_dwDev);

	//fill_rxbuf(&rxBuf);

	while (1) {
		flushRxbuf();
		//rxBuf.pendingBytes = BUFFERSIZE;

		//uwb_send_and_recv_test();


#if 0
		switch(g_cur_mode)
		{
			case MAIN_WKUPMODE:
				WakeupSlave(&g_dwDev);
				break;

			case MAIN_SAMPLEMODE:
				RecvFromSlave(&g_dwDev);
				break;

			case MAIN_IDLEMODE:
				if (!pollSleepCMD(&g_dwDev) < 0)
					g_cur_mode = MAIN_WKUPMODE;
				break;

			case MAIN_SLEEPMODE:
				g_cur_mode = MAIN_SLEEPMODE;
				break;

			default:
				g_cur_mode = MAIN_WKUPMODE;
				break;
		}

		Delay_ms(1);
#endif
	}
}
#endif
