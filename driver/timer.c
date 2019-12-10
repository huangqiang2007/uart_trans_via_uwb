#include "main.h"
#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_core.h"
#include "em_gpio.h"
#include "em_system.h"
#include "em_timer.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_dma.h"
#include "timer.h"
#include "uartdrv.h"
#include "mainctrl.h"

// Freq = 25M
#define TOP 25000
#define MS_COUNT  3125  //25000000 / 8 / 1000
#define MAX_MS    20    //65535 / MS_COUNT
#define MAX_TICK (0xFFFFFFF0 - WAKUP_DURATION)

volatile bool Timer1_overflow;
volatile uint32_t g_Ticks = 0;
volatile int32_t g_DMA_nMinus = CMD_LEN, g_DMA_nMinutemp = -1, g_DMA_total_transfers = 0;
volatile uint32_t tx_start_times = 0;
volatile uint32_t tx_finish_times = 0;

/**************************************************************************//**
 * @brief TIMER0_IRQHandler
 * Interrupt Service Routine TIMER0 Interrupt Line
 *****************************************************************************/
void TIMER0_IRQHandler(void)
{
	static int16_t DMA_nMinus_check_times = 0;
	uint32_t DMA_nMinus = 0;
	DMA_DESCRIPTOR_TypeDef *descr = (DMA_DESCRIPTOR_TypeDef *)DMA->CTRLBASE + DMA_CHANNEL;

	/* Clear flag for TIMER0 overflow interrupt */
	TIMER_IntClear(TIMER0, TIMER_IF_OF);
	g_Ticks++;
	if (g_Ticks > 0xFFFFFFF0 - WAKUP_DURATION)
		g_Ticks = 0;

#if 1
	/*
	 * if DMA nMinus field doesn't change at all during specific time scope, we update
	 * rxBuf's rxBuf->wrI pointer timely.
	 * */
	DMA_nMinus = (descr->CTRL & _DMA_CTRL_N_MINUS_1_MASK) >> _DMA_CTRL_N_MINUS_1_SHIFT;
	if (DMA_nMinus_check_times++ > 10000) {
		int8_t temp = 0;

		CORE_CriticalDisableIrq();
		temp = g_DMA_nMinus - g_DMA_nMinutemp;
		g_DMA_total_transfers += temp;
		rxBuf.wrI += temp;
		rxBuf.pendingBytes += temp;
		g_DMA_nMinus = g_DMA_nMinutemp;
		CORE_CriticalEnableIrq();

		DMA_nMinus_check_times = 0;
		//uartPutData("tt\n", 3);
	} else {
		if (DMA_nMinus == 0) {
			DMA_nMinus_check_times = 0;
		}

		if (g_DMA_nMinutemp != DMA_nMinus) {
			g_DMA_nMinutemp = DMA_nMinus;
			DMA_nMinus_check_times = 0;
			//uartPutData("ss\n", 3);
		}
	}
#endif

#if 0
	/* Disable TIMER */
	TIMER_Enable(TIMER0, false);

	TIMER0_status = stop;

	/*
	 * set DMA_CHANNEL interrupt flag
	 * */
	TIMER0_DMA_Req = TIMER_DMA_REQ;
	DMA_IntSet(1<<DMA_CHANNEL);
#endif
}

/**************************************************************************//**
 * @brief TIMER0_IRQHandler
 * Interrupt Service Routine TIMER0 Interrupt Line
 *****************************************************************************/
void TIMER1_IRQHandler(void)
{
	/* Clear flag for TIMER0 overflow interrupt */
	TIMER_IntClear(TIMER1, TIMER_IF_OF);
	Timer1_overflow = true;
}

#define USART0RX_TIMEOUT_NUM 200
uint32_t timer0_top_num = 695;
TIMER_STATUS TIMER0_status = stop;
TIMER_DMA_Req TIMER0_DMA_Req = TIMER_DMA_NOREQ;

void setupTimer0(void)
{
	/* Enable clock for TIMER0 module */
	CMU_ClockEnable(cmuClock_TIMER0, true);

	/* Select TIMER0 parameters */
	TIMER_Init_TypeDef timerInit =
	{
		.enable     = true,
		.debugRun   = true,
		.prescale   = timerPrescale8,
		.clkSel     = timerClkSelHFPerClk,
		.fallAction = timerInputActionNone,
		.riseAction = timerInputActionNone,
		.mode       = timerModeUp,
		.dmaClrAct  = false,
		.quadModeX4 = false,
		.oneShot    = false,
		.sync       = false,
	};

	/* Enable overflow interrupt */
	TIMER_IntEnable(TIMER0, TIMER_IF_OF);

	/* Enable TIMER0 interrupt vector in NVIC */
	NVIC_EnableIRQ(TIMER0_IRQn);

	/* Set TIMER Top value */
	TIMER_TopSet(TIMER0, MS_COUNT);
	//timer0_top_num = (double)CMU_ClockFreqGet(cmuClock_TIMER0) / 8.0 / USART0_BaudRate * 10.0 * USART0RX_TIMEOUT_NUM;
	//TIMER_TopSet(TIMER0, timer0_top_num );
	/* Configure TIMER */
	TIMER_Init(TIMER0, &timerInit);
}

void setupTimer1(void)
{
	/* Enable clock for TIMER0 module */
	CMU_ClockEnable(cmuClock_TIMER1, true);

	/* Select TIMER0 parameters */
	TIMER_Init_TypeDef timerInit =
	{
		.enable     = false,
		.debugRun   = true,
		.prescale   = timerPrescale8,
		.clkSel     = timerClkSelHFPerClk,
		.fallAction = timerInputActionNone,
		.riseAction = timerInputActionNone,
		.mode       = timerModeUp,
		.dmaClrAct  = false,
		.quadModeX4 = false,
		.oneShot    = true,
		.sync       = false,
	};

	/* Enable overflow interrupt */
	TIMER_IntEnable(TIMER1, TIMER_IF_OF);

	/* Enable TIMER0 interrupt vector in NVIC */
	NVIC_EnableIRQ(TIMER1_IRQn);

	/* Set TIMER Top value */
	//  TIMER_TopSet(TIMER1, TOP);

	/* Configure TIMER */
	TIMER_Init(TIMER1, &timerInit);
}

void timer_init(void)
{
//	g_Ticks = 0;

	setupTimer0();
	setupTimer1();
}

/*
 * precondition: timer0's interrupt interval is 1ms
 * */
void delayms(uint32_t ms)
{
	uint32_t ticks = 0;

	ticks = ms + g_Ticks;

	if (ticks > MAX_TICK) {
		ticks = ticks - MAX_TICK;
		while (g_Ticks < MAX_TICK);
		while (g_Ticks == MAX_TICK || g_Ticks < ticks);
	} else {
		while (g_Ticks < ticks);
	}
}

void Delay_us(uint32_t us)
{
  uint32_t countMax;
  countMax = us * 25;
  /* Set TIMER value */
  TIMER_CounterSet(TIMER0, 95);
  while (TIMER_CounterGet(TIMER0) < countMax);
}

/* max 20ms */
void __Delay_ms(uint32_t ms)
{
	Timer1_overflow = false;

	/* Set TIMER value */
	TIMER_CounterSet(TIMER1, 0);
	/* Set TIMER Top value */
	TIMER_TopSet(TIMER1, MS_COUNT * ms);
	/* Enable TIMER */
	TIMER_Enable(TIMER1, true);

	while (Timer1_overflow == false);
	/* Disable TIMER */
	TIMER_Enable(TIMER1, false);
}

void Delay_ms(uint32_t ms)
{
	if (ms < MAX_MS) {
		__Delay_ms(ms);
		return;
	}

	int n = ms / MAX_MS;
	while(n-- > 0) {
		__Delay_ms(MAX_MS);
	}

	if (ms % MAX_MS > 0)
		__Delay_ms(ms % MAX_MS);
}
