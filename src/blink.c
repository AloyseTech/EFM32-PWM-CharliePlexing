/***************************************************************************//**
 * @file
 * @brief Simple LED Blink Demo for EFM32TG_STK3300
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_timer.h"
#include "bsp.h"
#include "bsp_trace.h"



// Note: change this to set the desired duty cycle (used to update CCVB value)
static volatile int dutyCyclePercent = 30;

#define USE_PWM (1)
#define PIN_COUNT (3)
#define LED_FPS	(100)
#define LED_COUNT (PIN_COUNT*(PIN_COUNT-1))
static volatile uint8_t ledStates[LED_COUNT] = { 255, 25, 63, 0, 0, 0};
static volatile uint8_t ledIndex = 0;

#define UPDATE_FREQ (LED_COUNT * LED_FPS)
#define PWM_FREQ (UPDATE_FREQ*100)

//return the anode pin used to control a given node
uint8_t anode(uint8_t node)
{
	return node/(PIN_COUNT - 1);
}

//return the cathode pin used to control a given node
uint8_t cathode(uint8_t node)
{
	uint8_t c = (node%(PIN_COUNT - 1));
	if(c >= anode(node)) c++;
	return c;
}

void tristateAllPins()
{
	GPIO_PinModeSet(gpioPortD, 1, gpioModeInput, 0);
	GPIO_PinModeSet(gpioPortD, 2, gpioModeInput, 0);
	GPIO_PinModeSet(gpioPortD, 3, gpioModeInput, 0);
}

// Enable PWM on a given pin with a 8bit duty-cycle ratio
void writePin(uint8_t pin, uint8_t value)
{
	//	TIMER0->ROUTE = (TIMER_ROUTE_CC0PEN | TIMER_ROUTE_CC1PEN | TIMER_ROUTE_CC2PEN | TIMER_ROUTE_LOCATION_LOC3);
	switch(pin)
	{
	case 0:
		GPIO_PinModeSet(gpioPortD, 1, gpioModePushPull, 0);
		TIMER_CompareSet(TIMER0, 0, (TIMER_TopGet(TIMER0) * value) / 255);
		TIMER0->ROUTE |= (TIMER_ROUTE_CC0PEN | TIMER_ROUTE_LOCATION_LOC3);
		break;
	case 1:
		GPIO_PinModeSet(gpioPortD, 2, gpioModePushPull, 0);
		TIMER_CompareSet(TIMER0, 1, (TIMER_TopGet(TIMER0) * value) / 255);
		TIMER0->ROUTE |= (TIMER_ROUTE_CC1PEN | TIMER_ROUTE_LOCATION_LOC3);
		break;
	case 2:
		GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 0);
		TIMER_CompareSet(TIMER0, 2, (TIMER_TopGet(TIMER0) * value) / 255);
		TIMER0->ROUTE |= (TIMER_ROUTE_CC2PEN | TIMER_ROUTE_LOCATION_LOC3);
		break;
	default:
		break;
	}
}

void setPin(uint8_t pin)
{
	switch(pin)
	{
	case 0:
		GPIO_PinModeSet(gpioPortD, 1, gpioModePushPull, 1);
		break;
	case 1:
		GPIO_PinModeSet(gpioPortD, 2, gpioModePushPull, 1);
		break;
	case 2:
		GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 1);
		break;
	default:
		break;
	}
}

void clearPin(uint8_t pin)
{
	switch(pin)
	{
	case 0:
		GPIO_PinModeSet(gpioPortD, 1, gpioModePushPull, 0);
		break;
	case 1:
		GPIO_PinModeSet(gpioPortD, 2, gpioModePushPull, 0);
		break;
	case 2:
		GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 0);
		break;
	default:
		break;
	}
}

void TIMER1_IRQHandler(void)
{
	// Acknowledge the interrupt
	uint32_t flags = TIMER_IntGet(TIMER1);
	TIMER_IntClear(TIMER1, flags);

	//Disable PWM timer on all pins
#if USE_PWM
	TIMER1->ROUTE = 0;
#endif

//#if !USE_PWM
	tristateAllPins();
//#endif
	if(ledStates[ledIndex] > 0)
	{
#if USE_PWM
		writePin(anode(ledIndex), ledStates[ledIndex]);
		writePin(cathode(ledIndex), 0);
#else
		setPin(anode(ledIndex));
		clearPin(cathode(ledIndex));
#endif

	}

	ledIndex++;
	if(ledIndex == LED_COUNT)
		ledIndex = 0;

}


void initPwmTimer(void)
{
	GPIO_PinModeSet(gpioPortD, 1, gpioModePushPull, 0);
	GPIO_PinModeSet(gpioPortD, 2, gpioModePushPull, 0);
	GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 0);

	// Enable clock for TIMER1 module
	CMU_ClockEnable(cmuClock_TIMER0, true);

	// Configure TIMER1 Compare/Capture for output compare
	// Use PWM mode, which sets output on overflow and clears on compare events
	TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;
	timerCCInit.mode = timerCCModePWM;
	TIMER_InitCC(TIMER0, 0, &timerCCInit);
	TIMER_InitCC(TIMER0, 1, &timerCCInit);
	TIMER_InitCC(TIMER0, 2, &timerCCInit);

	// TIM1_CC0 #3 is GPIO Pin PB7
	// TIM1_CC1 #3 is GPIO Pin PB8
	// TIM1_CC2 #3 is GPIO Pin PB11
	TIMER0->ROUTE = (TIMER_ROUTE_CC0PEN | TIMER_ROUTE_CC1PEN /*| TIMER_ROUTE_CC2PEN*/ | TIMER_ROUTE_LOCATION_LOC3);

	// Set top value to overflow at the desired PWM_FREQ frequency
	uint32_t top = 0;
	top = CMU_ClockFreqGet(cmuClock_TIMER0) / PWM_FREQ;
	TIMER_TopSet(TIMER0, top);

	// Set compare value for initial duty cycle
	//TIMER_CompareSet(TIMER1, 0, (TIMER_TopGet(TIMER0) * dutyCyclePercent) / 100);
	TIMER_CompareSet(TIMER0, 0, 0);
	TIMER_CompareSet(TIMER0, 1, 0);
	TIMER_CompareSet(TIMER0, 2, 0);

	// Initialize the timer
	TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
	TIMER_Init(TIMER0, &timerInit);
}

void initUpdateTimer(void)
{
	// Enable clock for TIMER0 module
	CMU_ClockEnable(cmuClock_TIMER1, true);

	// Set top value to overflow at the desired PWM_FREQ frequency
	uint32_t top;
	top = CMU_ClockFreqGet(cmuClock_TIMER1) / UPDATE_FREQ;
	while(top > TIMER_MaxCount(TIMER1))
	{
		if(cmuClkDiv_512 == CMU_ClockDivGet(cmuClock_HFPER))
		{
			if(CMU_HFRCOBandGet() == 0)
				break;
			CMU_HFRCOBandSet(CMU_HFRCOBandGet() - 1);
			CMU_ClockDivSet(cmuClock_HFPER, cmuClkDiv_1);
		}

		CMU_ClockDivSet(cmuClock_HFPER, CMU_ClockDivGet(cmuClock_HFPER) * 2);
		top = CMU_ClockFreqGet(cmuClock_TIMER1) / UPDATE_FREQ;
	}
	TIMER_TopSet(TIMER1, top);

	// Initialize the timer
	TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
	TIMER_Init(TIMER1, &timerInit);

	// Enable TIMER1 compare event interrupts to update the duty cycle
	TIMER_IntEnable(TIMER1, TIMER_IEN_OF);
	NVIC_EnableIRQ(TIMER1_IRQn);
}

volatile uint32_t msTicks; /* counts 1ms timeTicks */

void Delay(uint32_t dlyTicks);

/***************************************************************************//**
 * @brief SysTick_Handler
 * Interrupt Service Routine for system tick counter
 ******************************************************************************/
void SysTick_Handler(void)
{
	msTicks++;       /* increment counter necessary in Delay()*/
}

/***************************************************************************//**
 * @brief Delays number of msTick Systicks (typically 1 ms)
 * @param dlyTicks Number of ticks to delay
 ******************************************************************************/
void Delay(uint32_t dlyTicks)
{
	uint32_t curTicks;

	curTicks = msTicks;
	while ((msTicks - curTicks) < dlyTicks) ;
}

/***************************************************************************//**
 * @brief  Main function
 ******************************************************************************/
int main(void)
{
	/* Chip errata */
	CHIP_Init();

	CMU_HFRCOBandSet(cmuHFRCOBand_28MHz);
	CMU_ClockDivSet(cmuClock_HFPER, cmuClkDiv_1);

	/* If first word of user data page is non-zero, enable Energy Profiler trace */
	BSP_TraceProfilerSetup();

#if USE_PWM
	initPwmTimer();
#endif
	initUpdateTimer();

	/* Setup SysTick Timer for 1 msec interrupts  */
	if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000)) {
		while (1) ;
	}

	/* Initialize LED driver */
	BSP_LedsInit();

	/* Infinite blink loop */
	uint32_t t = 0;
	double f = 2.0;
	while (1) {
		BSP_LedToggle(0);
#if USE_PWM
		Delay(1);
		t++;
		for(int i = 0; i < LED_COUNT; i++)
		{
			ledStates[i] = 128 + (sin(2*3.14*f*t/1000.0 + 2*3.14*i/LED_COUNT))*127.0;
		}
#else
		Delay(100);
		t++;
		for(int i = 0; i < LED_COUNT; i++)
		{
			ledStates[i] = (((t+i)%LED_COUNT) < (LED_COUNT/2))?1:0;
		}
#endif
	}
}
