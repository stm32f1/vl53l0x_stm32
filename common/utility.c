#include <stdio.h>
#include <math.h>
#include "device.h"
#include "utility.h"

static unsigned int delay_ms_count = 0;

void SysTick_Handler(void)
{
	if (delay_ms_count != 0)
		delay_ms_count--;
}

void delay_ms(__IO unsigned int msec)
{
	delay_ms_count = msec;
	SysTick_Config(SystemCoreClock / 1000);

	while (delay_ms_count != 0);

	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

void delay_s(__IO unsigned int sec)
{
	delay_ms(sec * 1000);
}

void pi()
{
	Buzzer = 1;
	delay_ms(60);
	Buzzer = 0;
}

void pipipi(int count)
{
	for (__IO int i = 0; i < count; i++) {
		delay_ms(60);
		Buzzer = 1;
		delay_ms(60);
		Buzzer = 0;
	}
}

void pii(int count)
{
	for (int i = 0; i < count; i++) {
		delay_ms(60);
		Buzzer = 1;
		delay_ms(160);
		Buzzer = 0;
	}
}

void inform_error(void)
{
	while (1) {
		pii(1);
		pipipi(1);
	}
}
