#include <stdio.h>
#include "common/device.h"
#include "common/utility.h"
#include "common/comutil.h"
#include "setup.h"
#include "common/tof/VL53L0X.h"


static void GPIO_setting(void);
static void I2C1_setting(void);
static void SysTick_setting(void);
static void tof_setting(void);

void setup(void)
{
	GPIO_setting();
	I2C1_setting();
	SysTick_setting();
	
	delay_ms(3);
	tof_setting();

#ifdef USE_SLEEP
	CoreDebug->DHCSR |= 0x7;
#endif
}

static void GPIO_setting(void)
{
	RCC_ADCCLKConfig(RCC_PCLK2_Div2);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
			RCC_APB2Periph_GPIOB |
			RCC_APB2Periph_GPIOC |
			RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);


	GPIO_InitTypeDef GPIO_InitStructure;


	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //JTAGでしか使わないピンをI/Oに振る


	//I2C
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

static void I2C1_setting(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	I2C_InitTypeDef I2C_InitStructure;

	/* I2C configuration */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2; 
    //I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = I2C_CLOCK;

	I2C_DeInit(I2C1);
	I2C_Init(I2C1, &I2C_InitStructure);
	I2C_Cmd(I2C1, ENABLE);

	//interrupt setting
	///*
	NVIC_InitTypeDef NVIC_InitStructure;

	//Configure the I2C event priority
	NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//*/
}

static void SysTick_setting(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
	NVIC_SetPriority(SysTick_IRQn, 0);
}

static void tof_setting(void)
{
	VL53L0X.address = 0x52;

	tof_rev_id = VL53L0X_readReg(IDENTIFICATION_REVISION_ID);

	xbee_printf("tof Rev ID is %d",tof_rev_id);

	if(!VL53L0X_init(true)) inform_error();

	VL53L0X_setTimeout(500);

	//VL53L0X_startContinuous(0);
}
