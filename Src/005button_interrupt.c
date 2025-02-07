#include "stm32f407xx.h"
#include <string.h>

#define HIGH 1
#define LOW 0
#define BTN_PRESSED LOW

void delay (void)
{
	//This will introduce ~200ms delay when sysclk is 16MHz
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void)
{

	GPIO_Handle_t GpioLed, GPIO_button;

	memset(&GpioLed,0,sizeof(GpioLed)); //Initialize each member element to 0 (to avoid random values if any parameter is not set in the application code)
	memset(&GPIO_button,0,sizeof(GPIO_button));

	//this is led gpio configuration

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&GpioLed);

	//Button configuration
	GPIO_button.pGPIOx = GPIOD;
	GPIO_button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIO_button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&GPIO_button);

	//IRQ Configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	while(1);
}

void EXTI9_5_IRQHandler (void)
{
	delay(); //200ms delay
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}
