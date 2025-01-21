#include "stm32f407xx.h"

int main(void)
{

	//LED in PD12 configuration
	GPIO_Handle_t GPIO_Led;

	GPIO_Led.pGPIOx = GPIOD;
	GPIO_Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_Led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_Led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&GPIO_Led);

	//Button in PA0 configuration
	GPIO_Handle_t GPIO_button;

	GPIO_button.pGPIOx = GPIOA;
	GPIO_button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIO_button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIO_button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GPIO_button);

	//Activate LED in PD12 depending on status of PA0

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == SET)
		{
			//Turn ON LED in PD12
			GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_12, SET);
		}else
		{
			//Turn OFF LED in PD12
			GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_12, RESET);
		}
	}

	return 0;
}
