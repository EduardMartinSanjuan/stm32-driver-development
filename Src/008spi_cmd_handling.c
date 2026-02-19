/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 --> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode: 5
 */
#include "stm32f407xx.h"
#include <string.h>

//Command codes
#define COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x54


#define LED_ON					1
#define LED_OFF					0

//Arduino analog pins
#define ANALOG_PIN0				0
#define ANALOG_PIN1				1
#define ANALOG_PIN2				2
#define ANALOG_PIN3				3
#define ANALOG_PIN4				4

//Arduino LED
#define LED_PIN					9

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;

	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32; //Generates sclk of 2MHz
	//SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI; //Hardware slave management enabled for NSS pin

	SPI_Init(&SPI2handle);
}

void GPIO_ButtonInit(void)
{
	//Button in PA0 configuration
	GPIO_Handle_t GPIOBtn,GpioLed;

	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

	//this is led gpio configuration
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&GpioLed);
}

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

uint8_t SPI_VerifyResponse (uint8_t ackbyte)
{
	if(ackbyte == (uint8_t)0xF5)
	{
		//ack
		return 1;
	}

	//nack
	return 0;
}

int main (void)
{
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

	GPIO_ButtonInit();
	//This function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//SPI2 Configuration
	SPI2_Inits();

	/*
	 * making SSOE 1 does NSS output enable.
	 * The NSS pin is automatically managed by the HW.
	 * i.e. when SPE=1, NSS will be pulled to low
	 * and NSS pin will be high when SPE=0
	 */
	SPI_SSOEConfig(SPI2,ENABLE);

	while(1)
	{
		//wait until the button is pressed to start the communication
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		//Debouncing purpose wait
		delay();

		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,ENABLE);

		//1. CMD_LED_CTRL <pin no(1)>	<value(1)>
		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		//send command
		SPI_SendData(SPI2, &commandcode, 1);

		//do dummy read to clear off the RXNE (as per each transmitted byte, the Rx buffer in master it also filled).
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//send some dummy bits (1byte) to fetch the response from the slave.
		SPI_SendData(SPI2, &dummy_write, 1);
		//Read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		//if ack send arguments
		if (SPI_VerifyResponse(ackbyte))
		{
			//send arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI2, args, 2);
			SPI_ReceiveData(SPI2,args,2);
		}
		//end of command LED control



		//2. CMD_SENSOR_READ <analog pin number(1)>

		//wait until the button is pressed to start the communication
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		//Debouncing purpose wait
		delay();

		commandcode = COMMAND_SENSOR_READ;
		//send command
		SPI_SendData(SPI2, &commandcode, 1);

		//do dummy read to clear off the RXNE (as per each transmitted byte, the Rx buffer in master it also filled).
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//send some dummy bits (1byte) to fetch the response from the slave.
		SPI_SendData(SPI2, &dummy_write, 1);
		//Read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		//if ack send arguments
		if (SPI_VerifyResponse(ackbyte))
		{
			//send arguments
			args[0] = ANALOG_PIN0;
			SPI_SendData(SPI2, args, 1);

			//do dummy read to clear off the RXNE (as per each transmitted byte, the Rx buffer in master it also filled).
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			//insert some delay so that slave can ready with the data
			delay();

			//send some dummy bits (1byte) to fetch the response from the slave.
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t analog_read;
			//Read the data from sensor
			SPI_ReceiveData(SPI2, &analog_read, 1);
		}



		/*
		//First send the number of bytes that will be send by SPI
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2,&dataLen,1);

		//Send data
		SPI_SendData(SPI2, (uint8_t*)user_data,strlen(user_data));

		//Confirm SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		//disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,DISABLE);
		*/
	}
	return 0;
}
