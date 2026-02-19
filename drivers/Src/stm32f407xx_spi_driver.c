#include "stm32f407xx_spi_driver.h"

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - Enable or disable the SPI Clock register
 *
 * @param[in]         - *pSPIx, ENABLE or DISABLE
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}else
	{
	if(EnorDi == DISABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_DI();
			}else if(pSPIx == SPI2)
			{
				SPI2_PCLK_DI();
			}else if(pSPIx == SPI3)
			{
				SPI3_PCLK_DI();
			}
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             - Configure SPI
 *
 * @param[in]         - SPI Handle structure
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//Configure the SPI_CR1 register
	uint32_t tempreg = 0;

	//enable the SPI peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//1. Configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. Configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//RXONLY bit should be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	//3. Configure the Serial clock speed (baud rate)
	tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	//4. Configure the DFF (data frame format)
	tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	//5. Configure the CPOL (clock polarity)
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	//6. Configure the CPHA (Clock Phase)
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	//7. Configure the SSM (Software Slave management)
	tempreg |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

	//Set the value of the tempreg containing the configuration of the SPI to the CR1 register
	pSPIHandle->pSPIx->CR1 = tempreg;
}

/*********************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             - Reset the given SPIx port
 *
 * @param[in]         - SPI Port
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
}

/*********************************************************************
 * @fn      		  - SPI_GetFlagStatus
 *
 * @brief             - Get the flag status of the SR Register
 *
 * @param[in]         - SPI Port
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}

/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             - Send data SPI
 *
 * @param[in]         - *pSPIx: pointer to SPI
 * @param[in]         - *pTxBuffer: pointer to the TxBuffer
 * @param[in]         - Len: Length of the data to be send
 *
 * @return            -  none
 *
 * @Note              -  This is a blocking call

 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until the TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//2.Check if data frame format is 8 bits or 16 bits (DFF from CR1)
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			//16 bit DFF = 1
			//1.1 Load DR with 2 byte of data
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			//1.2 Decrement Len 2 bytes
			Len --;
			Len --;
			//1.3 Increment the buffer address 2 byte
			(uint16_t*)pTxBuffer++;

		}else
		{
			//8 bit DFF = 0
			//1.1 Load DR with 1 byte of data
			pSPIx->DR = *pTxBuffer;
			//1.2 Decrement Len 1 byte
			Len--;
			//1.3 Increment the buffer address 1 byte
			pTxBuffer++;

		}
	}
}



/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
 *
 * @brief             - Enable or Disable (SPE: SPI enable)
 *
 * @param[in]         - *pSPIx: pointer to SPI
 * @param[in]         - EnOrDi: Enable or Disable
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -

 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE); //Enables the peripheral
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE); //Disables the peripheral
	}
}

/*********************************************************************
 * @fn      		  - SPI_SSIConfig
 *
 * @brief             -
 *
 * @param[in]         - *pSPIx: pointer to SPI
 * @param[in]         - EnOrDi: Enable or Disable
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -

 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI); //Enables the peripheral
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI); //Disables the peripheral
	}
}


/*********************************************************************
 * @fn      		  - SPI_SSOEConfig
 *
 * @brief             -
 *
 * @param[in]         - *pSPIx: pointer to SPI
 * @param[in]         - EnOrDi: Enable or Disable
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -

 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             -
 *
 * @param[in]         - *pSPIx: pointer to SPI
 * @param[in]         - EnOrDi: Enable or Disable
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -

 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)

{
	while(Len > 0)
	{
		//1. wait until the RXNE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		//2.Check if data frame format is 8 bits or 16 bits (DFF from CR1)
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			//16 bit DFF = 1
			//1.1 Load the data from DR to RxBuffer address
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			//1.2 Decrement Len 2 bytes
			Len --;
			Len --;
			//1.3 Increment the buffer address 2 byte
			(uint16_t*)pRxBuffer++;

		}else
		{
			//8 bit DFF = 0
			//1.1 Load DR with 1 byte of data
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			//1.2 Decrement Len 1 byte
			Len--;
			//1.3 Increment the buffer address 1 byte
			pRxBuffer++;

		}
	}
}
/*
 * IRQ Configuration and ISR Handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);



