#include "stm32f407xx_i2c_driver.h"


/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
 *
 * @brief             - Enable or disable the I2C Clock register
 *
 * @param[in]         - *pI2Cx, ENABLE or DISABLE
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}else
	{
	if(EnorDi == DISABLE)
		{
			if(pI2Cx == I2C1)
			{
				I2C1_PCLK_DI();
			}else if(pI2Cx == I2C2)
			{
				I2C2_PCLK_DI();
			}else if(pI2Cx == I2C3)
			{
				I2C3_PCLK_DI();
			}
		}
	}
}


/*********************************************************************
 * @fn      		  - I2C_Init
 *
 * @brief             - Configure I2C
 *
 * @param[in]         - I2C Handle structure
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	//Configure the I2C_CR1 register
	uint32_t tempreg = 0;

	//enable the I2C peripheral clock
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);
}

/*********************************************************************
 * @fn      		  - I2C_DeInit
 *
 * @brief             - Reset the given I2C port
 *
 * @param[in]         - I2C Handle structure
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
    if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}else if(pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}else if(pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}

/*
 * IRQ Configuration and ISR Handling
 */

/*********************************************************************
 * @fn      		  - I2C_IRQInterruptConfig
 *
 * @brief             - Enable or disable the given I2C IRQ number
 *
 * @param[in]         - IRQNumber
 * @param[in]         - EnorDi: ENABLE or DISABLE
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

/*********************************************************************
 * @fn      		  - I2C_IRQPriorityConfig
 *
 * @brief             - Configure the priority for the given I2C IRQ number
 *
 * @param[in]         - IRQNumber
 * @param[in]         - IRQPriority
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/*********************************************************************
 * @fn      		  - I2C_IRQHandling
 *
 * @brief             - Handle I2C interrupt events
 *
 * @param[in]         - I2C handle structure
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  Data interrupt handling requires interrupt-mode
 *                       transmit/receive state to be added to I2C_Handle_t.

 */
void I2C_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	(void)pI2CHandle;
}
