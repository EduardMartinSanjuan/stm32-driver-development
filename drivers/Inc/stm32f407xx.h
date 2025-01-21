

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo volatile

/*
 * Base Addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR			0x08000000U			/*Base Address of FLASH memory*/
#define SRAM1_BASEADDR			0x20000000U			/*Base Address of SRAM1 memory*/
#define SRAM2_BASEADDR			0x20001C00U			/*Base Address of SRAM2 memory*/
#define ROM						0x1FFF0000U			/*Base Address of ROM memory*/
#define SRAM 					SRAM1_BASEADDR		/*Base Address of SRAM memory*/

/*
 * AHBx and APBx Bus Peripherals Base Addresses
 */

#define PERIPH_BASEADDR				0x40000000U			/*Peripherals Base Address*/
#define APB1PERIPH_BASEADDR			PERIPH_BASEADDR		/*Base Address of APB1 Bus*/
#define APB2PERIPH_BASEADDR			0x40010000U			/*Base Address of APB2 Bus*/
#define AHB1PERIPH_BASEADDR			0x40020000U			/*Base Address of AHB1 Bus*/
#define AHB2PERIPH_BASEADDR			0x50000000U			/*Base Address of AHB2 Bus*/

/*
 * Base Addresses of peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x00)			/*Base Address of GPIOA Peripheral*/
#define GPIOB_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x0400)			/*Base Address of GPIOB Peripheral*/
#define GPIOC_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x0800)			/*Base Address of GPIOC Peripheral*/
#define GPIOD_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x0C00)			/*Base Address of GPIOD Peripheral*/
#define GPIOE_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x1000)			/*Base Address of GPIOE Peripheral*/
#define GPIOF_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x1400)			/*Base Address of GPIOF Peripheral*/
#define GPIOG_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x1800)			/*Base Address of GPIOG Peripheral*/
#define GPIOH_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x1C00)			/*Base Address of GPIOH Peripheral*/
#define GPIOI_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x2000)			/*Base Address of GPIOI Peripheral*/
#define GPIOJ_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x2400)			/*Base Address of GPIOJ Peripheral*/
#define GPIOK_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x0800)			/*Base Address of GPIOK Peripheral*/

#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800)			/*Base Address of RCC Register*/

/*
 * Base Addresses of peripherals which are hanging on APB1 bus
 */

#define I2C1_BASEADDR 			(APB1PERIPH_BASEADDR + 0x5400)			/*Base Address of I2C1 Peripheral*/
#define I2C2_BASEADDR 			(APB1PERIPH_BASEADDR + 0x5800)			/*Base Address of I2C2 Peripheral*/
#define I2C3_BASEADDR 			(APB1PERIPH_BASEADDR + 0x5C00)			/*Base Address of I2C3 Peripheral*/

#define SPI2_BASEADDR 			(APB1PERIPH_BASEADDR + 0x3800)			/*Base Address of SPI2 Peripheral*/
#define SPI3_BASEADDR 			(APB1PERIPH_BASEADDR + 0x3C00)			/*Base Address of SPI3 Peripheral*/

#define USART2_BASEADDR 		(APB1PERIPH_BASEADDR + 0x4400)			/*Base Address of USART2 Peripheral*/
#define USART3_BASEADDR 		(APB1PERIPH_BASEADDR + 0x4800)			/*Base Address of USART3 Peripheral*/
#define UART4_BASEADDR 			(APB1PERIPH_BASEADDR + 0x4C00)			/*Base Address of UART4 Peripheral*/
#define UART5_BASEADDR 			(APB1PERIPH_BASEADDR + 0x5000)			/*Base Address of UART5 Peripheral*/

/*
 * Base Addresses of peripherals which are hanging on APB2 bus
 */

#define SPI1_BASEADDR 			(APB2PERIPH_BASEADDR + 0x3000)			/*Base Address of SPI1 Peripheral*/

#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x1000)			/*Base Address of USART1 Peripheral*/
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR + 0x1400)			/*Base Address of USART2 Peripheral*/

#define EXTI_BASEADDR 			(APB2PERIPH_BASEADDR + 0x3C00)			/*Base Address of EXTI Peripheral*/

#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800)			/*Base Address of SYSCFG Peripheral*/



/************************************************* peripheral register definition structures *************************************************/

/*
 * GPIO Peripheral
 * Use: 	#define GPIOA 		((GPIO_RegDef_t*)GPIOA_BASEADDR)
 */

typedef struct
{
	__vo uint32_t MODER;  			/*GPIO port mode register. Offset: 0x00*/
	__vo uint32_t OTYPER;  			/*GPIO port output type register. Offset: 0x04*/
	__vo uint32_t OSPEEDR;  		/*GPIO port output speed register. Offset: 0x08*/
	__vo uint32_t PUPDR;  			/*GPIO port pull-up/pull-down register. Offset: 0x0C*/
	__vo uint32_t IDR; 				/*GPIO port input data register. Offset: 0x10*/
	__vo uint32_t ODR;  			/*GPIO port output data register. Offset: 0x14*/
	__vo uint32_t BSRR;  			/*GPIO port bit set/reset register. Offset: 0x18*/
	__vo uint32_t LCKR;				/*GPIO port configuration lock register. Offset: 0x1C*/
	__vo uint32_t AFR[2];			/*AFR[0]: GPIO alternate function low register. Offset: 0x20, AFR[1]: GPIO alternate function high register. Offset: 0x24*/
} GPIO_RegDef_t;


/*
* RCC Register definition
*/

typedef struct
{
	__vo uint32_t CR;				/* RCC clock control register. Offset: 0x00 */
	__vo uint32_t PLLCFGR;			/* RCC PLL configuration register. Offset: 0x04 */
	__vo uint32_t CFGR;				/* RCC clock configuration register. Offset: 0x08 */
	__vo uint32_t CIR;				/* RCC clock interrupt register. Offset: 0x0C */
	__vo uint32_t AHB1RSTR;			/* RCC AHB1 peripheral reset register. Offset: 0x10 */
	__vo uint32_t AHB2RSTR;			/* RCC AHB2 peripheral reset register. Offset: 0x14 */
	__vo uint32_t AHB3RSTR;			/* RCC AHB3 peripheral reset register. Offset: 0x18 */
	uint32_t Reserved1;				/* Reserved. Offset: 0x1C */
	__vo uint32_t APB1RSTR;			/* RCC APB1 peripheral reset register. Offset: 0x20 */
	__vo uint32_t APB2RSTR;			/* RCC APB2 peripheral reset register. Offset: 0x24 */
	uint32_t Reserved2[2];			/* Reserved. Offset: 0x28-0x2C */
	__vo uint32_t AHB1ENR;			/* RCC AHB1 peripheral clock enable register. Offset: 0x30 */
	__vo uint32_t AHB2ENR;			/* RCC AHB2 peripheral clock enable register. Offset: 0x34 */
	__vo uint32_t AHB3ENR;			/* RCC AHB3 peripheral clock enable register. Offset: 0x38 */
	uint32_t Reserved3;				/* Reserved. Offset: 0x3C */
	__vo uint32_t APB1ENR;			/* RCC APB1 peripheral clock enable register. Offset: 0x40 */
	__vo uint32_t APB2ENR;			/* RCC APB2 peripheral clock enable register. Offset: 0x44 */
	uint32_t Reserved4[2];			/* Reserved. Offset: 0x48-0x4C */
	__vo uint32_t AHB1LPENR;		/* RCC AHB1 peripheral clock enable in low power mode register. Offset: 0x50 */
	__vo uint32_t AHB2LPENR;		/* RCC AHB2 peripheral clock enable in low power mode register. Offset: 0x54 */
	__vo uint32_t AHB3LPENR;		/* RCC AHB3 peripheral clock enable in low power mode register. Offset: 0x58 */
	uint32_t Reserved5;				/* Reserved. Offset: 0x5C */
	__vo uint32_t APB1LPENR;		/* RCC APB1 peripheral clock enable in low power mode register. Offset: 0x60 */
	__vo uint32_t APB2LPENR;		/* RCC APB2 peripheral clock enable in low power mode register. Offset: 0x64 */
	uint32_t Reserved6[2];			/* Reserved. Offset: 0x68-0x6C */
	__vo uint32_t BDCR;				/* RCC Backup domain control register. Offset: 0x70 */
	__vo uint32_t CSR;				/* RCC clock control & status register. Offset: 0x74 */
	uint32_t Reserved7[2];			/* Reserved. Offset: 0x78-0x7C */
	__vo uint32_t SSCGR;			/* RCC spread spectrum clock generation register. Offset: 0x80 */
	__vo uint32_t PLLI2SCFGR;		/* RCC PLLI2S configuration register. Offset: 0x84 */
} RCC_RegDef_t;

/*
* Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
*/

#define GPIOA 		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 		((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 		((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 		((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI 		((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define GPIOJ 		((GPIO_RegDef_t*)GPIOJ_BASEADDR)
#define GPIOK 		((GPIO_RegDef_t*)GPIOK_BASEADDR)

#define RCC 		((RCC_RegDef_t*)RCC_BASEADDR)

/*
* Clock Enable Macros for GPIOx peripherals
*/

#define GPIOA_PCLK_EN()	(RCC->AHB1ENR |= (1 << 0))		/* Clock Enable for GPIOA */
#define GPIOB_PCLK_EN()	(RCC->AHB1ENR |= (1 << 1))		/* Clock Enable for GPIOB */
#define GPIOC_PCLK_EN()	(RCC->AHB1ENR |= (1 << 2))		/* Clock Enable for GPIOC */
#define GPIOD_PCLK_EN()	(RCC->AHB1ENR |= (1 << 3))		/* Clock Enable for GPIOD */
#define GPIOE_PCLK_EN()	(RCC->AHB1ENR |= (1 << 4))		/* Clock Enable for GPIOE */
#define GPIOF_PCLK_EN()	(RCC->AHB1ENR |= (1 << 5))		/* Clock Enable for GPIOF */
#define GPIOG_PCLK_EN()	(RCC->AHB1ENR |= (1 << 6))		/* Clock Enable for GPIOG */
#define GPIOH_PCLK_EN()	(RCC->AHB1ENR |= (1 << 7))		/* Clock Enable for GPIOH */
#define GPIOI_PCLK_EN()	(RCC->AHB1ENR |= (1 << 8))		/* Clock Enable for GPIOI */

/*
* Clock Enable Macros for I2Cx peripherals
*/

#define I2C1_PCLK_EN()	(RCC->APB1ENR |= (1 << 21))		/* Clock Enable for I2C1 */
#define I2C2_PCLK_EN()	(RCC->APB1ENR |= (1 << 22))		/* Clock Enable for I2C2 */
#define I2C3_PCLK_EN()	(RCC->APB1ENR |= (1 << 23))		/* Clock Enable for I2C3 */

/*
* Clock Enable Macros for SPIx peripherals
*/

#define SPI1_PCLK_EN()	(RCC->APB2ENR |= (1 << 12))		/* Clock Enable for SPI1 */
#define SPI2_PCLK_EN()	(RCC->APB1ENR |= (1 << 14))		/* Clock Enable for SPI2 */
#define SPI3_PCLK_EN()	(RCC->APB1ENR |= (1 << 15))		/* Clock Enable for SPI3 */

/*
* Clock Enable Macros for USARTx peripherals
*/

#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 4))	/* Clock Enable for USART1 */
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1 << 5))	/* Clock Enable for USART6 */

#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))	/* Clock Enable for USART2 */
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 18))	/* Clock Enable for USART3 */
#define UART4_PCLK_EN()		(RCC->APB1ENR |= (1 << 19))	/* Clock Enable for UART4 */
#define UART5_PCLK_EN()		(RCC->APB1ENR |= (1 << 20))	/* Clock Enable for UART5 */

/*
* Clock Enable Macros for SYSCFG peripherals
*/

#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))	/* Clock Enable for UART5 */

/*
* Clock Disable Macros for GPIOx peripherals
*/

#define GPIOA_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 0))		/* Clock Disable for GPIOA */
#define GPIOB_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 1))		/* Clock Disable for GPIOB */
#define GPIOC_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 2))		/* Clock Disable for GPIOC */
#define GPIOD_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 3))		/* Clock Disable for GPIOD */
#define GPIOE_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 4))		/* Clock Disable for GPIOE */
#define GPIOF_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 5))		/* Clock Disable for GPIOF */
#define GPIOG_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 6))		/* Clock Disable for GPIOG */
#define GPIOH_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 7))		/* Clock Disable for GPIOH */
#define GPIOI_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 8))		/* Clock Disable for GPIOI */

/*
* Clock Disable Macros for I2Cx peripherals
*/

#define I2C1_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 21))		/* Clock Disable for I2C1 */
#define I2C2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 22))		/* Clock Disable for I2C2 */
#define I2C3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 23))		/* Clock Disable for I2C3 */

/*
* Clock Disable Macros for SPIx peripherals
*/

#define SPI1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 12))		/* Clock Disable for SPI1 */
#define SPI2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 14))		/* Clock Disable for SPI2 */
#define SPI3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 15))		/* Clock Disable for SPI3 */

/*
* Clock Disable Macros for USARTx peripherals
*/

#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 4))		/* Clock Disable for USART1 */
#define USART6_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 5))		/* Clock Disable for USART6 */

#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 17))	/* Clock Disable for USART2 */
#define USART3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 18))	/* Clock Disable for USART3 */
#define UART4_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 19))	/* Clock Disable for UART4 */
#define UART5_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 20))	/* Clock Disable for UART5 */

/*
* Clock Disable Macros for SYSCFG peripherals
*/

#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 14))	/* Clock Disable for UART5 */


/*
 * Macros to reset GPIOx peripherals
 * First set to 0 (reset) then level as not reset status.
 */
#define GPIOA_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 0));	(RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define GPIOB_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 1));	(RCC->AHB1RSTR &= ~(1 << 1));} while(0)
#define GPIOC_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 2));	(RCC->AHB1RSTR &= ~(1 << 2));} while(0)
#define GPIOD_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 3));	(RCC->AHB1RSTR &= ~(1 << 3));} while(0)
#define GPIOE_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 4));	(RCC->AHB1RSTR &= ~(1 << 4));} while(0)
#define GPIOF_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 5));	(RCC->AHB1RSTR &= ~(1 << 5));} while(0)
#define GPIOG_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 6));	(RCC->AHB1RSTR &= ~(1 << 6));} while(0)
#define GPIOH_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 7));	(RCC->AHB1RSTR &= ~(1 << 7));} while(0)
#define GPIOI_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 8));	(RCC->AHB1RSTR &= ~(1 << 8));} while(0)


/*
 * Generic Macros
 */

#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET


/*
 * GPIO Driver specific
 */
#include "stm32f407xx_gpio_driver.h"

#endif /* INC_STM32F407XX_H_ */

