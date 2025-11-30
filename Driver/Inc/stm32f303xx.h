/*
 * stm32f303xx.h
 *
 *  Created on: Nov 4, 2025
 *      Author: aswin
 */

#ifndef INC_STM32F303XX_H_
#define INC_STM32F303XX_H_


#define __vo volatile
#include <stdint.h>


//Some Basic Macros
#define ENABLE						 1
#define DISABLE						 0
#define SET							 ENABLE
#define RESET						 DISABLE




//Base Addresses of various memories
#define FLASH_BASEADDR				0x08000000u
#define SRAM1_BASEADDR				0x20000000u
#define SRAM2_BASEADDR				0x10000000u
#define ROM_BASEADDR				0x1FFFD800u
#define SRAM						SRAM1_BASEADDR


//Base Addresses of various Buses
#define AHB1_BASEADDR				0x40020000u
#define AHB2_BASEADDR				0x48000000u
#define AHB3_BASEADDR				0x50000000u
#define AHB4_BASEADDR				0x60000000u
#define APB1_BASEADDR				0x40000000u
#define APB2_BASEADDR				0x40010000u


//Base Addresses of various peripherals

//GPIO Peripherals
#define GPIOA_BASEADDR				(AHB2_BASEADDR + 0x0000)
#define GPIOB_BASEADDR				(AHB2_BASEADDR + 0x0400)
#define GPIOC_BASEADDR				(AHB2_BASEADDR + 0x0800)
#define GPIOD_BASEADDR				(AHB2_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR				(AHB2_BASEADDR + 0x1000)
#define GPIOF_BASEADDR				(AHB2_BASEADDR + 0x1400)
#define GPIOG_BASEADDR				(AHB2_BASEADDR + 0x1800)
#define GPIOH_BASEADDR				(AHB2_BASEADDR + 0x1C00)

//RCC Peripherals
#define RCC_BASEADDR				(AHB1_BASEADDR + 0x1000)

//SPIx Peripherals
#define SPI4_BASEADDR				(APB2_BASEADDR + 0x3C00)
#define SPI3_BASEADDR				(APB1_BASEADDR + 0x3C00)
#define SPI2_BASEADDR				(APB1_BASEADDR + 0x3800)
#define SPI1_BASEADDR				(APB2_BASEADDR + 0x3000)

//I2Cx Peripherals
#define I2C3_BASEADDR				(AHB1_BASEADDR + 0x7800)
#define I2C2_BASEADDR				(AHB1_BASEADDR + 0x5800)
#define I2C1_BASEADDR				(AHB1_BASEADDR + 0x5400)

//UART USART Peripherals
#define UART5_BASEADDR				(AHB1_BASEADDR + 0x5000)
#define UART4_BASEADDR				(AHB1_BASEADDR + 0x4C00)
#define USART3_BASEADDR				(AHB1_BASEADDR + 0x4800)
#define USART2_BASEADDR				(AHB1_BASEADDR + 0x4400)
#define USART1_BASEADDR				(APB2_BASEADDR + 0x3800)

#define EXTI_BASEADDR				(APB2_BASEADDR + 0x0400)
#define SYSCFG_BASEADDR				(APB2_BASEADDR + 0x0000)


//Structure for GPIOx registers
typedef struct{

	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];
	__vo uint32_t BSR;

} GPIO_Regs_t;


//Structure for RCC Registers
typedef struct{

	__vo uint32_t CR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t APB1RSTR;
	__vo uint32_t AHBENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t APB1ENR;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t AHBRSTR;
	__vo uint32_t CFGR2;
	__vo uint32_t CFGR3;

}RCC_Regs_t;


//GPIO Peripheral definitions
#define GPIOA						((GPIO_Regs_t*)GPIOA_BASEADDR)
#define GPIOB						((GPIO_Regs_t*)GPIOB_BASEADDR)
#define GPIOC						((GPIO_Regs_t*)GPIOC_BASEADDR)
#define GPIOD						((GPIO_Regs_t*)GPIOD_BASEADDR)
#define GPIOE						((GPIO_Regs_t*)GPIOE_BASEADDR)
#define GPIOF						((GPIO_Regs_t*)GPIOF_BASEADDR)
#define GPIOG						((GPIO_Regs_t*)GPIOG_BASEADDR)
#define GPIOH						((GPIO_Regs_t*)GPIOH_BASEADDR)


//RCC Peripheral definitions
#define RCC							((RCC_Regs_t*)RCC_BASEADDR)


//Clock Enable Macros for GPIOx
#define GPIOA_CLK_EN()				( RCC -> AHBENR |= (1 << 17))
#define GPIOB_CLK_EN()				( RCC -> AHBENR |= (1 << 18))
#define GPIOC_CLK_EN()				( RCC -> AHBENR |= (1 << 19))
#define GPIOD_CLK_EN()				( RCC -> AHBENR |= (1 << 20))
#define GPIOE_CLK_EN()				( RCC -> AHBENR |= (1 << 21))
#define GPIOF_CLK_EN()				( RCC -> AHBENR |= (1 << 22))
#define GPIOG_CLK_EN()				( RCC -> AHBENR |= (1 << 23))
#define GPIOH_CLK_EN()				( RCC -> AHBENR |= (1 << 16))


//Clock Enable Macros for SPI
#define SPI1_CLK_EN()				( RCC -> APB2ENR |= (1 << 12))
#define SPI2_CLK_EN()				( RCC -> APB1ENR |= (1 << 14))
#define SPI3_CLK_EN()				( RCC -> APB1ENR |= (1 << 15))
#define SPI4_CLK_EN()				( RCC -> APB2ENR |= (1 << 15))


//Clock Enable Macros for I2C
#define I2C1_CLK_EN()				(RCC -> APB1ENR |= (1 << 21))
#define I2C2_CLK_EN()				(RCC -> APB1ENR |= (1 << 22))
#define I2C3_CLK_EN()				(RCC -> APB1ENR |= (1 << 30))


//Clock Enable Macros for USART UART
#define USART1_CLK_EN()				(RCC -> APB2ENR |= (1 << 14))
#define USART2_CLK_EN()				(RCC -> APB1ENR |= (1 << 17))
#define USART3_CLK_EN()				(RCC -> APB1ENR |= (1 << 18))
#define UART4_CLK_EN()				(RCC -> APB1ENR |= (1 << 19))
#define UART5_CLK_EN()				(RCC -> APB1ENR |= (1 << 20))


//Clock Enable Macros for SYSCFG & EXTI
#define SYSCFG_CLK_EN()				( RCC -> APB2ENR |= (1 << 0))


//Clock Disable Macros for GPIOx
#define GPIOA_CLK_DI()				( RCC -> AHBENR &= ~(1 << 17))
#define GPIOB_CLK_DI()				( RCC -> AHBENR &= ~(1 << 18))
#define GPIOC_CLK_DI()				( RCC -> AHBENR &= ~(1 << 19))
#define GPIOD_CLK_DI()				( RCC -> AHBENR &= ~(1 << 20))
#define GPIOE_CLK_DI()				( RCC -> AHBENR &= ~(1 << 21))
#define GPIOF_CLK_DI()				( RCC -> AHBENR &= ~(1 << 22))
#define GPIOG_CLK_DI()				( RCC -> AHBENR &= ~(1 << 23))
#define GPIOH_CLK_DI()				( RCC -> AHBENR &= ~(1 << 16))


//Clock Disable Macros for SPI
#define SPI1_CLK_DI()				( RCC -> APB2ENR &= ~(1 << 12))
#define SPI2_CLK_DI()				( RCC -> APB1ENR &= ~(1 << 14))
#define SPI3_CLK_DI()				( RCC -> APB1ENR &= ~(1 << 15))
#define SPI4_CLK_DI()				( RCC -> APB2ENR &= ~(1 << 15))

//Clock Disable Macros for I2C
#define I2C1_CLK_DI()				(RCC -> APB1ENR &= ~(1 << 21))
#define I2C2_CLK_DI()				(RCC -> APB1ENR &= ~(1 << 22))
#define I2C3_CLK_DI()				(RCC -> APB1ENR &= ~(1 << 30))


//Clock Disable Macros for USART UART
#define USART1_CLK_DI()				(RCC -> APB2ENR &= ~(1 << 14))
#define USART2_CLK_DI()				(RCC -> APB1ENR &= ~(1 << 17))
#define USART3_CLK_DI()				(RCC -> APB1ENR &= ~(1 << 18))
#define UART4_CLK_DI()				(RCC -> APB1ENR &= ~(1 << 19))
#define UART5_CLK_DI()				(RCC -> APB1ENR &= ~(1 << 20))


//Clock Disable Macros for SYSCFG & EXTI
#define SYSCFG_CLK_DI()				( RCC -> APB2ENR &= ~(1 << 0))


//Gpio Reset Macro
#define GPIOA_REG_RST()				do {( RCC -> AHBRSTR |= (1 << 17));  ( RCC -> AHBRSTR &= ~(1 << 17));} while(0)
#define GPIOB_REG_RST()				do {( RCC -> AHBRSTR |= (1 << 18));  ( RCC -> AHBRSTR &= ~(1 << 18));} while(0)
#define GPIOC_REG_RST()				do {( RCC -> AHBRSTR |= (1 << 19));  ( RCC -> AHBRSTR &= ~(1 << 19));} while(0)
#define GPIOD_REG_RST()				do {( RCC -> AHBRSTR |= (1 << 20));  ( RCC -> AHBRSTR &= ~(1 << 20));} while(0)
#define GPIOE_REG_RST()				do {( RCC -> AHBRSTR |= (1 << 21));  ( RCC -> AHBRSTR &= ~(1 << 21));} while(0)
#define GPIOF_REG_RST()				do {( RCC -> AHBRSTR |= (1 << 22));  ( RCC -> AHBRSTR &= ~(1 << 22));} while(0)
#define GPIOG_REG_RST()				do {( RCC -> AHBRSTR |= (1 << 23));  ( RCC -> AHBRSTR &= ~(1 << 23));} while(0)
#define GPIOH_REG_RST()				do {( RCC -> AHBRSTR |= (1 << 16));  ( RCC -> AHBRSTR &= ~(1 << 16));} while(0)



#include "stm32f303xx_gpio_driver.h"

#endif /* INC_STM32F303XX_H_ */
