/*
 * STM32L432xx.h
 *
 *  Created on: Aug 13, 2020
 *      Author: Avinash
 */
#include <stdint.h>
#ifndef INC_STM32L432XX_H_
#define INC_STM32L432XX_H_

/************************ Processor Specific Details**************************
 *
 * ARM Cortex M4 Processor NVIC ISERx register Addrs
 *
 */

#define NVIC_ISER0     	((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1  	((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2  	((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3 		((volatile uint32_t*)0xE000E10C)
#define NVIC_ISER3 		((volatile uint32_t*)0xE000E10C)

/* ARM Cortex M4 Processor NVIC ICERx register Addrs*/
#define NVIC_ICER0  	((volatile uint32_t*)0xE000E180)
#define NVIC_ICER1  	((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2  	((volatile uint32_t*)0xE000E188)
#define NVIC_ICER3 		((volatile uint32_t*)0xE000E18C)


/*ARM Cortex M4 Processor Priority Addrs */
#define NVIC_PR_BASE_ADDR	((volatile uint32_t*) 0xE000E400)

/* ARM Cortex M4 processor number of priority bits implemented in priority register*/
#define NO_PR_BITS_IMPLEMENTED 		4

/* base addresses of Flash and SRAM memories*/
#define FLASH_BASEADDR 	0x08000000U
#define SRAM_BASEADDR 	0x20000000U
#define ROM_BASEADDR 	0x1FFF0000U
#define ROM 			ROM_BASEADDR
#define SRAM 			SRAM_BASEADDR


/* AHBx and APBx Bus Peripheral base addr*/
#define PERIPH_BASE		0x40000000U
#define APB1PERIPH_BASE PERIPH_BASE
#define APB2PERIPH_BASE 0x40010000U
#define AHB1PERIPH_BASE 0x40020000U
#define AHB2PERIPH_BASE 0x48000000U

/* base addrs of peripherals which are on AHB2 bus*/
#define GPIOA_BASEADDR 	( AHB2PERIPH_BASE )
#define GPIOB_BASEADDR 	( AHB2PERIPH_BASE + 0x400 )
#define GPIOC_BASEADDR 	( AHB2PERIPH_BASE + 0x800 )

/* base addrs of peripherals which are on APB1 bus*/
#define I2C1_BASEADDR	( APB1PERIPH_BASE + 0x5400 )
#define I2C3_BASEADDR	( APB1PERIPH_BASE + 0x5C00 )

/* base addrs of peripherals which are on AHB1 bus*/
#define RCC_BASEADDR	( AHB1PERIPH_BASE + 0x1000 )


/* base addrs of peripherals which are on APB2 bus*/
#define SYSCFG_BASEADDR ( APB2PERIPH_BASE )
#define EXTI_BASEADDR 	( APB2PERIPH_BASE + 0x400 )
#define SPI_BASEADDR	( APB2PERIPH_BASE + 0x3000 )
#define USART1_BASEADDR	( APB2PERIPH_BASE + 0x3800 )

/**********************peripheral register definition structures**************/

/* peripheral register definition structure for GPIO Register*/
typedef struct
{
	volatile uint32_t MODER; 		/*GPIO port mode register*/
	volatile uint32_t OTYPER;		/*GPIO port output speed register*/
	volatile uint32_t OSPEEDR;		/*GPIO port output speed register*/
	volatile uint32_t PUPDR;			/*GPIO port pull-up/pull-down register*/
	volatile uint32_t IDR;			/*GPIO port input data register*/
	volatile uint32_t ODR;			/*GPIO port output data register */
	volatile uint32_t BSRR;			/*GPIO port bit set/reset register*/
	volatile uint32_t LCKR;			/*GPIO port configuration lock register*/
	volatile uint32_t AFR[2];		/*GPIO alternate function; AFR[0] alternate function low register, AFR[1] alternate function high register */
	volatile uint32_t BRR;			/*GPIO port bit reset register*/
}GPIO_RegDef_t;

/* peripheral register definition structure for RCC Register*/
typedef struct
{
	volatile uint32_t CR;			/*Clock control register */
	volatile uint32_t ICSCR;			/*Internal clock sources calibration register*/
	volatile uint32_t CFGR;			/*Clock configuration register*/
	volatile uint32_t PLLCFGR;		/*PLL configuration register*/
	volatile uint32_t PLLSAI1CFGR;	/*PLLSAI1 configuration register*/
	uint32_t reserved0;
	volatile uint32_t CIER;			/*Clock interrupt enable register*/
	volatile uint32_t CIFR;			/*Clock interrupt flag register */
	volatile uint32_t CICR;			/*Clock interrupt clear register */
	uint32_t reserved1;
	volatile uint32_t AHB1RSTR;		/*AHB1 peripheral reset register*/
	volatile uint32_t AHB2RSTR;		/*AHB2 peripheral reset register*/
	volatile uint32_t AHB3RSTR;		/*AHB3 peripheral reset register */
	uint32_t reserved2;
	volatile uint32_t APB1RSTR1;	/*APB1 peripheral reset register 1*/
	volatile uint32_t APB1RSTR2;	/*APB1 peripheral reset register 2*/
	volatile uint32_t APB2RSTR;		/*APB2 peripheral reset register*/
	uint32_t reserved3;
	volatile uint32_t AHB1ENR;		/*AHB1 peripheral clock enable register */
	volatile uint32_t AHB2ENR;		/*AHB2 peripheral clock enable register*/
	volatile uint32_t AHB3ENR;		/*AHB3 peripheral clock enable register*/
	uint32_t reserved4;
	volatile uint32_t APB1ENR1;		/*APB1 peripheral clock enable register 1*/
	volatile uint32_t APB1ENR2;		/*APB1 peripheral clock enable register 2*/
	volatile uint32_t APB2ENR;		/*APB2 peripheral clock enable register*/
	uint32_t reserved5;
	volatile uint32_t AHB1SMENR;	/*AHB1 peripheral clocks enable in Sleep and Stop modes register*/
	volatile uint32_t AHB2SMENR;	/*AHB2 peripheral clocks enable in Sleep and Stop modes register*/
	volatile uint32_t AHB3SMENR;    /*AHB3 peripheral clocks enable in Sleep and Stop modes register*/
	uint32_t reserved6;
	volatile uint32_t APB1SMENR1;	/*APB1 peripheral clocks enable in Sleep and Stop modes register 1*/
	volatile uint32_t APB1SMENR2;	/*APB1 peripheral clocks enable in Sleep and Stop modes register 2*/
	volatile uint32_t APB2SMENR;	/*APB2 peripheral clocks enable in Sleep and Stop modes register*/
	uint32_t reserved7;
	volatile uint32_t CCIPR;		/*Peripherals independent clock configuration register*/
	volatile uint32_t BDCR;			/*Backup domain control register*/
	volatile uint32_t CSR;			/*Control/status register*/
	volatile uint32_t CRRCR;		/*Clock recovery RC register*/
	volatile uint32_t CCIPR2;		/*Peripherals independent clock configuration register*/
}RCC_RegDef_t;

/* peripheral register definition structure for EXTI Register*/
typedef struct
{
	volatile uint32_t IMR1;			/*Interrupt mask register*/
	volatile uint32_t EMR1;			/*Event mask register */
	volatile uint32_t RTSR1;		/*Rising trigger selection register*/
	volatile uint32_t FTSR1;		/*Falling trigger selection register*/
	volatile uint32_t SWIER1;		/*Software interrupt event register*/
	volatile uint32_t PR1;			/*Pending register*/
	uint32_t reserved [2];
	volatile uint32_t IMR2;			/*Interrupt mask register*/
	volatile uint32_t EMR2;			/*Event mask register */
	volatile uint32_t RTSR2;		/*Rising trigger selection register*/
	volatile uint32_t FTSR2;		/*Falling trigger selection register*/
	volatile uint32_t SWIER2;		/*Software interrupt event register*/
	volatile uint32_t PR2;			/*Pending register*/
}EXTI_RegDef_t;

/* peripheral register definition structure for SYSCFG Register*/
typedef struct
{
	volatile uint32_t MEMRMP;
	volatile uint32_t CFGR1; 		/*SYSCFG configuration register 1*/
	volatile uint32_t EXTICR[4];	/*SYSCFG external interrupt configuration register 1 to 4*/
	volatile uint32_t SCSR;			/*SYSCFG SRAM2 control and status register*/
	volatile uint32_t CFGR2;		/*SYSCFG configuration register 2*/
	volatile uint32_t SWPR;			/*SYSCFG SRAM2 write protection register*/
	volatile uint32_t SKR;			/*SYSCFG SRAM2 key register*/
}SYSCFG_RegDef_t;


/* peripheral register definition structure for SPI Register*/
typedef struct
{
	volatile uint32_t CR1;/*SPI control register 1*/
	volatile uint32_t CR2;/*SPI control register 2*/
	volatile uint32_t SR;/*SPI status register*/
	volatile uint32_t DR;/*SPI data register*/
	volatile uint16_t reserved;/**/
	volatile uint32_t CRCPR;/*SPI CRC polynomial register*/
	volatile uint32_t RXCRCR;/*SPI Rx CRC register*/
	volatile uint32_t TXCRCR;/*SPI Tx CRC register*/

}SPI_RegDef_t;

/* peripheral register definition structure for I2C Register*/
typedef struct
{
	volatile uint32_t CR1;/*I2C control register 1*/
	volatile uint32_t CR2;/*I2C control register 2*/
	volatile uint32_t OAR1;/*I2C own address 1 register*/
	volatile uint32_t OAR2;/*I2C own address 2 register*/
	volatile uint32_t TIMINGR;/*I2C timing register*/
	volatile uint32_t TIMEOUTR;/*I2C timeout register*/
	volatile uint32_t ISR;/*I2C interrupt and status register*/
	volatile uint32_t ICR;/*I2C interrupt clear register*/
	volatile uint32_t PECR;/*I2C PEC register*/
	volatile uint32_t RXDR;/*I2C receive data register*/
	volatile uint32_t TXDR;/*I2C transmit data register*/
}I2C_RegDef_t;

/* peripheral register definition structure for USART Register*/
typedef struct
{
	volatile uint32_t CR1;/*Control register 1*/
	volatile uint32_t CR2;/*Control register 2*/
	volatile uint32_t CR3;/*Control register 3*/
	volatile uint32_t BRR;/*Baud rate register*/
	volatile uint32_t GTPR;/*Guard time and prescaler register*/
	volatile uint32_t RTOR;/*Receiver timeout register*/
	volatile uint32_t RQR;/*Request register*/
	volatile uint32_t ISR;/*Interrupt and status register*/
	volatile uint32_t ICR;/*Interrupt flag clear register*/
	volatile uint32_t RDR;/*Receive data register*/
	volatile uint32_t TDR;/*Transmit data register*/
}USART_RegDef_t;

/* peripheral register definition structure for I2C Register*/

/*  peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t*/
#define GPIOA					( (GPIO_RegDef_t*) GPIOA_BASEADDR )
#define GPIOB					( (GPIO_RegDef_t*) GPIOB_BASEADDR )
#define GPIOC					( (GPIO_RegDef_t*) GPIOC_BASEADDR )
#define RCC						( (RCC_RegDef_t*)  RCC_BASEADDR )
#define EXTI					( (EXTI_RegDef_t*) EXTI_BASEADDR )
#define SYSCFG					( (SYSCFG_RegDef_t*) SYSCFG_BASEADDR )
#define SPI						( (SPI_RegDef_t*) SPI_BASEADDR)
#define I2C1					( (I2C_RegDef_t*) I2C1_BASEADDR)
#define I2C3					( (I2C_RegDef_t*) I2C3_BASEADDR)

/* Clock Enable Macros for GPIOx peripherals*/
#define GPIOA_PCLK_EN()			( RCC->AHB2ENR |= ( 1 << 0 ) )
#define GPIOB_PCLK_EN()			( RCC->AHB2ENR |= ( 1 << 1 ) )
#define GPIOC_PCLK_EN()			( RCC->AHB2ENR |= ( 1 << 2 ) )


/* Clock Enable Macros for I2Cx peripherals*/
#define I2C1_PCLK_EN() 			( RCC->APB1ENR1 |= ( 1 << 21 ) )
#define I2C3_PCLK_EN() 			( RCC->APB1ENR1 |= ( 1 << 23 ) )

/* Clock Enable Macros for SPIx peripherals*/
#define SPI1_PCLK_EN() 			( RCC->APB2ENR |= ( 1 << 12 ) )

/*Clock Disable Macros for GPIOx peripherals */
#define GPIOA_PCLK_DI()			( RCC->AHB2ENR &= ~ ( 1 << 0 ) )
#define GPIOB_PCLK_DI()			( RCC->AHB2ENR &= ~ ( 1 << 1 ) )
#define GPIOC_PCLK_DI()			( RCC->AHB2ENR &= ~ ( 1 << 2 ) )

/*Clock Enable Macros for SYSCFG peripheral*/
#define SYSCFG_PCLK_EN() 		( RCC->APB2ENR |= ( 1 << 0 ) )

/* Clock Disable Macros for I2Cx peripherals*/
#define I2C1_PCLK_DI() 			( RCC->APB1ENR1 &= ~ ( 1 << 21 ) )
#define I2C3_PCLK_DI() 			( RCC->APB1ENR1 &= ~  ( 1 << 23 ))

/* Clock Disable Macros for SPIx peripherals*/
#define SPI1_PCLK_DI() 			( RCC->APB2ENR &= ~ ( 1 << 12 ) )

/* Macros to reset GPIOx peripherals*/
#define GPIOA_REG_RESET()		 do{ ( RCC->AHB2RSTR |= ( 1 << 0 ) ); ( RCC->AHB2RSTR &= ~( 1 << 0 ) ); }while(0)
#define GPIOB_REG_RESET()		 do{ ( RCC->AHB2RSTR |= ( 1 << 1 ) ); ( RCC->AHB2RSTR &= ~( 1 << 1 ) ); }while(0)
#define GPIOC_REG_RESET()		 do{ ( RCC->AHB2RSTR |= ( 1 << 2 ) ); ( RCC->AHB2RSTR &= ~( 1 << 2 ) ); }while(0)

/*returns port code for given GPIOx base address*/
#define GPIO_BASEADDR_TO_CODE(x)	(	( x == GPIOA ) ? 0: \
										( x == GPIOB ) ? 1: \
										( x == GPIOC ) ? 2: 0) \
/*generic macros*/
#define ENABLE				1U
#define DISABLE 			0U
#define SET 				ENABLE
#define RESET				DISABLE
#define PIN_SET 			SET
#define PIN_REST 			RESET
#define FLAG_RESET			RESET
#define FLAG_SET			SET
#define FAIL				SET
#define SUCCESS				RESET


#endif /* INC_STM32L432XX_H_ */
