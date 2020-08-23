/*
 * STM32L432X_gpio_driver.h
 *
 *  Created on: Jul. 13, 2020
 *      Author: Avinash
 */

#ifndef INC_STM32L432X_GPIO_DRIVER_H_
#define INC_STM32L432X_GPIO_DRIVER_H_

#include "STM32L432xx.h"

/*Configuration structure for GPIOx peripheral*/
typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_Pinmode;		/* !<possible values from @GPIO_PIN_MODES>*/
	uint8_t GPIO_PinSpeed;		/* !<possible values from @GPIO_PIN_SPEED>*/
	uint8_t GPIO_PinPuPdCtrl;	/* !<possible values from @GPIO_PIN_PU_PD_CONFIG>*/
	uint8_t	GPIO_PinOPType;		/* !<possible values from @GPIO_PIN_OUTPUT_TYPE>*/
	uint8_t GPIO_PinAltFn;
}GPIO_PinConfig_t;

/*Structure handle for a GPIO pin*/
typedef struct
{
	GPIO_RegDef_t *pGPIOx; /* Stores base address of GPIO port it is configured to*/
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

/*@GPIO_PIN_MODES
 * GPIO pin possible modes*/
#define GPIO_MODE_INPUT 		0
#define GPIO_MODE_OUTPUT		1
#define GPIO_MODE_ALTFN			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_F_EDGE		4
#define GPIO_MODE_IT_R_EDGE		5
#define GPIO_MODE_IT_RF_EDGE	6

/*@GPIO_PIN_OUTPUT_TYPE
 * GPIO pin possible output types*/
#define GPIO_OP_TYPE_PP			0
#define GPIO_OP_TYPE_OD			1

/*@GPIO_PIN_SPEED
 * GPIO pin possible output speeds*/
#define GPIO_SPD_LOW			0
#define GPIO_SPD_MED			1
#define GPIO_SPD_FAST			2
#define GPIO_SPD_HIGH			3

/*@GPIO_PIN_PU_PD_CONFIG
 * GPIO pin pull up and pull down configuration macros*/
#define GPIO_NO_PUPD			0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2

/*****************************************************************************
 * 						APIs supported by this driver
 * 		For more information about the APIs check the function definitions
 *****************************************************************************/

/*Peripheral Clock setup*/
void GPIO_PclkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);

/*Init and De-init*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeIit(GPIO_RegDef_t *pGPIOx);


/*Data read and write*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum,uint8_t Val);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Val);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum);

/*IRQ Configuration and ISR Handling*/
void GPIO_IRQITConfig(uint8_t IRQNumber,uint8_t EnOrDi);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32L432X_GPIO_DRIVER_H_ */
