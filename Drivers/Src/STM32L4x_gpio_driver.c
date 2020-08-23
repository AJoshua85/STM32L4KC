/*
 * STM32L4x_gpio_driver.c
 *
 *  Created on: Jul. 14, 2020
 *      Author: Avinash
 */

#include <STM32L4x_gpio_driver.h>

/*******************************************************************
 * @fn				- GPIO_PclkCtr
 *
 * @brief			- This function enables or disables peripheral clock for the given port
 *
 * @parem[in]		- base address of gpio peripheral
 * @parem[in]		- ENABLE or DISABLE macros
 *
 * @return			- none
 * @note			- none
 */
void GPIO_PclkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}


	}
	else if (EnOrDi == DISABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}

	}
}


/*******************************************************************
 * @fn				- GPIO_Init
 *
 * @brief			- This function Initializes a given port
 *
 * @parem[in]		- base address of gpio peripheral
 *
 * @return			- none
 * @note			- none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	 uint32_t temp;
	 uint8_t regposition, bitpostion, portcode;

	GPIO_PclkCtrl(pGPIOHandle->pGPIOx,ENABLE);
	if(pGPIOHandle->GPIO_PinConfig.GPIO_Pinmode <= GPIO_MODE_ANALOG)
	{
		/*non interrupt mode*/
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_Pinmode << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &=~( 0x03 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); /*clear bits before setting*/
		pGPIOHandle->pGPIOx->MODER |= temp;

	}
	else
	{
		/* Interrupt mode falling Edge trigger*/
		if(pGPIOHandle->GPIO_PinConfig.GPIO_Pinmode == GPIO_MODE_IT_F_EDGE)
		{
			/*1 configure the Falling trigger selection register*/
			EXTI->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			/* configure RTSR bit just in case if it is toggled on*/
			EXTI->RTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		/* Interrupt mode rising Edge trigger*/
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_Pinmode == GPIO_MODE_IT_R_EDGE)
		{
			/*1 configure the Rising trigger selection register*/
			EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			/* configure FTSR bit just in case if it is toggled on*/
			EXTI->FTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		/* Interrupt mode falling and rising Edge trigger*/
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_Pinmode ==  GPIO_MODE_IT_RF_EDGE)
		{
			/*Configure both*/
			EXTI->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		/*Configure appropriate GPIO port selection in SYSCFG_EXTICR*/
		regposition = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		bitpostion = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;

		/*Enable SYSCFG*/
		SYSCFG_PCLK_EN();

		/*Configure port code bit mapping based on given port*/
		portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		/*Set the port code on the appropriator register*/
		SYSCFG->EXTICR[regposition] &= ~( 0xF <<( 4 * bitpostion ) );/*clear bits before setting*/
		SYSCFG->EXTICR[regposition] |= ( portcode << ( 4 * bitpostion ) );

		/*Enable the EXTI interrupt delivery using IMR*/
		EXTI->IMR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}
	/*Configure speed*/
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed <<( 2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &=~( 0x03 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));/*clear bits before setting*/
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	/*Configure the Pull up/down settings */
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdCtrl <<( 2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &=~( 0x03 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));/*clear bits before setting*/
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	/*Configure the output type*/
	temp =0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &=~( 0x03 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));/*clear bits before setting*/
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	if((pGPIOHandle->GPIO_PinConfig.GPIO_Pinmode == GPIO_MODE_ALTFN))
	{
		regposition = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;/*Calculate to see if the value goes high or low register(AFR[0]/AFR[1])*/
		bitpostion = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;/*Calculates the bit position offset to for given GPIO pin*/
		pGPIOHandle->pGPIOx->AFR[regposition] &= ~( 0xF <<( 4 * bitpostion) );/*clear bits before setting*/
		pGPIOHandle->pGPIOx->AFR[regposition]|= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFn <<( 4 * bitpostion) );
	}
}

/*******************************************************************
 * @fn				- GPIO_DeIit
 *
 * @brief			- This function resets a given port
 *
 * @parem[in]		- base address of gpio peripheral
 *
 * @return			- none
 * @note			- none
 */
void GPIO_DeIit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}

	else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}

}


/*******************************************************************
 * @fn				- GPIO_ReadFromInputPin
 *
 * @brief			- This function reads value from a specific pin
 *
 * @parem[in]		- base address of gpio peripheral
 * @parem[in]		- pin number
 *
 * @return			- pin value
 * @note			- none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNum) & (0x1));/* Grab the corresponding bit and mask of rest*/
	return value;
}
/*******************************************************************
 * @fn				- GPIO_ReadFromInputPort
 *
 * @brief			- This function reads value from a port
 *
 * @parem[in]		- base address of gpio peripheral
 *
 * @return			- port value
 * @note			- none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = pGPIOx->IDR;
	return value;
}
/*******************************************************************
 * @fn				- GPIO_WriteToOutputPin
 *
 * @brief			- This function writes a bit to a specific pin
 *
 * @parem[in]		- base address of gpio peripheral
 * @parem[in]		- pin number
 *
 * @return			- none
 * @note			- none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum,uint8_t Val)
{
	if(Val == PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNum);/*Write 1 to the output data register to corresponding pin position*/
	}
	else
	{
		pGPIOx->ODR &= ~ (1 << PinNum);/*Write 0 to the output data register to corresponding pin position*/
	}

}
/*******************************************************************
 * @fn				- GPIO_WriteToOutputPort
 *
 * @brief			- This function writes a value to a port
 *
 * @parem[in]		- base address of gpio peripheral
 * @parem[in]		- value for port
 *
 * @return			- none
 * @note			- none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Val)
{
	pGPIOx->ODR = Val;
}
/*******************************************************************
 * @fn				- GPIO_ToggleOutputPin
 *
 * @brief			- This function toggle a pin value on a port
 *
 * @parem[in]		- base address of gpio peripheral
 * @parem[in]		- pin number
 *
 * @return			- none
 * @note			- none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum)
{
	pGPIOx->ODR ^= ( 1 << PinNum);
}


/*******************************************************************
 * @fn				-  GPIO_IRQITConfig
 *
 * @brief			- This function enables interrupts
 *
 * @parem[in]		- IRQ number
 * @parem[in]		- Enable or Disable
 *
 * @return			- none
 * @note			- none
 */
void GPIO_IRQITConfig(uint8_t IRQNumber,uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if(IRQNumber <=31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);

		}

		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ISER1 |= (1 << ( IRQNumber % 32 ));

		}
		else if (IRQNumber >=64 && IRQNumber < 96)
		{
			*NVIC_ISER2 |= (1 << ( IRQNumber % 64 ));
		}
	}
	else
	{
		if(IRQNumber <=31)
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		}

		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ICER1 |= (1 << ( IRQNumber % 32 ));
		}
		else if (IRQNumber >=64 && IRQNumber < 96)
		{
			*NVIC_ICER2 |= (1 << ( IRQNumber % 64 ));
		}
	}

}
/*******************************************************************
 * @fn				-  GPIO_IRQPriorityConfig
 *
 * @brief			- This function configures the priority of the interrupt
 *
 * @parem[in]		- IRQ number
 * @parem[in]		- IRQ priority
 *
 * @return			- none
 * @note			- none
 */

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx_register = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8-NO_PR_BITS_IMPLEMENTED);// This may vary depending on manufacture implementation

	*(NVIC_PR_BASE_ADDR + (iprx_register *4)) &= ~( 0xF << shift_amount);//clear bits before setting
	*(NVIC_PR_BASE_ADDR + (iprx_register *4)) |= ( IRQPriority << shift_amount);

}
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register corresponding to the pin number
	if(EXTI->PR1 & (1 <<PinNumber))
	{
		//clear
		EXTI->PR1 |=( 1  <<PinNumber);
	}

}
