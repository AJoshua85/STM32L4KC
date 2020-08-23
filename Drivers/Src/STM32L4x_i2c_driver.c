/*
 * STM32L4x_i2c_driver.c
 *
 *  Created on: Aug. 14, 2020
 *      Author: Avinash
 */
#include "STM32L4x_i2c_driver.h"
#include "STM32L432xx.h"

static void I2C_Start(I2C_RegDef_t *pI2Cx,uint8_t direction, uint8_t SlaveAddr, uint8_t size);
static I2CState checkI2CBus(I2C_RegDef_t *pI2Cx);
static IT_EV ITFlag;
static uint8_t dataDir;
uint8_t tsize;

/*******************************************************************
 * @fn				-  I2C_PclkCtrl
 *
 * @brief			- this function enables I2C peripheral clock
 * @parem[in]		- base address of of I2C peripheral
 * @parem[in]	    - enable or disable
 * @return			- none
 * @note			- none
 */
void I2C_PclkCtrl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}

		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else if (EnOrDi == DISABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}
/*******************************************************************
 * @fn				-  I2C_PeripheralControl
 *
 * @brief			- this function enables I2C peripheral
 * @parem[in]		- base address of of I2C peripheral
 * @parem[in]	    - enable or disable
 * @return			- none
 * @note			- none
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{

	if(EnOrDi== ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else if(EnOrDi == DISABLE)
	{
		while(I2C_GetFlagStatus(pI2Cx,I2C_BUSY_FLAG));
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}
/*******************************************************************
 * @fn				- I2C_Init
 *
 * @brief			- this function configures the I2C peripheral timings
 * @parem[in]		- base address of of I2C peripheral
 * @return			- none
 * @note			- none
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	//Enable I2C peripheral clock*
	I2C_PclkCtrl(pI2CHandle->pI2Cx,ENABLE);

	//Disable slave clock stretching
	//pI2CHandle->pI2Cx->CR1 |= 1<<I2C_CR1_NOSTRETCH;

	//Enable auto end
	pI2CHandle->pI2Cx->CR2|= 1 <<I2C_CR2_AUTOEND;

	//configure SCL timings
	tempreg |= pI2CHandle->I2C_Config.I2C_SCLL <<I2C_TIMINGR_SCLL;
	tempreg |=pI2CHandle->I2C_Config.I2C_SCLH <<I2C_TIMINGR_SCLH;
	tempreg |=pI2CHandle->I2C_Config.I2C_SDADEL <<I2C_TIMINGR_SDADEL;
	tempreg |=pI2CHandle->I2C_Config.I2C_SCLDEL <<I2C_TIMINGR_SCLDEL;
	tempreg |=pI2CHandle->I2C_Config.I2C_PRESC <<I2C_TIMINGR_PRESC;
	pI2CHandle->pI2Cx->TIMINGR = tempreg;

	//configure address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddr <<I2C_OAR1_ADDR_7BIT;
 	tempreg |= (1 << I2C_OAR1_OA1EN);
 	pI2CHandle->pI2Cx->OAR1 = tempreg;
 	pI2CHandle->TxRxState=I2C_READY;
 }
/*******************************************************************
 * @fn				- I2C_Start
 *
 * @brief			- this function initiates the start condition for I2C transmission
 * @parem[in]		- base address of of I2C peripheral
 * @parem[in]		- read or write access for the I2C slave
 * @parem[in]		- slave address
 * @parem[in]		- number of bytes to be sent
 * @return			- none
 * @note			- none
 */
static void I2C_Start(I2C_RegDef_t *pI2Cx,uint8_t direction, uint8_t SlaveAddr, uint8_t size)
{
	uint32_t tempreg = pI2Cx->CR2;
	if(direction == I2C_MASTER_WR)
	{
		tempreg &= ~(1 << I2C_CR2_RD_WRN);
	}
	else if (direction == I2C_MASTER_RD)
	{
		tempreg |= (1 << I2C_CR2_RD_WRN);
	}

	//Clear address Field and length field
	tempreg &= ~ ( (0x3FF) | (0xFF << 16) );
	tempreg |= (SlaveAddr << 1)|(size << 16);
	tempreg |=  (1 << I2C_CR2_START);
	pI2Cx->CR2 = tempreg;
}
/*******************************************************************
 * @fn				- I2C_IRQITConfig
 *
 * @brief			- this function enable or disable interrupt for given interrupt
 * @parem[in]		- IRQ number
 * @parem[in]		- enable or disable
 * @return			- none
 * @note			- none
 */
void I2C_IRQITConfig(uint8_t IRQNumber,uint8_t EnOrDi)
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
 * @fn				- I2C_IRQPriorityConfig
 *
 * @brief			- this function configure the priority of the I2C interrupt
 * @parem[in]		- I2C IRQ number
 * @parem[in]		- IRQ priority to be set
 * @return			- none
 * @note			- none
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx_register = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8-NO_PR_BITS_IMPLEMENTED);// This may vary depending on manufacture implementation

	*(NVIC_PR_BASE_ADDR + (iprx_register *4)) &= ~( 0xF << shift_amount);//clear bits before setting
	*(NVIC_PR_BASE_ADDR + (iprx_register *4)) |= ( IRQPriority << shift_amount);
}
/*******************************************************************
 * @fn				- I2C_ITCntrl
 *
 * @brief			- this function sends data to I2C bus via an interrupt
 *
 * @parem[in]		- base address of of I2C peripheral
 * @parem[in]		- interrupt mask for a given interrupt
 * @parem[in]		- enable or disable
 * @return
 * @note			This function can enable/disable multiple interrupts by combining the masks together
 */
void I2C_ITCntrl(I2C_RegDef_t *pI2Cx,uint8_t interrupt ,uint8_t EnOrDi)
{
	if(pI2Cx == I2C1)
	{
		if((interrupt & I2C_TXIE_IT) &&  EnOrDi == ENABLE)
		{
			pI2Cx->CR1 |= (1 << I2C_CR1_TXIE);
		}

		else if((interrupt & I2C_TXIE_IT) &&  EnOrDi == DISABLE)
		{
			pI2Cx->CR1 &= ~(1 << I2C_CR1_TXIE);
		}

		if((interrupt & I2C_RXIE_IT) &&  EnOrDi == ENABLE)
		{
			pI2Cx->CR1 |= (1 << I2C_CR1_RXIE);
		}

		else if((interrupt & I2C_RXIE_IT) &&  EnOrDi == DISABLE)
		{
			pI2Cx->CR1 &= ~(1 << I2C_CR1_RXIE);
		}

		if((interrupt & I2C_ADDRIE_IT) &&  EnOrDi == ENABLE)
		{
			pI2Cx->CR1 |= (1 << I2C_CR1_ADDRIE);
		}

		else if((interrupt & I2C_ADDRIE_IT) &&  EnOrDi == DISABLE)
		{
			pI2Cx->CR1 &= ~(1 << I2C_CR1_ADDRIE);
		}

		if((interrupt & I2C_NACKIE_IT) &&  EnOrDi == ENABLE)
		{
			pI2Cx->CR1 |= (1 << I2C_CR1_NACKIE);
		}

		else if((interrupt & I2C_NACKIE_IT) &&  EnOrDi == DISABLE)
		{
			pI2Cx->CR1 &= ~(1 << I2C_CR1_NACKIE);
		}

		if((interrupt & I2C_STOPIE_IT) &&  EnOrDi == ENABLE)
		{
			pI2Cx->CR1 |= (1 << I2C_CR1_STOPIE);
		}

		else if((interrupt & I2C_STOPIE_IT) &&  EnOrDi == DISABLE)
		{
			pI2Cx->CR1 &= ~(1 << I2C_CR1_STOPIE);
		}
	}

	else if (pI2Cx == I2C3)
	{
		if((interrupt & I2C_TXIE_IT) &&  EnOrDi == ENABLE)
		{
			pI2Cx->CR1 |= (1 << I2C_CR1_TXIE);
		}

		else if((interrupt & I2C_TXIE_IT) &&  EnOrDi == DISABLE)
		{
			pI2Cx->CR1 &= ~(1 << I2C_CR1_TXIE);
		}

		if((interrupt & I2C_RXIE_IT) &&  EnOrDi == ENABLE)
		{
			pI2Cx->CR1 |= (1 << I2C_CR1_RXIE);
		}

		else if((interrupt & I2C_RXIE_IT) &&  EnOrDi == DISABLE)
		{
			pI2Cx->CR1 &= ~(1 << I2C_CR1_RXIE);
		}

		if((interrupt & I2C_ADDRIE_IT) &&  EnOrDi == ENABLE)
		{
			pI2Cx->CR1 |= (1 << I2C_CR1_ADDRIE);
		}

		else if((interrupt & I2C_ADDRIE_IT) &&  EnOrDi == DISABLE)
		{
			pI2Cx->CR1 &= ~(1 << I2C_CR1_ADDRIE);
		}

		if((interrupt & I2C_NACKIE_IT) &&  EnOrDi == ENABLE)
		{
			pI2Cx->CR1 |= (1 << I2C_CR1_NACKIE);
		}

		else if((interrupt & I2C_NACKIE_IT) &&  EnOrDi == DISABLE)
		{
			pI2Cx->CR1 &= ~(1 << I2C_CR1_NACKIE);
		}

		if((interrupt & I2C_STOPIE_IT) &&  EnOrDi == ENABLE)
		{
			pI2Cx->CR1 |= (1 << I2C_CR1_STOPIE);
		}

		else if((interrupt & I2C_STOPIE_IT) &&  EnOrDi == DISABLE)
		{
			pI2Cx->CR1 &= ~(1 << I2C_CR1_STOPIE);
		}
	}
}
/*******************************************************************
 * @fn				- I2C_GetFlagStatus
 *
 * @brief			- reads the ISR register to determine which interrupt has occurred
 * @parem[in]		- base address of of I2C peripheral
 * @parem[in]		- Interrupt to check
 * @return			- 0 or 1
 * @note			- none
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{

	if(pI2Cx->ISR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}
/*******************************************************************
 * @fn				- I2CReadStatusFlag
 *
 * @brief			- checks to see which interrupt has occurred and sets ITFlag
 * @parem[in]		- base address of of I2C peripheral
 * @return			- none
 * @note			- none
 */
void I2CReadStatusFlag(I2C_RegDef_t *pI2Cx)
{
	if(I2C_GetFlagStatus(pI2Cx,I2C_ADDR_FLAG))
	{
		ITFlag = ADDRMATCH;
	}
	else if(I2C_GetFlagStatus(pI2Cx,I2C_TXIS_FLAG))
	{
		ITFlag = TXISREADY;
	}

	else if(I2C_GetFlagStatus(pI2Cx,I2C_RXNE_FLAG))
	{
		ITFlag = RXEREADY;
	}

	else if(I2C_GetFlagStatus(pI2Cx, I2C_STOPF_FLAG))
	{
		ITFlag = STOPBIT;
	}

	else if(I2C_GetFlagStatus(pI2Cx,I2C_NACKF_FLAG))
	{
		ITFlag = NACKF;
	}
}
/*******************************************************************
 * @fn				- getI2CFlag
 *
 * @brief			- returns the Interrupt Flag that has been triggered
 * @parem[in]		- none
 * @return			- IT_EV
 * @note			- none
 */
uint8_t getI2CFlag(void)
{
	return ITFlag;
}
/*******************************************************************
 * @fn				- addressMatchEvent
 *
 * @brief			- this function acknowledges ADDR interrupt
 * @parem[in]		- I2C Configuration
 * @return			- none
 * @note			- none
 */
void addressMatchEvent(I2C_RegDef_t *pI2Cx)
{
	//Write transfer
	if((pI2Cx->ISR) & I2C_DIR_FLAG)
	{
		dataDir =SLAVE_TRANSMITTER;
		//flush the transmit data register
		pI2Cx->ISR |= 1 << I2C_ISR_TXE;
	}
	//Read transfer
	else
	{
		dataDir =SLAVE_RECEIVER;
	}
	//Clear ADDR interrupt
	pI2Cx->ICR |= 1 << I2C_ICR_ADDRCF;
}
/*******************************************************************
 * @fn				- recieverBufferFullEvent
 *
 * @brief			- this function acknowledges RXNE interrupt and reads the data
 * 					  from the I2C Bus
 *
 * @parem[in]		- I2C Configuration
 * @return			- none
 * @note			- none
 */
void recieverBufferFullEvent(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX_MASTER)
	{
		if(pI2CHandle->RxLen > 0)
		{
			*(pI2CHandle->pRxBuffer)= pI2CHandle->pI2Cx->RXDR;
			pI2CHandle->pRxBuffer++;
			pI2CHandle->RxLen--;

			if(pI2CHandle->RxLen == 0)
			{
				pI2CHandle->TxRxState= I2C_READY;
			}
		}
	}
	else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX_SLAVE)
	{
		if(pI2CHandle->RxLen > 0)
		{
			*(pI2CHandle->pRxBuffer)= pI2CHandle->pI2Cx->RXDR;
			pI2CHandle->pRxBuffer++;
			pI2CHandle->RxLen--;
		}
	}
}
/*******************************************************************
 * @fn				- stopFlagEvent
 *
 * @brief			- this function acknowledges STOPF interrupt
 *
 * @parem[in]		- I2C Configuration
 * @return			- none
 * @note			- none
 */
void stopFlagEvent(I2C_Handle_t *pI2CHandle)
{
	//Clear stop flag interrupt
	pI2CHandle->pI2Cx->ICR |= 1 << I2C_ICR_STOPCF;

	if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX_SLAVE)
	{
		I2C_ITCntrl(pI2CHandle->pI2Cx,I2C_RXIE_IT,DISABLE);
	}
	else if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX_SLAVE)
	{
		I2C_ITCntrl(pI2CHandle->pI2Cx,I2C_TXIE_IT,DISABLE);
	}
	pI2CHandle->TxRxState= I2C_READY;
}
/*******************************************************************
 * @fn				- nackEvent
 *
 * @brief			- this function acknowledges NACKF interrupt
 *
 * @parem[in]		- I2C Configuration
 * @return			- none
 * @note			- none
 */
void nackEvent(I2C_Handle_t *pI2CHandle)
{

	//Master transmit failed
	if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX_MASTER)
	{
		//Clear NACKF flag event
		pI2CHandle->pI2Cx->ICR |= 1 << I2C_ICR_NACKCF;
		pI2CHandle->TxLen= 0;
		pI2CHandle->TxRxState= I2C_READY;
	}
	//Master receive fail
	else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX_MASTER)
	{
		//Clear NACKF flag event
		pI2CHandle->pI2Cx->ICR |= 1 << I2C_ICR_NACKCF;
		pI2CHandle->RxLen= 0;
		pI2CHandle->TxRxState= I2C_READY;
	}

	// Slave transmit fail
	else if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX_SLAVE && pI2CHandle->TxLen > 0)
	{
			//Clear NACKF flag event
			pI2CHandle->pI2Cx->ICR |= 1 << I2C_ICR_NACKCF;
			pI2CHandle->TxLen= 0;
			pI2CHandle->TxRxState= I2C_READY;
	}

	// Slave successful transmit to master
	else if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX_SLAVE && pI2CHandle->TxLen == 0)
	{
		//Clear NACKF flag event
		pI2CHandle->pI2Cx->ICR |= 1 << I2C_ICR_NACKCF;
		pI2CHandle->TxLen= 0;
	}
}
/*******************************************************************
 * @fn				- transmitReadyEvent
 *
 * @brief			- this function acknowledges TXIS interrupt and transfers data
 * 					  to the I2C bus
 *
 * @parem[in]		- I2C Configuration
 * @return			- none
 * @note			- none
 */
void transmitReadyEvent(I2C_Handle_t *pI2CHandle)
{
	uint8_t temp =3;

	if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX_MASTER)
	{
		if(pI2CHandle->TxLen > 0)
		{
			pI2CHandle->pI2Cx->TXDR = *(pI2CHandle->pTxBuffer);
			pI2CHandle->pTxBuffer++;
			pI2CHandle->TxLen--;

			//Notify main transmit is done
			if(pI2CHandle->TxLen == 0)
			{
				pI2CHandle->TxRxState = I2C_READY;
			}
		}
		else
		{
			pI2CHandle->pI2Cx->TXDR = temp;
		}
	}
	else if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX_SLAVE)
	{
		if(pI2CHandle->TxLen > 0)
		{
			pI2CHandle->pI2Cx->TXDR = *(pI2CHandle->pTxBuffer);
			pI2CHandle->pTxBuffer++;
			pI2CHandle->TxLen--;
		}
		else
		{
			pI2CHandle->pI2Cx->TXDR = temp;
		}

	}
}
/*******************************************************************
 * @fn				- I2C_MasterSendDataIT
 *
 * @brief			- this function transmits data to the I2C bus via an interrupt
 *
 * @parem[in]		- I2C Configuration
 * @parem[in]		- transmit buffer
 * @parem[in]		- length of transmit buffer
 * @parem[in]		- Slave address
 * @return			- 0 or 1
 * @note			- none
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t size, uint8_t SlaveAddr)
{
	if(pI2CHandle->TxRxState == I2C_READY && checkI2CBus(pI2CHandle->pI2Cx))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = size;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX_MASTER;
		I2C_ITCntrl(pI2CHandle->pI2Cx,I2C_NACKIE_IT|I2C_TXIE_IT,ENABLE);
		I2C_Start(pI2CHandle->pI2Cx,I2C_MASTER_WR,SlaveAddr,size);
		return SUCCESS;
	}
	return FAIL;
}
/*******************************************************************
 * @fn				- I2C_MasterRecieveDataIT
 *
 * @brief			- this function receives data to the I2C bus via an interrupt
 *
 * @parem[in]		- I2C Configuration
 * @parem[in]		- receive buffer
 * @parem[in]		- length of receive buffer
 * @parem[in]		- slave address
 * @return			- 0 or 1
 * @note			- none
 */
uint8_t I2C_MasterRecieveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t size, uint8_t SlaveAddr)
{
	if(pI2CHandle->TxRxState == I2C_READY && checkI2CBus(pI2CHandle->pI2Cx))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = size;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX_MASTER;
		I2C_ITCntrl(pI2CHandle->pI2Cx,I2C_NACKIE_IT|I2C_RXIE_IT,ENABLE);
		I2C_Start(pI2CHandle->pI2Cx,I2C_MASTER_RD,SlaveAddr,size);
		return SUCCESS;
	}
	return FAIL;
}
/*******************************************************************
 * @fn				- SlaveRecieveDataIT
 *
 * @brief			- this function receives data to the I2C bus via an interrupt
 *
 * @parem[in]		- I2C Configuration
 * @parem[in]		- receive buffer
 * @parem[in]		- length of receive buffer
 * @return			- 0 or 1
 * @note			- none
 */
uint8_t I2C_SlaveRecieveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t size)
{
	if(pI2CHandle->TxRxState == I2C_READY && checkI2CBus(pI2CHandle->pI2Cx))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = size;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX_SLAVE;
		I2C_ITCntrl(pI2CHandle->pI2Cx,I2C_TXIE_IT|I2C_ADDRIE_IT|I2C_NACKIE_IT|I2C_STOPIE_IT,ENABLE);
		return SUCCESS;
	}
	return FAIL;
}
/*******************************************************************
 * @fn				- I2C_SlaveSendDataIT
 *
 * @brief			- this function sends data to the I2C bus via an interrupt
 *
 * @parem[in]		- I2C Configuration
 * @parem[in]		- transmit buffer
 * @parem[in]		- length of transmit buffer
 * @return			- 0 or 1
 * @note			- none
 */
uint8_t I2C_SlaveSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t size)
{
	if(pI2CHandle->TxRxState == I2C_READY && checkI2CBus(pI2CHandle->pI2Cx))
	{

		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = size;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX_SLAVE;
		I2C_ITCntrl(pI2CHandle->pI2Cx,I2C_TXIE_IT|I2C_ADDRIE_IT|I2C_NACKIE_IT|I2C_STOPIE_IT,ENABLE);
		return SUCCESS;
	}
	return FAIL;
}
/*******************************************************************
 * @fn				- checkI2CBus
 *
 * @brief			- this function checks if the I2C bus is busy
 *
 * @parem[in]		- base address of of I2C peripheral
 * @return			- 0 or 1
 * @note			- none
 */
static I2CState checkI2CBus(I2C_RegDef_t *pI2Cx)
{
	if(I2C_GetFlagStatus(pI2Cx,I2C_BUSY_FLAG))
	{
		return I2C_BUSY;
	}
	return I2C_READY;
}
