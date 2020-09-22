/*
 * STM32L4x_uart_driver.c
 *
 *  Created on: Aug. 25, 2020
 *      Author: Avinash
 */
#include <stddef.h>
#include "STM32L432xx.h"
#include "STM32L4x_usart_driver.h"

uint8_t txenum = 0;
uint8_t rxenum = 0;
uint8_t end= 0;
static USART_IT_EV ITFlag;

static UARTState USART_checkRxBus(USART_RegDef_t *pUSARTx);
static void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

/*USART interrupt events*/
static void USART_ReceiveReady(USART_Handle_t *pUSARTHandle);
static void USART_TransmitReady(USART_Handle_t *pUSARTHandle);
static void USART_EndTransmit(USART_Handle_t *pUSARTHandle);

void USART_PclkCtrl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}
	}
	else if (EnOrDi == DISABLE)
	{
		if(pUSARTx == USART1)
		{
			 USART1_PCLK_DI();
		}
	}
}

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if(EnOrDi== ENABLE)
	{
		pUSARTx->CR1 |= (1 <<USART_CR1_UE);
	}
	else if(EnOrDi == DISABLE)
	{

	}

}

/*******************************************************************
 * @fn				- USART_Init
 *
 * @brief			- this function configures USART
 * 					  peripheral settings
 *
 * @parem[in]		- base address of the USART peripheral
 * @return			- none
 * @note			- none
 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	uint32_t tempreg= pUSARTHandle->pUSARTx->CR1;
	USART_PclkCtrl(pUSARTHandle->pUSARTx,ENABLE);

	/*****************Configure CR1 Register************************/
	//Configure USART Tx and Rx
	if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		tempreg &= ~(1<<USART_CR1_TE);
		tempreg |= (1<<USART_CR1_RE);
	}
	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		tempreg &= ~(1<<USART_CR1_RE);
		tempreg |= (1 << USART_CR1_TE);
	}
	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TXRX)
	{
		tempreg |= (( 1 << USART_CR1_RE) | ( 1 << USART_CR1_TE));
	}

	//Configure data length
	if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_7BITS)
	{
		tempreg &= ~ ((1 << USART_CR1_M0) | (1 << USART_CR1_M1));
		tempreg |= (1 << USART_CR1_M1);
	}
	else if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_8BITS)
	{
		tempreg &= ~ ((1 << USART_CR1_M0) | (1 << USART_CR1_M1));
	}
	else if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
	{
		tempreg &= ~ ((1 << USART_CR1_M0) | (1 << USART_CR1_M1));
		tempreg |= (1 << USART_CR1_M0);
	}

	//Configure parity
	if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
	{
		tempreg &= ~(1 <<  USART_CR1_PCE);
	}
	else if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		//Enable parity selection
		tempreg |= (1 <<  USART_CR1_PCE);
		tempreg &= ~(1 << USART_CR1_PS);
	}
	else if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD)
	{
		//Enable parity selection
		tempreg |= (1 <<  USART_CR1_PCE);
		tempreg |= (1 << USART_CR1_PS);
	}

	//Configure oversampling
	if(pUSARTHandle->USART_Config.USART_OverSampling == USART_OVERSAMPLE_8)
	{
		tempreg |= (1 << USART_CR1_OVER8);
	}
	else if(pUSARTHandle->USART_Config.USART_OverSampling == USART_OVERSAMPLE_16)
	{
		tempreg &= ~(1 << USART_CR1_OVER8);
	}

	pUSARTHandle->pUSARTx->CR1 = tempreg;

	/*****************Configure CR2 Register************************/
	tempreg= pUSARTHandle->pUSARTx->CR2;

	//Configure stop bits
	if(pUSARTHandle->USART_Config.USART_NoOfStopBits == USART_STOPBITS_1)
	{
		tempreg &= ~(0x3U << USART_CR2_STOP);
	}
	else if(pUSARTHandle->USART_Config.USART_NoOfStopBits == USART_STOPBITS_0_5 )
	{
		tempreg &= ~(0x3U << USART_CR2_STOP);
		tempreg |= (1 << USART_CR2_STOP);
	}

	else if(pUSARTHandle->USART_Config.USART_NoOfStopBits == USART_STOPBITS_2)
	{
		tempreg &= ~(0x3U << USART_CR2_STOP);
		tempreg |= (0x2U << USART_CR2_STOP);
	}

	else if(pUSARTHandle->USART_Config.USART_NoOfStopBits == USART_STOPBITS_1_5)
	{
		tempreg |= (0x3U << USART_CR2_STOP);
	}

	//Configure data frame format
	if(pUSARTHandle->USART_Config.USART_DataFrame == USART_MSB_FIRST)
	{
		tempreg |= (1 << USART_CR2_MSBFIRST);
	}
	else if(pUSARTHandle->USART_Config.USART_DataFrame == USART_LSB_FIRST)
	{
		tempreg &= ~(1 << USART_CR2_MSBFIRST);
	}
	pUSARTHandle->pUSARTx->CR2 = tempreg;

	/*****************Configure CR3 Register************************/
	tempreg= pUSARTHandle->pUSARTx->CR3;

	if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_NONE )
	{
		tempreg &= ~(1 << USART_CR3_CTSE);
		tempreg &= ~(1 << USART_CR3_RTSE);
	}
	else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		tempreg |= (1 << USART_CR3_CTSE);
	}
	else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		tempreg |= (1 << USART_CR3_RTSE);
	}
	else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		tempreg |= (1 << USART_CR3_RTSE);
		tempreg |= (1 << USART_CR3_CTSE);
	}
	pUSARTHandle->pUSARTx->CR3 = tempreg;

	//configure baud rate
	USART_SetBaudRate(USART1,pUSARTHandle->USART_Config.USART_Baud);
	pUSARTHandle->RxBusyState = USART_READY;
	pUSARTHandle->TxBusyState = USART_READY;

}
/*******************************************************************
 * @fn				- USART_IRQITConfig
 *
 * @brief			- this function enables or disables interrupt
 * 					  in the NVIC
 *
 * @parem[in]		- IRQ number
 * @parem[in]		- enable or disable
 * @return			- none
 * @note			- none
 */
void USART_IRQITConfig(uint8_t IRQNumber,uint8_t EnOrDi)
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
 * @fn				- USART_IRQPriorityConfig
 *
 * @brief			- this function configure the priority of
 * 					  the USART interrupt
 *
 * @parem[in]		- USART IRQ number
 * @parem[in]		- IRQ priority to be set
 * @return			- none
 * @note			- none
 */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx_register = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8-NO_PR_BITS_IMPLEMENTED);// This may vary depending on manufacture implementation

	*(NVIC_PR_BASE_ADDR + (iprx_register *4)) &= ~( 0xF << shift_amount);//clear bits before setting
	*(NVIC_PR_BASE_ADDR + (iprx_register *4)) |= ( IRQPriority << shift_amount);
}
/*******************************************************************
 * @fn				- USART_ITCntrl
 *
 * @brief			- this function enables  USART peripheral interrupts
 *
 * @parem[in]		- base address of the USART peripheral
 * @parem[in]		- interrupt mask for a given interrupt
 * @parem[in]		- enable or disable
 * @return			- none
 * @note			- This function can enable/disable multiple interrupts
 * 					  by combining the masks together
 */
void USART_ITCntrl(USART_RegDef_t *pUSARTx,uint8_t interrupt ,uint8_t EnOrDi)
{

	if(pUSARTx == USART1)
	{
		if(interrupt & USART_RXNEIE_IT && EnOrDi== ENABLE)
		{
			pUSARTx->CR1 |= USART_RXNEIE_IT;
		}
		else if(interrupt & USART_RXNEIE_IT && EnOrDi== DISABLE)
		{
			pUSARTx->CR1 &= ~(USART_RXNEIE_IT);
		}
		if(interrupt & USART_TCIE_IT && EnOrDi== ENABLE)
		{
			pUSARTx->CR1 |= USART_TCIE_IT;
		}
		else if(interrupt & USART_TCIE_IT && EnOrDi== DISABLE)
		{
			pUSARTx->CR1 &= ~(USART_TCIE_IT);
		}
		if(interrupt & USART_TXEIE_IT && EnOrDi== ENABLE)
		{
			pUSARTx->CR1 |= USART_TXEIE_IT;
		}
		else if(interrupt & USART_TXEIE_IT && EnOrDi== DISABLE)
		{
			pUSARTx->CR1 &= ~(USART_TXEIE_IT);
		}
		if(interrupt & USART_PEIE_IT && EnOrDi== ENABLE)
		{
			pUSARTx->CR1 |= USART_PEIE_IT;
		}
		else if(interrupt & USART_PEIE_IT&& EnOrDi== DISABLE)
		{
			pUSARTx->CR1 &= ~(USART_PEIE_IT);
		}
	}
}
/*******************************************************************
 * @fn				- USART_GetFlagStatus
 *
 * @brief			- reads the ISR register to determine
 * 					  which interrupt has occurred
 *
 * @parem[in]		- base address of the USART peripheral
 * @parem[in]		- Interrupt to check
 * @return			- FLAG_SET or FLAG_RESET
 * @note			- none
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName)
{
	if(pUSARTx->ISR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}
/*******************************************************************
 * @fn				- USART_ReadStatusFlag
 *
 * @brief			- checks to see which interrupt has occurred
 * 					  and sets the ITFlag
 *
 * @parem[in]		- base address of USART peripheral
 * @return			- none
 * @note			- none
 */
void USART_ReadStatusFlag(USART_RegDef_t *pUSARTx)
{
	//Check to see if the interrupt is enabled and flag is set
	if( USART_GetFlagStatus(pUSARTx,USART_RXNE_FLAG) && (pUSARTx->CR1 & USART_RXNEIE_IT))
	{
		ITFlag = USART_RXEREADY;
	}
	else if( USART_GetFlagStatus(pUSARTx, USART_TXE_FLAG) && (pUSARTx->CR1 & USART_TXEIE_IT))
	{
		ITFlag = USART_TXISREADY;
	}
	else if( USART_GetFlagStatus(pUSARTx, USART_TC_FLAG) && (pUSARTx->CR1 & USART_TCIE_IT))
	{
		ITFlag = ENDTX;
	}
}
/*******************************************************************
 * @fn				- USART_SetBaudRate
 *
 * @brief			- this function configures the baud rate
 *
 * @parem[in]		- base address of the USART peripheral
 * 					- baud rate
 * @return			- none
 * @note			- none
 */
static void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	uint64_t temp;
	uint32_t usartdiv;

	//Oversampling by 8
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		//Round up the value
		temp = (((20 *CLK_SPEED) /BaudRate)+ 5)/10;
		temp = (uint32_t)temp;
		usartdiv = (temp & 0xFFF0);
		temp = (temp >> 1) & ~(0xFFF8);
		usartdiv = usartdiv|temp;
	}
	//Oversampling by 16
	else
	{
		//Round up the value
		temp = ((10*CLK_SPEED/BaudRate)+5)/10;
		usartdiv =(uint32_t)temp;
	}
	pUSARTx->BRR= usartdiv;
}

/*******************************************************************
 * @fn				- USART_TransmitReady
 *
 * @brief			- this function acknowledges TXE interrupt
 *
 * @parem[in]		- USART Configuration handle
 * @return			- none
 * @note			- none
 */
static void USART_TransmitReady(USART_Handle_t *pUSARTHandle)
{
	txenum++;
	uint16_t *pdata;
	if(pUSARTHandle->TxBusyState == USART_BUSY)
	{
		if(pUSARTHandle->TxLen > 0)
		{
			//9Bit data frame format
			if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
			{
				//Load the first two bytes from the buffer masking off rest of bits other than the first 9bits
				pdata = (uint16_t*)pUSARTHandle->pTxBuffer;
				pUSARTHandle->pUSARTx->TDR = (*pdata & (uint16_t)0x01FF);

				if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
				{
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen-=2;
				}
				else
				{
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen--;
				}
			}
			//8Bit data frame format
			else if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_8BITS)
			{
					pUSARTHandle->pUSARTx->TDR = *(pUSARTHandle->pTxBuffer);
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen--;

			}
			//7Bit data frame format
			else
			{
				pUSARTHandle->pUSARTx->TDR = (*(pUSARTHandle->pTxBuffer) & (uint8_t)0x7F);
				pUSARTHandle->pTxBuffer++;
				pUSARTHandle->TxLen--;
			}
		}
		//Transmit completed disable TXE interrupt enable Transmit complete interrupt
		else
		{
			USART_ITCntrl(pUSARTHandle->pUSARTx,USART_TCIE_IT,ENABLE);
			USART_ITCntrl(pUSARTHandle->pUSARTx,USART_TXEIE_IT,DISABLE);
		}
	}
}
/*******************************************************************
 * @fn				- USART_ReceiveReady
 *
 * @brief			- this function acknowledges RXNE interrupt
 *
 * @parem[in]		- USART Configuration handle
 * @return			- none
 * @note			- none
 */
static void USART_ReceiveReady(USART_Handle_t *pUSARTHandle)
{
	if(pUSARTHandle->RxBusyState == USART_BUSY)
	{
		if(pUSARTHandle->RxLen > 0)
		{
			rxenum++;
			//9Bit data frame format
			if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
			{
				if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
				{
					//Parity not used 9bit data frame
					*((uint16_t*)pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->RDR & (uint16_t)0x01FF);
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->RxLen-=2;
				}
				else
				{
					//8 Bit data frame ignoring parity bit
					*(pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->RDR & (uint8_t)0xFF);
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->RxLen--;
				}
			}
			//8Bit data frame format
			else if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_8BITS)
			{
				if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
				{
					*(pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->RDR & (uint8_t)0xFF);
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->RxLen--;
				}
				else
				{
					//7bit data ignoring parity bit
					*(pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->RDR & (uint8_t)0x7F);
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->RxLen--;
				}
			}
			//7bit data frame format
			else
			{
				*(pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->RDR & (uint8_t)0x7F);
				pUSARTHandle->pRxBuffer++;
				pUSARTHandle->RxLen--;
			}
			//Transfer finished
			if(pUSARTHandle->RxLen == 0)
			{
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ITCntrl(pUSARTHandle->pUSARTx,USART_RXNEIE_IT,DISABLE);
			}
		}
		else
		{
			//length mismatch discard
			pUSARTHandle->pUSARTx->RQR |= (1 << USART_RQR_RXFRQ);
		}
	}
}
/*******************************************************************
 * @fn				- USART_EndTransmit
 *
 * @brief			- this function acknowledges TC interrupt
 * 					  and disables TXE interrupt
 *
 * @parem[in]		- USART Configuration handle
 * @return			- none
 * @note			- none
 */
static void USART_EndTransmit(USART_Handle_t *pUSARTHandle)
{
	end++;
	USART_ITCntrl(pUSARTHandle->pUSARTx,USART_TCIE_IT|USART_TXEIE_IT,DISABLE);
	pUSARTHandle->pUSARTx->ICR |= (1 << USART_ICR_TCCF);
	pUSARTHandle->pTxBuffer = NULL;
	pUSARTHandle->TxBusyState = USART_READY;

}
/*******************************************************************
 * @fn				- USART_SendData
 *
 * @brief			- this function transfers data
 * 					  through polling method
 *
 * @parem[in]		- USART Configuration handle
 * 					- Transmit buffer
 * 					- length of the transmit buffer
 * @return			- none
 * @note			- none
 */
void USART_SendData(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint8_t len)
{
	uint8_t i;
	uint16_t *pdata;

	for(i=0;i <len;i++)
	{
		while(!(pUSARTHandle-> pUSARTx->ISR & USART_TXE_FLAG));

		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			pdata = (uint16_t*)pTxBuffer;
			pUSARTHandle->pUSARTx->TDR = (*pdata & (uint16_t)0x01FF);

			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				pTxBuffer++;
				pTxBuffer++;
				i++;
			}
			else
			{
				pTxBuffer++;
			}
		}
		else if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_8BITS)
		{
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//8bit data
				pUSARTHandle->pUSARTx->TDR = *(pTxBuffer);
				pTxBuffer++;
			}
		}
		else
		{
			//7bit data
			pUSARTHandle->pUSARTx->TDR = (*(pTxBuffer) & (uint8_t)0x7F);
			pTxBuffer++;
		}
	}
	while(!(pUSARTHandle->pUSARTx->ISR & USART_TC_FLAG));
}
/*******************************************************************
 * @fn				- USART_ReceiveData
 *
 * @brief			- this function receives data
 * 					  through polling method
 *
 * @parem[in]		- USART Configuration handle
 * 					- Transmit buffer
 * 					- length of the receiver buffer
 * @return			- none
 * @note			- none
 */
void USART_ReceiveData(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint8_t len)
{
	uint8_t i;
	for(i=0;i <len;i++)
	{
		while(!(pUSARTHandle->pUSARTx->ISR & USART_RXNE_FLAG));
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//9bit data value with no parity
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				*((uint16_t*)pRxBuffer)= (pUSARTHandle->pUSARTx->RDR &(uint16_t)0x01FF);
				pRxBuffer++;
				pRxBuffer++;
				i++;
			}
			//8bit data value ignoring parity
			else
			{
				*(pRxBuffer)= (pUSARTHandle->pUSARTx->RDR & (uint8_t)0xFF);
				pRxBuffer++;
			}
		}
		else if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_8BITS)
		{
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				*(pRxBuffer)= (pUSARTHandle->pUSARTx->RDR & (uint8_t)0xFF);
				pRxBuffer++;
			}
			else
			{
				//7bit data ignoring parity bit
				*(pRxBuffer)= (pUSARTHandle->pUSARTx->RDR & (uint8_t)0x7F);
				pRxBuffer++;
			}
		}
		else
		{
			//7bit data
			*(pRxBuffer)= (pUSARTHandle->pUSARTx->RDR & (uint8_t)0x7F);
			pRxBuffer++;
		}
	}
}
/**************************************************************************************
 * @fn				- USART_SendDataIT
 *
 * @brief			- this function transfers data through interrupt
 *
 * @parem[in]		- USART Configuration handle
 * 					- Transmit buffer
 * 					- length of the transmit buffer
 * @return			- SUCCESS or FAIL
 * @note			- none
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint8_t size)
{
	if(pUSARTHandle->TxBusyState == USART_READY)
	{
		pUSARTHandle->TxLen = size;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY;
		USART_ITCntrl(pUSARTHandle->pUSARTx,USART_TXEIE_IT,ENABLE);
		return SUCCESS;
	}
	return FAIL;
}
/**************************************************************************************
 * @fn				- USART_ReceiveDataIT
 *
 * @brief			- this function receives data through interrupt
 *
 * @parem[in]		- USART Configuration handle
 * 					- receive buffer
 * 					- length of the receive buffer
 * @return			- SUCCESS or FAIL
 * @note			- none
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint8_t size)
{
	if(pUSARTHandle->RxBusyState == USART_READY && USART_checkRxBus(pUSARTHandle->pUSARTx))
	{
		pUSARTHandle->RxLen = size;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY;
		USART_ITCntrl(pUSARTHandle->pUSARTx,USART_RXNEIE_IT,ENABLE);
		return SUCCESS;
	}
	return FAIL;
}
/*******************************************************************
 * @fn				- USART_checkRxBus
 *
 * @brief			- this function checks USART rx bus is free
 *
 * @parem[in]		- base address of of USART peripheral
 * @return			- none
 * @note			- none
 */
static UARTState USART_checkRxBus(USART_RegDef_t *pUSARTx)
{
	if(USART_GetFlagStatus(pUSARTx,USART_BUSY_FLAG))
	{
		return USART_BUSY;
	}
	return USART_READY;
}
/*******************************************************************
 * @fn				- USART_IRQHandling
 *
 * @brief			- this function acknowledges the interrupts
 *
 * @parem[in]		- USART Configuration handle
 * @return			- none
 * @note			- none
 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
	USART_ReadStatusFlag(pUSARTHandle->pUSARTx);
	if (ITFlag == ENDTX)
	{
		USART_EndTransmit(pUSARTHandle);
	}
	else if(ITFlag == USART_TXISREADY)
	{
		USART_TransmitReady(pUSARTHandle);
	}
	else if(ITFlag == USART_RXEREADY)
	{
		USART_ReceiveReady(pUSARTHandle);
	}
}

