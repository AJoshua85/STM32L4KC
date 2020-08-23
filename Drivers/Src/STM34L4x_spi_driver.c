/*
 * STM34L4x_spi_driver.c
 *
 *  Created on: Jul. 14, 2020
 *      Author: Avinash
 */


#include "STM32L4x_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle);
static uint8_t SPI_GetDataLen(SPI_RegDef_t *pSPIx);

void SPI_PclkCtrl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{

	if(EnOrDi == ENABLE)
	{
		if(pSPIx == SPI)
		{
			SPI1_PCLK_EN();
		}

	}
	else if (EnOrDi == DISABLE)
	{
		if(pSPIx == SPI)
		{
			SPI1_PCLK_DI();
		}

	}
}


void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t temp= 0;


	//Enable SPI peripheral clock*
	SPI_PclkCtrl(pSPIHandle->pSPIx,ENABLE);

	//Configure the device mode*/
	if(pSPIHandle->SPIConfig.SPI_DeviceMode == SPI_DEVICE_MODE_MASTER)
	{
		//Enable SS output enable Note: NSSI controls the value of Slave select pin
		SPI_SSOEConfig(pSPIHandle->pSPIx,ENABLE);
	}

	temp |= pSPIHandle ->SPIConfig.SPI_DeviceMode <<SPI_CR1_MSTR;

	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//BIDIMODE bit cleared
		temp &= ~( 1 << SPI_CR1_BIDIMODE );
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//BIDIMODE bit set
		temp |= ( 1 << SPI_CR1_BIDIMODE );
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_S_RXONLY)
	{
		//BIDIMODE bit cleared
		temp &= ~( 1 << SPI_CR1_BIDIMODE );
		//RXONLY bit set
		temp |= ( 1 << SPI_CR1_RXONLY);
	}

	//Configure the spi serial clock speed (baud rate)
	temp|= pSPIHandle->SPIConfig.SPI_Sclkspeed <<SPI_CR1_BR;

	//Configure CPOL
	temp|= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//Configure CPHA
	temp|= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	//Configure Software slave select management
	if (pSPIHandle->SPIConfig.SPI_SSM== ENABLE)
	{
		temp|= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;
		temp|= ( 1<<SPI_CR1_SSI);//makes NSS signal internally high avoids MODF error
	}

	//Hardware slave select enabled pulse generated automatically
	else if( pSPIHandle->SPIConfig.SPI_SSM == DISABLE && pSPIHandle->SPIConfig.SPI_DeviceMode == SPI_DEVICE_MODE_MASTER )
	{
		pSPIHandle->pSPIx->CR2 &=~(1 << SPI_CR2_NSSP);// Clear bits before setting
		pSPIHandle->pSPIx->CR2 |=(pSPIHandle->SPIConfig.SPI_NSSP << SPI_CR2_NSSP);
	}

	pSPIHandle->pSPIx->CR1 =temp;
	pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_FRXTH);

	/*Configure the Data size */
	pSPIHandle->pSPIx->CR2 &= ~(0xF << SPI_CR2_DFF);// Clear bits before setting
	pSPIHandle->pSPIx->CR2 |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR2_DFF);

}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{

	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


void SPI_DeIit(SPI_RegDef_t *pSPIx)
{

}


void  SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	SPI_SSIConfig(pSPIx,DISABLE);
	while(Len > 0)
	{
		// wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET);

		//Check data length settings
		if(SPI_GetDataLen(pSPIx) == SPI_DFF_16BITS)
		{

			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			//Force the complier to do 8bit write
			*((uint8_t*) &(pSPIx->DR)) = *pTxBuffer;
			pTxBuffer++;
			Len--;
		}
	}
	SPI_SSIConfig(pSPIx,ENABLE);
}

static uint8_t SPI_GetDataLen(SPI_RegDef_t *pSPIx)
{
	return ( ((pSPIx->CR2)>> 8) & (SPI_DFF_MASK) );
}

void  SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		// wait until RXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET);

		//Check data length settings
		if(SPI_GetDataLen(pSPIx) == SPI_DFF_16BITS)
		{
			*((uint16_t*)pRxBuffer) = (uint16_t)pSPIx->DR;
			pRxBuffer++;
			pRxBuffer++;
			Len--;
			Len--;
		}
		else
		{
			*(pRxBuffer) = (uint8_t)(pSPIx->DR);
			pRxBuffer++;
			Len--;
		}
	}
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi== ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else if(EnOrDi == DISABLE)
	{
		//Check to see if SPI is busy
		while(SPI_GetFlagStatus(pSPIx,SPI_BUSY_FLAG));
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi== ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi== ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}


void SPI_IRQITConfig(uint8_t IRQNumber,uint8_t EnOrDi)
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
			*NVIC_ISER3 |= (1 << ( IRQNumber % 64 ));
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
			*NVIC_ICER3 |= (1 << ( IRQNumber % 64 ));
		}
	}

}


void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t readSR, readCR2;

	//check TXE
	readSR = pHandle->pSPIx->SR &(1 << SPI_SR_TXE);
	readCR2 = pHandle->pSPIx->SR &(1<<SPI_CR2_TXEIE);
	if( readSR & readCR2 )
	{
		spi_txe_interrupt_handle(pHandle);
	}

	//check RXNE
	readSR = pHandle->pSPIx->SR &(1 << SPI_SR_RXNE);
	readCR2 =  pHandle->pSPIx->SR &(1<<SPI_CR2_RXNEIE);
	if( readSR & readCR2 )
	{
		spi_rxne_interrupt_handle(pHandle);
	}
	//check overrun flag
	readSR = pHandle->pSPIx->SR &(1 << SPI_SR_OVR);
	readCR2 =  pHandle->pSPIx->SR &(1<< SPI_CR2_ERRIE);
	if( readSR & readCR2 )
	{
			spi_ovr_interrupt_handle(pHandle);
	}

}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer,uint8_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		pSPIHandle->pTxBuffer=pTxBuffer;
		pSPIHandle->TxLen=Len;
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//Enable the TXEIE control bit to get interrupt when TXE flag is set in SR
		pSPIHandle->pSPIx->CR2|= ( 1 << SPI_CR2_TXEIE);

	}


	return state;


}
uint8_t SPI_RecieveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer,uint8_t Len)
{
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX)
	{
		pSPIHandle->pTxBuffer=pRxBuffer;
		pSPIHandle->RxLen=Len;
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//Enable the RXNEIE control bit to get interrupt when RX flag is set in SR
		pSPIHandle->pSPIx->CR2|= ( 1 << SPI_CR2_RXNEIE);

	}
	return state;
}

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//Check data length settings
	if(SPI_GetDataLen(pSPIHandle->pSPIx) == SPI_DFF_16BITS)
	{

		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}
	else
	{
		//Force the complier to do 8bit write
		*((uint8_t*) &(pSPIHandle->pSPIx->DR)) = *(pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}
	//Close the transmission when Tx length is zero
	if(! pSPIHandle->TxLen)
	{
		//Clear the interrupt
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEvenCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}

}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

	//Check data length settings
	if(SPI_GetDataLen(pSPIHandle->pSPIx) == SPI_DFF_16BITS)
	{
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen-=2;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->pRxBuffer++;
	}
	else
	{
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	//Close the reception when Rx length is zero
	if(!pSPIHandle->RxLen)
	{
		//Clear the interrupt
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEvenCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);

	}

}
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//clear the overun flag
	if(pSPIHandle->TxState !=SPI_BUSY_IN_TX)
	{
		SPI_ClearOVRFlag(pSPIHandle->pSPIx);
	}

	SPI_ApplicationEvenCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
	(void)temp;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &=~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer=NULL;
	pSPIHandle->TxLen =0;
	pSPIHandle->TxState= SPI_READY;

}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &=~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer=NULL;
	pSPIHandle->RxLen =0;
	pSPIHandle->RxState= SPI_READY;

}



void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx_register = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8-NO_PR_BITS_IMPLEMENTED);/* This may vary depending on manufacture implementation*/

	*(NVIC_PR_BASE_ADDR + iprx_register) &= ~( 0xF << shift_amount);/*clear bits before setting*/
	*(NVIC_PR_BASE_ADDR + iprx_register) |= ( IRQPriority << shift_amount);

}

__attribute__((weak)) void SPI_ApplicationEvenCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{

}
