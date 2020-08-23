/*
 * STM32L4x_spi_driver.h
 *
 *  Created on: Jul. 14, 2020
 *      Author: Avinash
 */
#include "STM32L432xx.h"
#include <stddef.h>
#ifndef INC_STM32L4X_SPI_DRIVER_H_
#define INC_STM32L4X_SPI_DRIVER_H_

/*Configuration structure for SPIx peripheral*/
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t	SPI_Sclkspeed;
	uint8_t SPI_DFF;
	uint8_t	SPI_CPOL;
	uint8_t	SPI_CPHA;
	uint8_t SPI_SSM;
	uint8_t SPI_NSSP;/*NSS pulse management only needed when hardware slave select mode is selected*/

}SPI_Config_t;

/*Structure handle for a SPI pin*/
typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint8_t	TxLen;
	uint8_t RxLen;
	uint8_t TxState;
	uint8_t RxState;

}SPI_Handle_t;

/*@SPI_DeviceMode */
#define SPI_DEVICE_MODE_SLAVE		0
#define SPI_DEVICE_MODE_MASTER		1


/*@SPI_BusConfig */
#define SPI_BUS_CONFIG_FD			1
#define SPI_BUS_CONFIG_HD			2
#define SPI_BUS_CONFIG_S_TXONLY		3
#define SPI_BUS_CONFIG_S_RXONLY		4

/*@SPI_SclkSpeed */
#define SPI_SCLK_SPEED_DIV2			0
#define SPI_SCLK_SPEED_DIV4			1
#define SPI_SCLK_SPEED_DIV8			2
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7

/*@SPI_DFF*/
#define SPI_DFF_8BITS				0x7
#define SPI_DFF_16BITS				0xF
#define SPI_DFF_MASK				SPI_DFF_16BITS

/*@CPOL*/
#define SPI_CPOL_LOW				0
#define SPI_CPOL_HIGH				1

/*@CPHA*/
#define SPI_CPHA_LOW				0
#define SPI_CPHA_HIGH				1

/*@SPI_SSM*/
#define SPI_SSM_DI					0
#define SPI_SSM_EN					1

/*@SPI_NSSP*/
#define SPI_NSSP_DI					0
#define SPI_NSSP_EN					1


/*************************************************************************
*Bit position definitions of SPI peripheral
**************************************************************************/

/* Bit position definition SPI_CR1*/

#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define	SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_CRC_LEN			11
#define SPI_CR1_CRCNEXT			12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15

/* Bit position definition SPI_CR2*/
#define SPI_CR2_SSOE			2
#define SPI_CR2_NSSP			3
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7
#define SPI_CR2_DFF				8
#define SPI_CR2_FRXTH			12


/* Bit position definition SPI_SR*/
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8
#define SPI_SR_FRLVL			9
#define SPI_SR_FTVL				11

/*SPI status flags definitions*/
#define SPI_RXNE_FLAG			(1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG			(1 << SPI_SR_TXE )
#define SPI_BUSY_FLAG			(1 << SPI_SR_BSY )
#define SPI_MODF_FLAG			(1 << SPI_SR_MOF )
#define SPI_TEST_FLAG			(1 << SPI_SR_CRCERR )

/*SPI Interrupt states*/
#define SPI_READY				0
#define SPI_BUSY_IN_RX			1
#define SPI_BUSY_IN_TX			2

/*SPI Application events*/
#define SPI_EVENT_TX_CMPLT 		0
#define SPI_EVENT_RX_CMPLT 		1
#define SPI_EVENT_OVR_ERR	 	3
#define SPI_EVENT_CRC_ERR		4

/*Peripheral Clock setup*/
void SPI_PclkCtrl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/*Init and De-init*/
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeIit(SPI_RegDef_t *pSPIx);

/*Data Send and Receive*/
void  SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void  SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);


uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer,uint8_t Len);
uint8_t SPI_RecieveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer,uint8_t Len);



/*Other peripheral control api*/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*IRQ Configuration and ISR handling*/
void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriority(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*Application callback*/
void SPI_ApplicationEvenCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);


#endif /* INC_STM32L4X_SPI_DRIVER_H_ */
