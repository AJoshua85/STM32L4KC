/*
 * STM32L4x_i2c_driver.h
 *
 *  Created on: Jul. 18, 2020
 *      Author: Avinash
 */

#ifndef INC_STM32L4X_I2C_DRIVER_H_
#define INC_STM32L4X_I2C_DRIVER_H_

#include "STM32L432xx.h"

/*I2C interrupt events*/
typedef enum
{
	ADDRMATCH,
	RXEREADY,
	STOPBIT,
	NACKF,
	TXISREADY
}IT_EV;

/*I2C Status*/
typedef enum
{
	I2C_BUSY,
	I2C_READY,
	I2C_BUSY_IN_RX_MASTER,
	I2C_BUSY_IN_TX_MASTER,
	I2C_BUSY_IN_RX_SLAVE,
	I2C_BUSY_IN_TX_SLAVE
}I2CState;

/*Configuration Structure for I2Cx peripheral*/
typedef struct
{
	uint8_t	 I2C_PRESC;
	uint8_t	 I2C_SCLDEL;
	uint8_t	 I2C_SDADEL;
	uint8_t	 I2C_SCLH;
	uint8_t	 I2C_SCLL;
	uint8_t	 I2C_DeviceAddr;


}I2C_Config_t;


/*Handle Structure for I2Cx peripheral*/
typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint8_t	TxLen;
	uint8_t RxLen;
	I2CState TxRxState;
	uint8_t DevAddr;
}I2C_Handle_t;

/*@I2C_SCLSpeed*/
#define I2C_SCL_SPEED_SM 100000
#define I2C_SCL_SPEED_FM4K 400000
#define I2C_SCL_SPEED_SM2K 200000

/*@I2C_ACKCntrl*/
#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

/*@I2C_FMDutyCycle*/
#define I2C_FM_DUTY_2
#define I2C_FM_DUTY_16_9

/*************************************************************************
*Bit position definitions of I2C peripheral
**************************************************************************/

/* Bit position definition I2C_CR1*/
#define I2C_CR1_PE			0
#define I2C_CR1_TXIE		1
#define I2C_CR1_RXIE		2
#define I2C_CR1_ADDRIE		3
#define I2C_CR1_NACKIE		4
#define I2C_CR1_STOPIE		5
#define I2C_CR1_TCIE		6
#define I2C_CR1_ERRIE		7
#define I2C_CR1_DNF			8
#define I2C_CR1_ANOFF		12
#define I2C_CR1_TXDMAEN		14
#define I2C_CR1_RXDMANE		15
#define I2C_CR1_SBC			16
#define I2C_CR1_NOSTRETCH	17
#define I2C_CR1_WUPEN		18
#define I2C_CR1_GCEN		19
#define I2C_CR1_SMBHEN		20
#define I2C_CR1_SMBDEN		21
#define I2C_CR1_ALERTEN		22
#define I2C_CR1_PECEN		23

/* Bit position definition I2C_CR2*/
#define I2C_CR2_SADDR		0
#define I2C_CR2_RD_WRN		10
#define I2C_CR2_ADD10		11
#define I2C_CR2_HEAD10R		12
#define I2C_CR2_START		13
#define I2C_CR2_STOP		14
#define I2C_CR2_NACK		15
#define I2C_CR2_NBYTES		23
#define I2C_CR2_RELOAD		24
#define I2C_CR2_AUTOEND		25
#define I2C_CR2_PECBYTE		26


/* Bit position definition I2C_TIMINGR*/
#define I2C_TIMINGR_SCLL	0
#define I2C_TIMINGR_SCLH	8
#define I2C_TIMINGR_SDADEL	16
#define I2C_TIMINGR_SCLDEL	20
#define I2C_TIMINGR_PRESC	28

/* Bit position definition I2C_OAR1*/
#define I2C_OAR1_ADDR_10BIT		0
#define I2C_OAR1_ADDR_7BIT		1
#define I2C_OAR1_OA1MODE		10
#define I2C_OAR1_OA1EN			15

/* Bit position definition I2C_ISR*/
#define I2C_ISR_TXE			0
#define I2C_ISR_TXIS		1
#define I2C_ISR_RXNE		2
#define I2C_ISR_ADDR		3
#define I2C_ISR_NACKF		4
#define I2C_ISR_STOPF		5
#define I2C_ISR_TC			6
#define I2C_ISR_TCR			7
#define I2C_ISR_BERR		8
#define I2C_ISR_ARLO		9
#define I2C_ISR_OVR			10
#define I2C_ISR_PECERR		11
#define I2C_ISR_TIMEOUT		12
#define I2C_ISR_ALERT		13
#define I2C_ISR_BUSY		15
#define I2C_ISR_DIR			16
#define I2C_ISR_ADDCODE		17

/*Bit position definition I2C_ICR*/
#define I2C_ICR_ADDRCF		3
#define I2C_ICR_NACKCF		4
#define I2C_ICR_STOPCF		5


/*I2C interrupt mask*/
#define I2C_TXIE_IT		(1 << I2C_CR1_TXIE)
#define I2C_RXIE_IT		(1 << I2C_CR1_RXIE)
#define I2C_ADDRIE_IT	(1 << I2C_CR1_ADDRIE)
#define I2C_NACKIE_IT	(1 << I2C_CR1_NACKIE)
#define I2C_STOPIE_IT	(1 << I2C_CR1_STOPIE)

/*I2C status flags definitions*/
#define I2C_TXE_FLAG			(1 << I2C_ISR_TXE)
#define I2C_TXIS_FLAG			(1 << I2C_ISR_TXIS)//Interrupt Flag
#define I2C_NACKF_FLAG			(1 << I2C_ISR_NACKF)
#define I2C_BUSY_FLAG			(1 << I2C_ISR_BUSY)
#define I2C_RXNE_FLAG			(1 << I2C_ISR_RXNE)
#define I2C_ADDR_FLAG			(1 << I2C_ISR_ADDR)
#define I2C_RXNE_FLAG			(1 << I2C_ISR_RXNE)
#define I2C_STOPF_FLAG			(1 << I2C_ISR_STOPF)
#define I2C_DIR_FLAG			(1 << I2C_ISR_DIR)

/*I2C  Master Data bus configuration*/
#define I2C_MASTER_WR		0
#define I2C_MASTER_RD		1

/*I2C Slave transfer direction*/
#define SLAVE_RECEIVER		2
#define SLAVE_TRANSMITTER	1

/*I2C Interrupts*/
#define IRQ_NO_I2C1_EV 		31




void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

/*Peripheral Clock setup*/
void I2C_PclkCtrl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

/*Init and De-init*/
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeIit(I2C_RegDef_t *pI2Cx);

void I2C_ITCntrl(I2C_RegDef_t *pI2Cx,uint8_t interrupt ,uint8_t EnOrDi);

uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t size, uint8_t SlaveAddr);
uint8_t  I2C_MasterRecieveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t size, uint8_t SlaveAddr);
uint8_t  I2C_SlaveRecieveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t size);
uint8_t  I2C_SlaveSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t size);


uint8_t I2C_MasterSendData(I2C_RegDef_t *pI2Cx, uint8_t *pTxbuffer, uint8_t size,uint8_t SlaveAddr);
void I2C_MasterRecieveData(I2C_RegDef_t *pI2Cx, uint8_t *pRxbuffer, uint8_t size,uint8_t SlaveAddr);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pSPIx, uint32_t FlagName);

void I2C_IRQITConfig(uint8_t IRQNumber,uint8_t EnOrDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);


void transmitReadyEvent(I2C_Handle_t *pI2CHandle);


void I2CReadStatusFlag(I2C_RegDef_t *pI2Cx);
void I2C_SetsFlag(I2C_RegDef_t *pI2Cx);
uint8_t getI2CFlag(void);
void addressMatchEvent(I2C_RegDef_t *pI2Cx);
void recieverBufferFullEvent(I2C_Handle_t *pI2CHandle);
void stopFlagEvent(I2C_Handle_t *pI2CHandle);
void nackEvent(I2C_Handle_t *pI2CHandle);


uint32_t RCC_GetPCLK1Val(void);

#endif /* INC_STM32L4X_I2C_DRIVER_H_ */
