/*
 * STM32L4x_uart_driver.h
 *
 *  Created on: Aug. 25, 2020
 *      Author: Avinash
 */

#ifndef INC_STM32L4X_USART_DRIVER_H_
#define INC_STM32L4X_USART_DRIVER_H_

/*UART Status*/
typedef enum
{
	USART_BUSY,
	USART_READY,
	USART_MODE_ONLY_TX,
	USART_MODE_ONLY_RX,
	USART_MODE_ONLY_TXRX

}UARTState;

/*USART interrupt events*/
typedef enum
{
	USART_RXEREADY,
	USART_TXISREADY,
	ENDTX

}USART_IT_EV;

/* Configuration structure for USARTx peripheral*/
typedef struct
{
	UARTState USART_Mode;
	uint32_t USART_Baud;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_WordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;
	uint8_t USART_OverSampling;
	uint8_t USART_DataFrame;

}USART_Config_t;

/*Handle structure for USARTx peripheral*/
typedef struct
{
	USART_RegDef_t *pUSARTx;
	USART_Config_t USART_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	UARTState TxBusyState;
	UARTState RxBusyState;
}USART_Handle_t;

/* @USART_Baud Possible options for USART_Baud */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					2400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define USART_STD_BAUD_3M 					3000000

/*@USART_OverSampling Possible options for USART_OverSampling*/
#define USART_OVERSAMPLE_8	  0
#define USART_OVERSAMPLE_16	  1

/* @USART_ParityControl Possible options for USART_ParityControl*/
#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE  0

/* @USART_WordLength Possible options for USART_WordLength*/
#define USART_WORDLEN_7BITS  0
#define USART_WORDLEN_8BITS  1
#define USART_WORDLEN_9BITS  2

/* @USART_NoOfStopBits Possible options for USART_NoOfStopBits */
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

/* @USART_HWFlowControl Possible options for USART_HWFlowControl*/
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

/* @USART_NoOfStopBits Possible options for USART_NoOfStopBits */
#define USART_LSB_FIRST   	0
#define USART_MSB_FIRST   	1
/*************************************************************************
*Bit position definitions of USART peripheral
**************************************************************************/

/* Bit position definition USART_CR1*/
#define USART_CR1_UE			0
#define USART_CR1_UESM			1
#define USART_CR1_RE			2
#define USART_CR1_TE			3
#define USART_CR1_IDLEIE		4
#define USART_CR1_RXNEIE		5
#define USART_CR1_TCIE			6
#define USART_CR1_TXEIE			7
#define USART_CR1_PEIE			8
#define USART_CR1_PS			9
#define USART_CR1_PCE			10
#define USART_CR1_WAKE			11
#define USART_CR1_M0			12
#define USART_CR1_MME			13
#define USART_CR1_CMIE			14
#define USART_CR1_OVER8			15
#define USART_CR1_DEDT			16
#define USART_CR1_DEAT			21
#define USART_CR1_RTOIE			26
#define USART_CR1_EOBIE			27
#define USART_CR1_M1			28

/* Bit position definition USART_CR2*/
#define USART_CR2_ADDM7			4
#define USART_CR2_LBDL			5
#define USART_CR2_LBDIE			6
#define USART_CR2_LBCL			8
#define USART_CR2_CPHA			9
#define USART_CR2_CPOL			10
#define USART_CR2_CLKEN			11
#define USART_CR2_STOP			12
#define USART_CR2_LINEN			14
#define USART_CR2_SWAP			15
#define USART_CR2_RXINV			16
#define USART_CR2_TXINV			17
#define USART_CR2_DATAINV		18
#define USART_CR2_MSBFIRST		19
#define USART_CR2_ABREN			20
#define USART_CR2_ABRMOD		21
#define USART_CR2_RTOEN			23
#define USART_CR2_ADD			24

/* Bit position definition USART_CR3*/
#define USART_CR3_EIE			0
#define USART_CR3_IREN			1
#define USART_CR3_IRLP			2
#define USART_CR3_HDSEL			3
#define USART_CR3_NACK			4
#define USART_CR3_SCEN			5
#define USART_CR3_DMAR			6
#define USART_CR3_DMAT			7
#define USART_CR3_RTSE			8
#define USART_CR3_CTSE			9
#define USART_CR3_CTSIE			10
#define USART_CR3_ONEBIT		11
#define USART_CR3_OVRDIS		12
#define USART_CR3_DDRE			13
#define USART_CR3_DEM			14
#define USART_CR3_DEP			15
#define USART_CR3_SCARCNT		17
#define USART_CR3_WUS			20
#define USART_CR3_WUFIE			22
#define USART_CR3_UCESM			23
#define USART_CR3_TCBGTIE		24

/* Bit position definition USART_GTPR*/
#define USART_GTPR_PSC 			0
#define USART_GTPR_GT			8

/* Bit position definition USART_RTOR*/
#define USART_RTOR_RTO			0
#define USART_RTOR_BLEN			24

/* Bit position definition USART_RQR*/
#define USART_RQR_ABRRQ			0
#define USART_RQR_SBKRQ			1
#define USART_RQR_MMRQ			2
#define USART_RQR_RXFRQ			3
#define USART_RQR_TXFRQ			4

/* Bit position definition USART_ISR*/
#define USART_ISR_PE			0
#define USART_ISR_FE			1
#define USART_ISR_NF			2
#define USART_ISR_ORE			3
#define USART_ISR_IDLE			4
#define USART_ISR_RXNE			5
#define USART_ISR_TC			6
#define USART_ISR_TXE			7
#define USART_ISR_LBDF			8
#define USART_ISR_CTSIF			9
#define USART_ISR_CTS			10
#define USART_ISR_RTOF			11
#define USART_ISR_EOBF			12
#define USART_ISR_ABRE			14
#define USART_ISR_ABRF			15
#define USART_ISR_BUSY			16
#define USART_ISR_CMF			17
#define USART_ISR_SBKF			18
#define USART_ISR_RWU			19
#define USART_ISR_WUF			20
#define USART_ISR_TEACK			21
#define USART_ISR_REACK			22
#define USART_ISR_TCBGT			25

/* Bit position definition USART_ICR*/
#define USART_ICR_PECF			0
#define USART_ICR_FECF			1
#define USART_ICR_NCF			2
#define USART_ICR_ORECF			3
#define USART_ICR_IDLECF		4
#define USART_ICR_TCCF			6
#define USART_ICR_TCBGTCF		7
#define USART_ICR_LBDCF			8
#define USART_ICR_CTSCF			9
#define USART_ICR_RTOCF			11
#define USART_ICR_EOBCF			12
#define USART_ICR_CMCF			17
#define USART_ICR_WUCF			20

/*USART interrupt masks*/
#define USART_RXNEIE_IT			(1 << USART_CR1_RXNEIE)
#define USART_TCIE_IT			(1 << USART_CR1_TCIE)
#define USART_TXEIE_IT			(1 << USART_CR1_TXEIE)
#define USART_PEIE_IT			(1 << USART_CR1_PEIE)

/*USART status flags definitions*/
#define USART_PE_FLAG		(1 << USART_ISR_PE)
#define USART_FE_FLAG		(1 << USART_ISR_FE)
#define USART_NF_FLAG		(1 << USART_ISR_NF)
#define USART_ORE_FLAG		(1 <<USART_ISR_ORE)
#define USART_IDLE_FLAG		(1 <<USART_ISR_IDLE)
#define USART_RXNE_FLAG		(1 <<USART_ISR_RXNE)
#define USART_TC_FLAG		(1 <<USART_ISR_TC)
#define USART_TXE_FLAG		(1 <<USART_ISR_TXE)
#define USART_LBDF_FLAG		(1 <<USART_ISR_LBDF)
#define USART_CTSIF_FLAG	(1 <<USART_ISR_CTSIF)
#define USART_CTS_FLAG		(1 <<USART_ISR_CTS)
#define USART_RTOF_FLAG		(1 <<USART_ISR_RTOF)
#define USART_EOBF_FLAG		(1 <<USART_ISR_EOBF)
#define USART_ABRE_FLAG		(1 <<USART_ISR_ABRE)
#define USART_ABRF_FLAG		(1 <<USART_ISR_ABRF)
#define USART_BUSY_FLAG		(1 <<USART_ISR_BUSY)
#define USART_CMF_FLAG		(1 <<USART_ISR_CMF)
#define USART_SBKF_FLAG		(1 <<USART_ISR_SBKF)
#define USART_RWU_FLAG		(1 <<USART_ISR_RWU)
#define USART_WUF_FLAG		(1 <<USART_ISR_WUF)
#define USART_TEACK_FLAG	(1 <<USART_ISR_TEACK)
#define USART_REACK_FLAG	(1 <<USART_ISR_REACK)
#define USART_TCBGT_FLAG	(1 <<USART_ISR_TCBGT)

#define IRQ_NO_USART1_EV 	37U
#define IRQ_NO_USART2_EV 	38U
/*****************************************************************************
 * 						APIs supported by this driver
 * 		For more information about the APIs check the function definitions
 *****************************************************************************/

/*Peripheral Clock setup */
void USART_PclkCtrl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);

/*Init and De-init*/
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);


/*Interrupt configuration and interrupt flag status*/
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName);
void USART_ReadStatusFlag(USART_RegDef_t *pUSARTx);


void USART_ITCntrl(USART_RegDef_t *pUSARTx,uint8_t interrupt ,uint8_t EnOrDi);
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);
void USART_IRQITConfig(uint8_t IRQNumber,uint8_t EnOrDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

/*Data Send and Receive*/
void USART_SendData(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint8_t len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint8_t len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint8_t size);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint8_t size);
#endif /* INC_STM32L4X_USART_DRIVER_H_ */
