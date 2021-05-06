/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: Apr 8, 2021
 *      Author: prakh
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include "stm32f446xx.h"

/*
 * Configuration structure for SPIx peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;					/* possible values from @SPI_DeviceMode */
	uint8_t SPI_BusConfig;					/* possible values from @SPI_BusConfig */
	uint8_t SPI_SclkSpeed;					/* possible values from @SPI_SclkSpeed */
	uint8_t	SPI_DFF;						/* possible values from @SPI_DFF */
	uint8_t SPI_CPOL;						/* possible values from @SPI_CPOL */
	uint8_t SPI_CPHA;						/* possible values from @SPI_CPHA */
	uint8_t SPI_SSM;						/* possible values from @SPI_SSM */
}SPI_PinConfig_t;


/*
 * Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t *pSPIx;						/* This holds the base address of SPIx(x: 0,1,2) peripheral */
	SPI_PinConfig_t SPI_PinConfig; 				/* This holds the SPIx peripheral configuration settings */
	uint8_t 		*pTxBuffer; 				/* To store the app. Tx buffer address */
	uint8_t 		*pRxBuffer;					/* To store the app. Rx buffer address */
	uint32_t 		TxLen;						/* To store Tx len */
	uint32_t 		RxLen;						/* To store Rx len */
	uint8_t 		TxState;					/* To store Tx state */
	uint8_t 		RxState;					/* To store Rx state */
}SPI_Handle_t;


/*
 * Possible SPI Application Status
 */
#define SPI_READY			0
#define SPI_BUSY_IN_RX		1
#define SPI_BUSY_IN_TX		2

/*
 * Possible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT			1
#define SPI_EVENT_RX_CMPLT			2
#define SPI_EVENT_OVR_ERR			3
#define SPI_EVENT_CRC_ERR			4


/*
 * @SPI_DeviceMode
 * Device Mode
 */
#define SPI_DEVICE_MODE_MASTER    1				/* Master Mode */
#define SPI_DEVICE_MODE_SLAVE     0				/* Slave Mode */


/*
 * @SPI_BusConfig
 * Bus Configuration
 */
#define SPI_BUS_CONFIG_FD                1		/* Full-duplex Config */
#define SPI_BUS_CONFIG_HD                2		/* Half-duplex Config */
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY    3		/* Simlex Config */

/*
 * @SPI_SclkSpeed
 * Clock Speed
 */
#define SPI_SCLK_SPEED_DIV2				0		/* fPCLK/2 */
#define SPI_SCLK_SPEED_DIV4             1		/* fPCLK/4 */
#define SPI_SCLK_SPEED_DIV8             2		/* fPCLK/8 */
#define SPI_SCLK_SPEED_DIV16            3		/* fPCLK/16 */
#define SPI_SCLK_SPEED_DIV32           	4		/* fPCLK/32 */
#define SPI_SCLK_SPEED_DIV64            5		/* fPCLK/64 */
#define SPI_SCLK_SPEED_DIV128           6		/* fPCLK/128 */
#define SPI_SCLK_SPEED_DIV256           7		/* fPCLK/256 */

/*
 * @SPI_DFF
 * Data Frame Format
 */
#define SPI_DFF_8BITS 		0					/* 8-bit data frame format is selected for transmission/reception */
#define SPI_DFF_16BITS  	1					/* 16-bit data frame format is selected for transmission/reception */

/*
 * @CPOL
 * Clock Polarity
 */
#define SPI_CPOL_HIGH 		1					/* CK to 1 when idle */
#define SPI_CPOL_LOW 		0					/* CK to 0 when idle */

/*
 * @CPHA
 * Clock Phase
 */
#define SPI_CPHA_HIGH 		1					/* The second clock transition is the first data capture edge  */
#define SPI_CPHA_LOW 		0					/* The first clock transition is the first data capture edge */

/*
 * @SPI_SSM
 * Software Slave Management
 */
#define SPI_SSM_EN     		1					/* Software slave management enabled */
#define SPI_SSM_DI			0					/* Software slave management disabled */


/*
 * SPI related status flags definitions
 */
#define SPI_TXE_FLAG		(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG		(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG		(1 << SPI_SR_BSY)


/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/

/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and De-Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * Other Peripheral Control APIS
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
