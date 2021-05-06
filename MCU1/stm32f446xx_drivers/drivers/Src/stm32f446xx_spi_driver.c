/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Apr 8, 2021
 *      Author: prakh
 */

#include "stm32f446xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - This function enables or disables SPIx peripheral clock
 *
 * @param[in]         - base address of SPIx peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
			SPI1_PCLK_EN();
		else if (pSPIx == SPI2)
			SPI2_PCLK_EN();
		else if (pSPIx == SPI3)
			SPI3_PCLK_EN();
		else if (pSPIx == SPI4)
			SPI4_PCLK_EN();

	}else
	{
		if(pSPIx == SPI1)
			SPI1_PCLK_DI();
		else if (pSPIx == SPI2)
			SPI2_PCLK_DI();
		else if (pSPIx == SPI3)
			SPI3_PCLK_DI();
		else if (pSPIx == SPI4)
			SPI4_PCLK_DI();

	}
}

/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             - This function initializes the given SPIx peripheral
 *
 * @param[in]         - handle structure for SPIx peripheral
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t tempreg = 0;

	//enable the peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//1. Configure the device mode
	tempreg |= pSPIHandle->SPI_PinConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. Configure the bus config
	if(pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}else if (pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempreg |= ~(1 << SPI_CR1_RXONLY);
	}

	//3. Configure the SPI serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPI_PinConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. Configure the DFF
	tempreg |= pSPIHandle->SPI_PinConfig.SPI_DFF << SPI_CR1_DFF;

	//5. Configure the CPOL
	tempreg |= pSPIHandle->SPI_PinConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. Configure the CPHA
	tempreg |= pSPIHandle->SPI_PinConfig.SPI_CPHA << SPI_CR1_CPHA;

	//7. configure the SSM
	tempreg |= pSPIHandle->SPI_PinConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;		//assign the temp register to CR1 register
}

/*********************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             - This function de-initializes the given SPIx peripheral
 *
 * @param[in]         - base address of SPIx peripheral
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
		SPI1_REG_RESET();
	else if (pSPIx == SPI2)
		SPI2_REG_RESET();
	else if (pSPIx == SPI3)
		SPI3_REG_RESET();
	else if (pSPIx == SPI4)
		SPI4_REG_RESET();

}

/*********************************************************************
 * @fn      		  - SPI_GetFlagStatus
 *
 * @brief             - Gets the status of the flag from the status register
 *
 * @param[in]         - base address of SPIx peripheral
 * @param[in]         - name of the flag
 *
 * @return            - 1 or 0
 *
 * @Note              - none
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             - Send data from the given SPIx peripheral
 *
 * @param[in]         - base address of SPIx peripheral
 * @param[in]         - pointer to transfer buffer
 * @param[in]         - size of data transfer (in bytes)
 *
 * @return            - none
 *
 * @Note              - This is a blocking call or polling based API
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until the TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//2. check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//16 bit DFF
			//1. Load the data in to the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else
		{
			//8 bit DFF
			//1. Load the data in to the DR
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}

	}
}

/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             - Receive data from the given SPIx peripheral
 *
 * @param[in]         - base address of SPIx peripheral
 * @param[in]         - pointer to receive buffer
 * @param[in]         - size of data to be received (in bytes)
 *
 * @return            - none
 *
 * @Note              - This is a blocking call or polling based API
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until the RXNE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == (uint8_t)FLAG_RESET);

		//2. check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//16 bit DFF
			//1. Load the data from DR to Rxbuffer address
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}else
		{
			//8 bit DFF
			//1. Load the data from DR to Rxbuffer address
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}

	}
}

/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
 *
 * @brief             - This function is used to enable or disable the SPIx peripheral
 *
 * @param[in]         - base address of SPIx peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/*********************************************************************
 * @fn      		  - SPI_SSIConfig
 *
 * @brief             - This function is used to SET or RESET the SSI bit of the SPIx peripheral
 *
 * @param[in]         - base address of SPIx peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - none
 *
 * @Note              - Used when SPIx is configured in SSM to avoid the Master Mode Fault (MODF)
 * 						If ENABLED, this makes the NSS signal high and avoid MODF error
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/*********************************************************************
 * @fn      		  - SPI_SSOEConfig
 *
 * @brief             - This function is used to SET or RESET the SSOE bit of the SPIx peripheral
 *
 * @param[in]         - base address of SPIx peripheral
 * @param[in]         -	ENABLE or DISABLE macros
 *
 * @return            - none
 *
 * @Note              -	none
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |=  (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &=  ~(1 << SPI_CR2_SSOE);
	}
}

/*********************************************************************
 * @fn      		  - SPI_IRQInterruptConfig
 *
 * @brief             - This function configures the given IRQ number of the SPIx peripheral
 *
 * @param[in]         - IRQ number of the interrupt
 * @param[in]		  - ENABLE or DISABLE macros
 *
 * @return            - none
 *
 * @Note              - Configuration in this API is processor specific (Cortex M4)
 * 						Only 96 IRQ numbers implemented on STM32F446xx MCU
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber < 32)
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if(IRQNumber >= 32 && IRQNumber < 64)
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));

		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));

		}else if(IRQNumber >= 96 && IRQNumber < 128)
		{
			//program ISER3 register
			*NVIC_ISER3 |= (1 << (IRQNumber % 96));
		}
	}else
	{
		if(IRQNumber < 32)
		{
			//program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);

		}else if(IRQNumber >= 32 && IRQNumber < 64)
		{
			//program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));

		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));

		}else if(IRQNumber >= 96 && IRQNumber < 128)
		{
			//program ICER3 register
			*NVIC_ICER3 |= (1 << (IRQNumber % 96));
		}
	}

}

/*********************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
 *
 * @brief             - This function processes the occurred interrupt's priority
 *
 * @param[in]         - IRQ number of the interrupt
 * @param[in]         - priority of the given IRQ number
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/*********************************************************************
 * @fn      		  - SPI_SendDataIT
 *
 * @brief             - Send data from the given SPIx peripheral using SPIx Interrupt
 *
 * @param[in]         - handle structure for SPIx peripheral
 * @param[in]         - pointer to transfer buffer
 * @param[in]         - size of data transfer (in bytes)
 *
 * @return            - state
 *
 * @Note              - This is a non-blocking call or an Interrupt based API
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		//1. Save the Tx buffer address and Len information in some global variable
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->RxLen = Len;

		//2. Mark the SPI state as busy in transmission so that
		//	 no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to getinterrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		//4. Data transmission will be handled by the ISR code
	}

	return state;
}

/*********************************************************************
 * @fn      		  - SPI_ReceiveDataIT
 *
 * @brief             - Receive data from the given SPIx peripheral
 *
 * @param[in]         - handle structure for SPIx peripheral
 * @param[in]         - pointer to receive buffer
 * @param[in]         - size of data to be received (in bytes)
 *
 * @return            - state
 *
 * @Note              - This is a non-blocking call or an Interrupt based API
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		//1. Save the Rx buffer address and Len information in some global variable
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		//2. Mark the SPI state as busy in reception so that
		//	 no other code can take over same SPI peripheral until reception is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the RXNEIE control bit to getinterrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		//4. Data reception will be handled by the ISR code
	}

	return state;
}

/*********************************************************************
 * @fn      		  - SPI_IRQHandling
 *
 * @brief             - This function processes the occurred interrupt on the given SPIx peripheral
 *
 * @param[in]         - handle structure for SPIx peripheral
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;

	//check for TXE flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);
	if(temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	//check for RXNE flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
	if(temp1 && temp2)
	{
		//handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	//check for overrun flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
	if(temp1 && temp2)
	{
		//handle RXNE
		spi_ovr_err_interrupt_handle(pHandle);
	}

}

/*
 * some helper function implementations
 */

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//check the DFF bit in CR1
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		//16 bit DFF
		//1. Load the data in to the DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else
	{
		//8 bit DFF
		//1. Load the data in to the DR
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(! pSPIHandle->TxLen)
	{
		//TxLen is zero, so close the spi transmission and inform the application that
		//Tx is over.

		//this prevents interrupts from setting up of TXE flag
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);	//to indicate Tx complete
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//check the DFF bit in CR1
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		//16 bit DFF
		//1. Load the data in to the DR
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		(uint16_t*)pSPIHandle->pRxBuffer++;
		(uint16_t*)pSPIHandle->pRxBuffer++;
	}else
	{
		//8 bit DFF
		//1. Load the data in to the DR
		*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(! pSPIHandle->RxLen)
	{
		//RxLen is zero, so close the spi reception and inform the application that
		//Rx is over.

		//this prevents interrupts from setting up of RXNE flag
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);	//to indicate Rx complete
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1. clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	//This is a weak implementation. The application may override this function.
}
