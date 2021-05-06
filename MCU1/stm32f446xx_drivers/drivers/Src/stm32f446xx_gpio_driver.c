/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Apr 6, 2021
 *      Author: prakh
 */

#include "stm32f446xx_gpio_driver.h"

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
			GPIOA_PCLK_EN();
		else if (pGPIOx == GPIOB)
			GPIOB_PCLK_EN();
		else if (pGPIOx == GPIOC)
			GPIOC_PCLK_EN();
		else if (pGPIOx == GPIOD)
			GPIOD_PCLK_EN();
		else if (pGPIOx == GPIOE)
			GPIOE_PCLK_EN();
		else if (pGPIOx == GPIOF)
			GPIOF_PCLK_EN();
		else if (pGPIOx == GPIOG)
			GPIOG_PCLK_EN();
		else if (pGPIOx == GPIOH)
			GPIOH_PCLK_EN();
	}else
	{
		if(pGPIOx == GPIOA)
			GPIOA_PCLK_DI();
		else if (pGPIOx == GPIOB)
			GPIOB_PCLK_DI();
		else if (pGPIOx == GPIOC)
			GPIOC_PCLK_DI();
		else if (pGPIOx == GPIOD)
			GPIOD_PCLK_DI();
		else if (pGPIOx == GPIOE)
			GPIOE_PCLK_DI();
		else if (pGPIOx == GPIOF)
			GPIOF_PCLK_DI();
		else if (pGPIOx == GPIOG)
			GPIOG_PCLK_DI();
		else if (pGPIOx == GPIOH)
			GPIOH_PCLK_DI();
	}
}

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function initializes the given GPIO port
 *
 * @param[in]         - handle structure for a GPIO port
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;		//temp register

	//enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1. configure the mode of the gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		//clearing
		pGPIOHandle->pGPIOx->MODER |= temp;		//setting

	}else
	{
		//this part will code later. (interrupt mode)
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. Configure the Falling Trigger Selection Register(FTSR)
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. Configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear the corresponding RTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1. Configure both FTSR and RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= (portcode << (temp2 * 4));

		//3. enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;

	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		//clearing
	pGPIOHandle->pGPIOx->OSPEEDER |= temp;		//setting

	temp = 0;

	//3. configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		//clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;			//setting

	temp = 0;

	//4. configure the optype

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		//clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;		//setting

	temp = 0;

	//5. configure the alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//configure the alt function register
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));		//clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |=  (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 ));

	}

}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - This function de-initializes the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
		GPIOA_REG_RESET();
	else if (pGPIOx == GPIOB)
		GPIOB_REG_RESET();
	else if (pGPIOx == GPIOC)
		GPIOC_REG_RESET();
	else if (pGPIOx == GPIOD)
		GPIOD_REG_RESET();
	else if (pGPIOx == GPIOE)
		GPIOE_REG_RESET();
	else if (pGPIOx == GPIOF)
		GPIOF_REG_RESET();
	else if (pGPIOx == GPIOG)
		GPIOG_REG_RESET();
	else if (pGPIOx == GPIOH)
		GPIOH_REG_RESET();
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - This function reads from the given GPIO input pin
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - pin number of the given GPIO port
 *
 * @return            - 0 or 1
 *
 * @Note              - none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);

	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - This function reads from the given GPIO input port
 *
 * @param[in]         - base address of the GPIO peripheral
 *
 * @return            - 16-bit integer
 *
 * @Note              - none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;

	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - This function writes to the given GPIO pin
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - pin number of the given GPIO port
 * @param[in]		  - GPIO_PIN_SET or GPIO_PIN_RESET macros
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit field corresponding pin number
		pGPIOx->ODR |= (1 << PinNumber);

	}else
	{
		//write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - This function writes to the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]		  - 16-bit value to be written to the port
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - This function toggles the given GPIO pin
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - pin number of the given GPIO port
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*********************************************************************
 * @fn      		  - GPIO_IRQInterruptConfig
 *
 * @brief             - This function configures the given IRQ number of a GPIO pin
 *
 * @param[in]         - IRQ number of the interrupt
 * @param[in]		  - ENABLE or DISABLE macros
 *
 * @return            - none
 *
 * @Note              - Configuration in this API is processor specific (Cortex M4)
 * 						Only 96 IRQ numbers implemented on STM32F446xx MCU
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn      		  - GPIO_IRQPriorityConfig
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
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);


}

/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - This function processes the occurred interrupt of a GPIO pin
 *
 * @param[in]         - pin number of the given GPIO port
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	 //clear the EXTI PR register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber))
	{
		//clear (it is cleared by writing 1 to the PR register)
		EXTI->PR |= (1 << PinNumber);
	}
}
