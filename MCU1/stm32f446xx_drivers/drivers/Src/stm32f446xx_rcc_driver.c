/*
 * stm32f446xx_rcc_driver.c
 *
 *  Created on: May 5, 2021
 *      Author: prakh
 */

#include "stm32f446xx_rcc_driver.h"

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB1_PreScaler[4] = {2, 4 , 8, 16};

/*********************************************************************
 * @fn      		  - RCC_GetPCLK1Value
 *
 * @brief             - This function returns the PCLK1 value
 *
 * @param[in]         - none
 *
 * @return            - PCLK1 value
 *
 * @Note              - none
 */
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk;

	uint8_t clksrc, temp, ahbp, apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0)
	{
		SystemClk = 16000000; 			//System clock is HSI
	}else if(clksrc == 1)
	{
		SystemClk = 8000000;			//System clock is HSE
	}else if(clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}

	//for ahb
	temp = ((RCC->CFGR >> 4) & 0xF);

	if(temp < 8)
	{
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	//for apb1
	temp = ((RCC->CFGR >> 10) & 0x7);

	if(temp < 4)
	{
		apb1p = 1;
	}else
	{
		apb1p = APB1_PreScaler[temp-4];
	}

	pclk1 = (SystemClk / ahbp) / apb1p;

	return pclk1;
}

/*********************************************************************
 * @fn      		  - RCC_GetPCLK2Value
 *
 * @brief             - This function returns the PCLK2 value
 *
 * @param[in]         - none
 *
 * @return            - PCLK2 value
 *
 * @Note              - none
 */
uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t pclk2, temp, SystemClk = 0;

	uint8_t clksrc, ahbp, apb2p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0)
	{
		SystemClk = 16000000; 			//System clock is HSI
	}else if(clksrc == 1)
	{
		SystemClk = 8000000;			//System clock is HSE
	}else if(clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}

	//for ahb
	temp = ((RCC->CFGR >> 4) & 0xF);

	if(temp < 8)
	{
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	//for apb2
	temp = ((RCC->CFGR >> 13) & 0x7);

	if(temp < 0x04)
	{
		apb2p = 1;
	}else
	{
		apb2p = APB1_PreScaler[temp-4];
	}

	pclk2 = (SystemClk / ahbp) / apb2p;

	return pclk2;
}

/*********************************************************************
 * @fn      		  - RCC_GetPLLOutputClock
 *
 * @brief             - This function returns the PLL clock
 *
 * @param[in]         - none
 *
 * @return            - PLL clock value
 *
 * @Note              - none
 */
uint32_t RCC_GetPLLOutputClock()
{
	uint32_t temp = 0;

	//Function not implemented yet!!

	return temp;
}
