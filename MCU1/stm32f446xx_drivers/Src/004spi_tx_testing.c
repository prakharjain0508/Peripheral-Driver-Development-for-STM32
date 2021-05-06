/*
 * 004spi_tx_testing.c
 *
 *  Created on: Apr 9, 2021
 *      Author: prakh
 */

/*
 * PB12 --> NSS
 * PB13 --> SCLK
 * PB14 --> MISO
 * PB15 --> MOSI
 * ALT Function mode: 5
 */

#include "stm32f446xx.h"
#include <string.h>

void SPI_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//NSS
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_N0_12;
	//GPIO_Init(&SPIPins);

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_N0_13;
	GPIO_Init(&SPIPins);

	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_N0_14;
	//GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_N0_15;
	GPIO_Init(&SPIPins);

}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPI_PinConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPI_PinConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_PinConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;		//generates SCLK of 8MHz
	SPI2Handle.SPI_PinConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPI_PinConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPI_PinConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPI_PinConfig.SPI_SSM = SPI_SSM_EN;		//Software slave management enable for NSS pin

	SPI_Init(&SPI2Handle);
}

int main(void)
{
	char user_data[] = "Hello World";

	//this function initializes the GPIO pins to behave as SPI2 pins
	SPI_GPIOInits();

	//this function initializes the SPI peripheral parameters
	SPI2_Inits();

	//set the SSI bit in CR1 register of SPI2 peripheral
	SPI_SSIConfig(SPI2, ENABLE);

	//enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	//to send data
	SPI_SendData(SPI2, (uint8_t *)user_data, strlen(user_data));

	//lets confirm if SPI is not busy
	while( SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) );

	//disable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);

	return 0;
}
