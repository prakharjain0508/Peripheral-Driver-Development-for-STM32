/*
 * 009i2c_master_rx_testing.c
 *
 *  Created on: May 4, 2021
 *      Author: prakh
 */


#include "stm32f446xx.h"
#include <string.h>
#include <stdio.h>

#define SLAVE_ADDR 0x68

extern void initialise_monitor_handles();

void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

I2C_Handle_t I2C1Handle;

//rcv buffer
uint8_t rcv_buf[32];

/*
 * PB9 --> SDA
 * PB6 --> SCL
 * ALT Function mode: 4
 */

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;



	//SCLK
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_N0_6;
	GPIO_Init(&I2CPins);

	//MOSI
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_N0_9;
	GPIO_Init(&I2CPins);

}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = 0x61;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);

}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GpioButton;

	GpioButton.pGPIOx = GPIOC;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_N0_13;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioButton);
}

int main(void)
{
	uint8_t command_code;
	uint8_t len;

	initialise_monitor_handles();

	printf("Application is running\n");

	GPIO_ButtonInit();

	//i2c peripheral configuration
	I2C1_Inits();

	//i2c pin inits
	I2C1_GPIOInits();

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1Handle.pI2Cx, ENABLE);

	while(1)
	{

		//wait for button press
		while( GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_N0_13) );

		//button de-bounce
		delay();

		//sending command data to ask for data length to be received
		command_code = 0x51;
		I2C_MasterSendData(&I2C1Handle, &command_code, 1, SLAVE_ADDR, I2C_ENABLE_SR);
		//receiving data length
		I2C_MasterReceiveData(&I2C1Handle, &len, 1, SLAVE_ADDR, I2C_ENABLE_SR);

		//sending command data to start receiving data from slave
		command_code = 0x52;
		I2C_MasterSendData(&I2C1Handle, &command_code, 1, SLAVE_ADDR, I2C_ENABLE_SR);
		//receiving data from slave
		I2C_MasterReceiveData(&I2C1Handle, rcv_buf, len, SLAVE_ADDR, I2C_DISABLE_SR);

		rcv_buf[len+1] = '\0';

		printf("Data: %s", rcv_buf);
	}

}
