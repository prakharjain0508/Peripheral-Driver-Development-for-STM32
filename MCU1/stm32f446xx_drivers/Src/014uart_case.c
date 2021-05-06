/*
 * 014uart_case.c
 *
 *  Created on: May 5, 2021
 *      Author: prakh
 */


#include <stdio.h>
#include <string.h>
#include "stm32f446xx.h"



//we have 3 different messages that we transmit to arduino
char *msg[3] = {"hihihihihihi123", "Hello How are you ?" , "Today is Monday !"};

//reply from arduino will be stored here
uint8_t rx_buf[1024] ;

USART_Handle_t usart4_handle;


//This flag indicates reception completion
uint8_t rxCmplt = RESET;
uint8_t status=21;

uint8_t g_data = 0;

extern void initialise_monitor_handles();

void USART4_Init(void)
{
	usart4_handle.pUSARTx = UART4;
	usart4_handle.USART_Config.USART_Baud = USART_STD_BAUD_9600;
	usart4_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart4_handle.USART_Config.USART_Mode = USART_MODE_TXRX;
	usart4_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart4_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart4_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&usart4_handle);
}

void USART4_GPIOInit(void)
{
	GPIO_Handle_t usart_gpios;

	usart_gpios.pGPIOx = GPIOA;
	usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_gpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart_gpios.GPIO_PinConfig.GPIO_PinAltFunMode = 8;

	usart_gpios.GPIO_PinConfig.GPIO_PinNumber  = GPIO_PIN_N0_0;
	GPIO_Init(&usart_gpios);

	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_N0_1;
	GPIO_Init(&usart_gpios);

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

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

int main(void)
{
	uint32_t cnt = 0;


	initialise_monitor_handles();

	USART4_GPIOInit();

    USART4_Init();

    USART_IRQInterruptConfig(IRQ_NO_UART4,  ENABLE);

    USART_PeripheralControl(UART4, ENABLE);

    printf("Application is running\n");

    //do forever
    while(1)
    {
		//wait till button is pressed
		while( GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_N0_13) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		// Next message index ; make sure that cnt value doesn't cross 2
		cnt = cnt % 3;

		//First lets enable the reception in interrupt mode
		//this code enables the receive interrupt
		while ( USART_ReceiveDataIT(&usart4_handle, rx_buf, strlen(msg[cnt])) != USART_READY );

		//Send the msg indexed by cnt in blocking mode
    	USART_SendData(&usart4_handle, (uint8_t*)msg[cnt], strlen(msg[cnt]));

    	printf("Transmitted : %s\n", msg[cnt]);


    	//Now lets wait until all the bytes are received from the arduino .
    	//When all the bytes are received rxCmplt will be SET in application callback
    	printf("Status: %d\n", status);
    	while(rxCmplt != SET);

    	//just make sure that last byte should be null otherwise %s fails while printing
    	rx_buf[strlen(msg[cnt])+ 1] = '\0';

    	//Print what we received from the arduino
    	printf("Received : %s\n",rx_buf);

    	//invalidate the flag
    	rxCmplt = RESET;

    	//move on to next message indexed in msg[]
    	cnt ++;
    }


	return 0;
}


void UART4_IRQHandler(void)
{
	status = 4;
	USART_IRQHandling(&usart4_handle);
}

void USART_ApplicationEventCallback( USART_Handle_t *pUSARTHandle, uint8_t ApEv)
{
	status = 2;
   if(ApEv == USART_EVENT_RX_CMPLT)
   {
			rxCmplt = SET;

   }else if (ApEv == USART_EVENT_TX_CMPLT)
   {
	   ;
   }
}
