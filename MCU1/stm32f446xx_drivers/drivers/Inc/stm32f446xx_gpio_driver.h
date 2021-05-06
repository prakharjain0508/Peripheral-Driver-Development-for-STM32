/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Apr 6, 2021
 *      Author: prakh
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"

/*
 * Configuration structure for a GPIO pin
 */
typedef struct
{
	uint8_t GPIO_PinNumber;					/* possible values from @GPIO_PIN_NUMBERS */
	uint8_t GPIO_PinMode;					/* possible values from @GPIO_PIN_MODES */
	uint8_t GPIO_PinSpeed;					/* possible values from @GPIO_PIN_SPEEDS */
	uint8_t	GPIO_PinPuPdControl;			/* possible values from @GPIO_PIN_PUPD */
	uint8_t GPIO_PinOPType;					/* possible values from @GPIO_PIN_OUT_TYPES */
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;


/*
 * Handle structure for a GPIO pin
 */
typedef struct
{
	GPIO_RegDef_t *pGPIOx;					/* This holds the base address of the GPIO port to which the pin belongs */
	GPIO_PinConfig_t GPIO_PinConfig; 		/* This holds the GPIO pin configuration settings */
}GPIO_Handle_t;


/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_N0_0			0			/* GPIO pin number 0 */
#define GPIO_PIN_N0_1			1			/* GPIO pin number 1 */
#define GPIO_PIN_N0_2			2			/* GPIO pin number 2 */
#define GPIO_PIN_N0_3			3			/* GPIO pin number 3 */
#define GPIO_PIN_N0_4			4			/* GPIO pin number 4 */
#define GPIO_PIN_N0_5			5			/* GPIO pin number 5 */
#define GPIO_PIN_N0_6			6			/* GPIO pin number 6 */
#define GPIO_PIN_N0_7			7			/* GPIO pin number 7 */
#define GPIO_PIN_N0_8			8			/* GPIO pin number 8 */
#define GPIO_PIN_N0_9			9			/* GPIO pin number 9 */
#define GPIO_PIN_N0_10			10			/* GPIO pin number 10 */
#define GPIO_PIN_N0_11			11			/* GPIO pin number 11 */
#define GPIO_PIN_N0_12			12			/* GPIO pin number 12 */
#define GPIO_PIN_N0_13			13			/* GPIO pin number 13 */
#define GPIO_PIN_N0_14			14			/* GPIO pin number 14 */
#define GPIO_PIN_N0_15			15			/* GPIO pin number 15 */

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN 			0			/* Input (reset state) */
#define GPIO_MODE_OUT			1			/* General purpose output mode */
#define GPIO_MODE_ALTFN 		2			/* Alternate function mode */
#define GPIO_MODE_ANALOG 		3			/* Analog mode */
#define GPIO_MODE_IT_FT			4			/* Interrupt mode (Falling edge trigger)  */
#define GPIO_MODE_IT_RT			5			/* Interrupt mode (Rising edge trigger)  */
#define GPIO_MODE_IT_RFT		6			/* Interrupt mode (Rising-falling edge trigger)  */

/*
 * @GPIO_PIN_OUT_TYPES
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP			0			/* Output push-pull (reset state) */
#define GPIO_OP_TYPE_OD			1			/* Output open-drain */

/*
 * @GPIO_PIN_SPEEDS
 * GPIO pin possible output speeds
 *
 * Note: Refer to the product datasheets for the values of OSPEEDRy bits versus VDD range and external load.
 */
#define GPIO_SPEED_LOW 			0			/* Low speed */
#define GPIO_SPEED_MEDIUM 		1			/* Medium speed */
#define GPIO_SPEED_FAST 		2			/* Fast speed */
#define GPIO_SPEED_HIGH 		3			/* High speed */

/*
 * @GPIO_PIN_PUPD
 * GPIO pin pull-up AND pull-down configuration macros
 */
#define GPIO_NO_PUPD 			0			/* No pull-up, pull-down */
#define GPIO_PIN_PU 			1			/* Pull-up */
#define GPIO_PIN_PD 			2			/* Pull-down */


/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-Init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
