/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef ATMEL_START_PINS_H_INCLUDED
#define ATMEL_START_PINS_H_INCLUDED

#include <hal_gpio.h>

// SAMC21 has 9 pin functions

#define GPIO_PIN_FUNCTION_A 0
#define GPIO_PIN_FUNCTION_B 1
#define GPIO_PIN_FUNCTION_C 2
#define GPIO_PIN_FUNCTION_D 3
#define GPIO_PIN_FUNCTION_E 4
#define GPIO_PIN_FUNCTION_F 5
#define GPIO_PIN_FUNCTION_G 6
#define GPIO_PIN_FUNCTION_H 7
#define GPIO_PIN_FUNCTION_I 8

#define STLED3 GPIO(GPIO_PORTA, 4)
#define P_SDA GPIO(GPIO_PORTA, 8)
#define P_SCL GPIO(GPIO_PORTA, 9)
#define PUMP_EN GPIO(GPIO_PORTA, 13)
#define PUMP_SA GPIO(GPIO_PORTA, 14)
#define PUMP_SB GPIO(GPIO_PORTA, 15)
#define T_SDA GPIO(GPIO_PORTA, 16)
#define T_SCL GPIO(GPIO_PORTA, 17)
#define OLD_CMD GPIO(GPIO_PORTA, 21)
#define CANTX GPIO(GPIO_PORTA, 24)
#define CANRX GPIO(GPIO_PORTA, 25)
#define STLED1 GPIO(GPIO_PORTB, 8)
#define STLED2 GPIO(GPIO_PORTB, 9)
#define UTXFRX GPIO(GPIO_PORTB, 22)
#define URXFTX GPIO(GPIO_PORTB, 23)

#endif // ATMEL_START_PINS_H_INCLUDED
