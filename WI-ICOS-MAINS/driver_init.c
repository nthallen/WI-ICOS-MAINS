/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_init.h"
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>

/*! The buffer size for USART */
#define USART_CTRL_BUFFER_SIZE 16

struct usart_async_descriptor USART_CTRL;
struct can_async_descriptor   CAN_CTRL;

static uint8_t USART_CTRL_buffer[USART_CTRL_BUFFER_SIZE];

struct i2c_m_async_desc I2C_P;

struct i2c_m_async_desc I2C_T;

void I2C_P_PORT_init(void)
{

	gpio_set_pin_pull_mode(P_SDA,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(P_SDA, PINMUX_PA08C_SERCOM0_PAD0);

	gpio_set_pin_pull_mode(P_SCL,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(P_SCL, PINMUX_PA09C_SERCOM0_PAD1);
}

void I2C_P_CLOCK_init(void)
{
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM0_GCLK_ID_CORE, CONF_GCLK_SERCOM0_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM0_GCLK_ID_SLOW, CONF_GCLK_SERCOM0_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_mclk_set_APBCMASK_SERCOM0_bit(MCLK);
}

void I2C_P_init(void)
{
	I2C_P_CLOCK_init();
	i2c_m_async_init(&I2C_P, SERCOM0);
	I2C_P_PORT_init();
}

void I2C_T_PORT_init(void)
{

	gpio_set_pin_pull_mode(T_SDA,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(T_SDA, PINMUX_PA16D_SERCOM3_PAD0);

	gpio_set_pin_pull_mode(T_SCL,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(T_SCL, PINMUX_PA17D_SERCOM3_PAD1);
}

void I2C_T_CLOCK_init(void)
{
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM3_GCLK_ID_CORE, CONF_GCLK_SERCOM3_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM3_GCLK_ID_SLOW, CONF_GCLK_SERCOM3_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_mclk_set_APBCMASK_SERCOM3_bit(MCLK);
}

void I2C_T_init(void)
{
	I2C_T_CLOCK_init();
	i2c_m_async_init(&I2C_T, SERCOM3);
	I2C_T_PORT_init();
}

/**
 * \brief USART Clock initialization function
 *
 * Enables register interface and peripheral clock
 */
void USART_CTRL_CLOCK_init()
{

	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM5_GCLK_ID_CORE, CONF_GCLK_SERCOM5_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM5_GCLK_ID_SLOW, CONF_GCLK_SERCOM5_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_mclk_set_APBCMASK_SERCOM5_bit(MCLK);
}

/**
 * \brief USART pinmux initialization function
 *
 * Set each required pin to USART functionality
 */
void USART_CTRL_PORT_init()
{

	gpio_set_pin_function(UTXFRX, PINMUX_PB22D_SERCOM5_PAD2);

	gpio_set_pin_function(URXFTX, PINMUX_PB23D_SERCOM5_PAD3);
}

/**
 * \brief USART initialization function
 *
 * Enables USART peripheral, clocks and initializes USART driver
 */
void USART_CTRL_init(void)
{
	USART_CTRL_CLOCK_init();
	usart_async_init(&USART_CTRL, SERCOM5, USART_CTRL_buffer, USART_CTRL_BUFFER_SIZE, (void *)NULL);
	USART_CTRL_PORT_init();
}

void CAN_CTRL_PORT_init(void)
{

	gpio_set_pin_function(CANRX, PINMUX_PA25G_CAN0_RX);

	gpio_set_pin_function(CANTX, PINMUX_PA24G_CAN0_TX);
}
/**
 * \brief CAN initialization function
 *
 * Enables CAN peripheral, clocks and initializes CAN driver
 */
void CAN_CTRL_init(void)
{
	hri_mclk_set_AHBMASK_CAN0_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, CAN0_GCLK_ID, CONF_GCLK_CAN0_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	can_async_init(&CAN_CTRL, CAN0);
	CAN_CTRL_PORT_init();
}

void system_init(void)
{
	init_mcu();

	// GPIO on PA04

	gpio_set_pin_level(STLED3,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(STLED3, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(STLED3, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA13

	gpio_set_pin_level(PUMP_EN,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(PUMP_EN, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(PUMP_EN, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA14

	gpio_set_pin_level(PUMP_SA,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(PUMP_SA, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(PUMP_SA, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA15

	gpio_set_pin_level(PUMP_SB,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(PUMP_SB, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(PUMP_SB, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA21

	gpio_set_pin_level(OLD_CMD,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   true);

	// Set pin direction to output
	gpio_set_pin_direction(OLD_CMD, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(OLD_CMD, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB08

	gpio_set_pin_level(STLED1,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(STLED1, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(STLED1, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB09

	gpio_set_pin_level(STLED2,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(STLED2, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(STLED2, GPIO_PIN_FUNCTION_OFF);

	I2C_P_init();

	I2C_T_init();
	USART_CTRL_init();
	CAN_CTRL_init();
}
