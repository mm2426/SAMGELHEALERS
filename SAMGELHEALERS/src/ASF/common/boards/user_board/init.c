/**
 * \file
 *
 * \brief User board initialization template
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include <asf.h>
#include <board.h>
#include <conf_board.h>

void board_init(void)
{
	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;

	/* Init IO Port service to enable configuring pins */
	ioport_init();

	/* Configure UART0 pins */
	gpio_configure_group(PINS_UART0_PIO, PINS_UART0, PINS_UART0_FLAGS);

	/* Configure UART1 pins */
	gpio_configure_group(PINS_UART1_PIO, PINS_UART1, PINS_UART1_FLAGS);

	/* Configure USART0 pins */
	gpio_configure_group(PINS_USART0_PIO, PINS_USART0, PINS_USART0_FLAGS);

	/* Configure USART1 pins */
	gpio_configure_group(PINS_USART1_PIO, PINS_USART1, PINS_USART1_FLAGS);

	//Configure TWI0 Pins
	gpio_configure_group(PINS_TWI0_PIO, PINS_TWI0, PINS_TWI0_FLAGS);

	//Configure Solenoid Pins
	gpio_configure_pin(PIN_INAVALVE1_IDX, (PIO_OUTPUT_0 | PIO_DEFAULT));
	gpio_configure_pin(PIN_INAVALVE2_IDX, (PIO_OUTPUT_0 | PIO_DEFAULT));

	//Configure Air Pump SSRly
	gpio_configure_pin(PIN_AIR_PUMP_IDX, (PIO_OUTPUT_0 | PIO_DEFAULT));

	//Configure LED Pins
	gpio_configure_pin(PIN_DEBUGLED_IDX, (PIO_OUTPUT_0 | PIO_DEFAULT));
}
