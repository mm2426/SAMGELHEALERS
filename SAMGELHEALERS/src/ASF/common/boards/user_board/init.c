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
	/* Init IO Port service to enable configuring pins */
	ioport_init();

	/* Configure UART1 pins */
	gpio_configure_group(PINS_UART1_PIO, PINS_UART1, PINS_UART1_FLAGS);

	/* Configure USART1 pins */
	gpio_configure_group(PINS_USART1_PIO, PINS_USART1, PINS_USART1_FLAGS);
}
