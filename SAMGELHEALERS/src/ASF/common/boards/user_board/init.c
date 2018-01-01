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

	#if defined(BOARD_NIRA91)
		/* Configure TWI0 Pins (Pressure sensors) */
		gpio_configure_group(PINS_TWI0_PIO, PINS_TWI0, PINS_TWI0_FLAGS);

		/* Enable PIO Clocks (GPIO I/P and Ext Int) */
		pmc_enable_periph_clk(ID_PIOA);
		pmc_enable_periph_clk(ID_PIOB);

		/* Configure Solenoid Pins */
		gpio_configure_pin(PIN_INAVALVE1_IDX, (PIO_OUTPUT_0 | PIO_DEFAULT));
		gpio_configure_pin(PIN_INAVALVE2_IDX, (PIO_OUTPUT_0 | PIO_DEFAULT));

		/* Configure Air Pump SSRly */
		gpio_configure_pin(PIN_AIR_PUMP_IDX, (PIO_OUTPUT_0 | PIO_DEFAULT));

		/* Configure LED Pins */
		gpio_configure_pin(PIN_DEBUGLED_IDX, (PIO_OUTPUT_0 | PIO_DEFAULT));

		/* Configure SW Pins */
		gpio_configure_pin(PIN_SW_PRESS_UP_IDX, (PIO_INPUT | PIO_OPENDRAIN | PIO_DEBOUNCE));
		gpio_configure_pin(PIN_SW_PRESS_DN_IDX, (PIO_INPUT | PIO_OPENDRAIN | PIO_DEBOUNCE));
		gpio_configure_pin(PIN_SW_DURATION_UP_IDX, (PIO_INPUT | PIO_OPENDRAIN | PIO_DEBOUNCE));
		gpio_configure_pin(PIN_SW_DURATION_DN_IDX, (PIO_INPUT | PIO_OPENDRAIN | PIO_DEBOUNCE));
		gpio_configure_pin(PIN_SW_DELAY_UP_IDX, (PIO_INPUT | PIO_OPENDRAIN | PIO_DEBOUNCE));
		#ifndef BOARD_NIRA91
			/* Delay down pin cannot be utilized as input as this pin is short with DISP UART RX (Schematic mistake) */
			gpio_configure_pin(PIN_SW_DELAY_DN_IDX, (PIO_INPUT | PIO_OPENDRAIN | PIO_DEBOUNCE));
		#endif
		gpio_configure_pin(PIN_SW_SOS_IDX, PIO_INPUT);

		/*	
		 *Set debounce filter slow clock divider for PIOA and PIOB.
		 *Calling this function with mask value 0 will not disable the already enabled debounce filters.  
		 *Debounce value set for 100ms. (10Hz Filter)
		 */ 
		pio_set_debounce_filter(PIOA, 0, 10);
		pio_set_debounce_filter(PIOB, 0, 10);

		/* Initialize PIOs interrupt handlers (see PIO definition in board.h). */
		pio_handler_set(PIN_SW_SOS_PIO, PIN_SW_SOS_PIO_ID, PIN_SW_SOS_MASK, (PIO_OPENDRAIN | PIO_DEBOUNCE | PIO_IT_LOW_LEVEL), SosIntHandler);
		
		/* Enable PIO controller IRQs. */
		NVIC_EnableIRQ((IRQn_Type) PIN_SW_SOS_PIO_ID);

		/* PIO configuration for Buttons. */
		pio_handler_set_priority(PIN_SW_SOS_PIO, (IRQn_Type) PIN_SW_SOS_PIO_ID, 0);
		
		/* Enable PIO line interrupts. */
		pio_enable_interrupt(PIN_SW_SOS_PIO, PIN_SW_SOS_MASK);
	#endif
}
