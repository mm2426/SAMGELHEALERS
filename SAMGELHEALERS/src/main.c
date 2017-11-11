/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>

void InitConUart(void);

int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */
	sysclk_init();
	/* Initialize all peripherals */
	board_init();
	delay_init(sysclk_get_cpu_hz());

	//LED PIN
	gpio_configure_pin(PIO_PC23_IDX, (PIO_OUTPUT_1 | PIO_DEFAULT));

	InitConUart();

	while (1)
	{
		printf("%d\r\n", 90);
		delay_ms(1000);
	}
	
}

/**
 *  Configure UART for debug message output.
 */
void InitConUart(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONSOLE_UART_BAUDRATE,
		.paritytype = UART_MR_PAR_NO
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONSOLE_UART, &uart_serial_options);
}
