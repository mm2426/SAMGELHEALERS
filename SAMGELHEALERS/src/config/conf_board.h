/**
 * \file
 *
 * \brief User board configuration template
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef CONF_BOARD_H
#define CONF_BOARD_H

/* External XTAL related params */
#define BOARD_FREQ_MAINCK_XTAL		(12000000UL)
#define BOARD_FREQ_MAINCK_BYPASS	(12000000UL)
#define BOARD_OSC_STARTUP_US		(15625UL)

/** Console UART Interface */
#define CONSOLE_UART				UART1
#define CONSOLE_UART_ID				ID_UART1
/** Baudrate setting */
#define CONSOLE_UART_BAUDRATE		115200

#endif // CONF_BOARD_H
