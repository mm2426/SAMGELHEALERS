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
#define BOARD_FREQ_MAINCK_XTAL			(12000000UL)
#define BOARD_FREQ_MAINCK_BYPASS		(12000000UL)
#define BOARD_OSC_STARTUP_US			(15625UL)
#define BOARD_FREQ_SLCK_XTAL			(32768UL)
#define BOARD_FREQ_SLCK_BYPASS			(32768UL)

/** Console UART Interface */
#if defined(BOARD_XPLND)
	#define DISP_UART					UART1
	#define DISP_UART_ID				ID_UART1
#elif defined(BOARD_NIRA91)
	#define DISP_USART					USART1
	#define DISP_USART_ID				ID_USART1
#else
	#error "Board Selection Macro Undefined"
#endif

/** Baudrate setting */
#define DISP_UART_BAUDRATE				921600

/* Sensor1 USART Interface */
#if (defined(BOARD_XPLND) || defined(BOARD_NIRA91))
	#define SEN1_USART					USART0
	#define SEN1_USART_ID				ID_USART0
	#define SEN1_BAUDRATE				9600
#else
	#error "Board Selection Macro Undefined"
#endif

/* Sensor2 USART Interface */
#if defined(BOARD_XPLND)
	#define SEN2_USART					USART1
	#define SEN2_USART_ID				ID_USART1
	#define SEN2_BAUDRATE				9600
#elif defined(BOARD_NIRA91)
	#define SEN2_UART					UART0
	#define SEN2_UART_ID				ID_UART0
	#define SEN2_BAUDRATE				9600
#else
	#error "Board Selection Macro Undefined"
#endif

#define SEN_USART_BUFF_SIZE				200

#define BOARD_TWI						TWI0
#define BOARD_TWI_ID					ID_TWI0

#endif // CONF_BOARD_H
