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
#define DISP_UART					UART1
#define DISP_UART_ID				ID_UART1
/** Baudrate setting */
#define DISP_UART_BAUDRATE			921600

/* Sensor1 USART Interface */
#define SEN1_USART					USART1
#define SEN1_USART_ID				ID_USART1
#define SEN1_BAUDRATE				9600

/* Sensor2 USART Interface */
#define SEN2_USART					USART0
#define SEN2_USART_ID				ID_USART0
#define SEN2_BAUDRATE				9600

#define SEN_USART_BUFF_SIZE			200

#endif // CONF_BOARD_H
