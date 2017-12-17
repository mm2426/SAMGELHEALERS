/**
 * \file
 *
 * \brief User board definition template
 *
 */

 /* This file is intended to contain definitions and configuration details for
 * features and devices that are available on the board, e.g., frequency and
 * startup time for an external crystal, external memory devices, LED and USART
 * pins.
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef USER_BOARD_H
#define USER_BOARD_H

#include <conf_board.h>

/** UART0 pins (UTXD0 and URXD0) definitions, PA9,10. */
#define PINS_UART0              (PIO_PA9A_URXD0 | PIO_PA10A_UTXD0)
#define PINS_UART0_FLAGS        (PIO_PERIPH_A | PIO_DEFAULT)
#define PINS_UART0_PIO			PIOA

/** UART1 pins (UTXD1 and URXD1) definitions, PB3, 2. */
#define PINS_UART1              (PIO_PB2A_URXD1 | PIO_PB3A_UTXD1)
#define PINS_UART1_FLAGS        (PIO_PERIPH_A | PIO_DEFAULT)
#define PINS_UART1_PIO			PIOB

/* USART0 Definitions*/
#define PINS_USART0_PIO			PIOA
/** USART1 pins (RXD1, TXD1) definitions, PA21,22,24. */
#define PINS_USART0				(PIO_PA5A_RXD0 | PIO_PA6A_TXD0)
#define PINS_USART0_FLAGS		(PIO_PERIPH_A | PIO_DEFAULT)

/* USART1 Definitions*/
#define PINS_USART1_PIO			PIOA
/** USART1 pins (RXD1, TXD1) definitions, PA21,22,24. */
#define PINS_USART1				(PIO_PA21A_RXD1 | PIO_PA22A_TXD1)
#define PINS_USART1_FLAGS		(PIO_PERIPH_A | PIO_DEFAULT)

/* TWI0 Definitions*/
#define PINS_TWI0_PIO		PIOA
/* TWI0 pins (TWCK0 & TWD0) definitions, PA4, 3. */
#define PINS_TWI0			(PIO_PA4A_TWCK0 | PIO_PA3A_TWD0)
#define PINS_TWI0_FLAGS		(PIO_PERIPH_A | PIO_DEFAULT)

/* Solenoid Valve Pins */
#define PIN_INAVALVE1_IDX		PIO_PA13_IDX
#define PIN_INAVALVE2_IDX		PIO_PA14_IDX

/* Air Pump SSRly */
#define PIN_AIR_PUMP_IDX		PIO_PA24_IDX

/* Debug LED Pin */
#define PIN_DEBUGLED_IDX		PIO_PA16_IDX

#endif // USER_BOARD_H
