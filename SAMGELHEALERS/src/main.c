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


#define BIT_IS_SET(a,b)\
	(a&(1<<b))

#include <asf.h>
#include "senBuffIOPDC.h"

void InitConUart(void);
void SenProcessData(uint8_t senNo);
void SenParseFrame(uint8_t senNo, uint8_t data);

enum rxStates 
{
	q0, q1, q2, q3, q4, q5
};

enum rxStates sen1State = q0;
enum rxStates sen2State = q0;
uint8_t sen1Sum = 0, sen2Sum = 0;
uint8_t sen1Ctr = 0, sen2Ctr = 0;
uint16_t hrtRate = 0;
uint8_t disp = 0, spo2;

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
	SenInitUsart();

	printf("RST\r\n");

	while (1)
	{
		//printf("%d\r\n", 90);

		//Call frequently to update next pointer in PDC
		SenPdcManageBuff();

		if(SenGetRxBytes(1)>5)
		{
			//Run trigger calculation algorithm
			SenProcessData(1);
		}

// 		if(SenGetRxBytes(2))
// 		{
// 			//Pass sen2 data to display
// 			SenProcessData(2);
// 		}

		if(disp)
		{
			//Send data on console UART
			printf("Hrt = %d\r\n", hrtRate);
			printf("Sp02 = %d\r\n", spo2);
			disp = 0;
		}

		//delay_ms(1);
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

void SenProcessData(uint8_t senNo)
{
	/*uint8_t rxBytes;
	if(senNo == 1)
	{
		rxBytes = SenGetRxBytes(1);
	}
	else
	{
		rxBytes = SenGetRxBytes(2);
	}*/

	while(SenGetRxBytes(senNo))
	{
		SenParseFrame(senNo, SenGetByte(senNo));
		//rxBytes--;
	}
}

void SenParseFrame(uint8_t senNo, uint8_t data)
{
	if(senNo == 1)
	{
		switch(sen1State)
		{
			case q0:
				if(data==0x01)
				{
					sen1State = q1;
					sen1Sum = 0x01;
				}
				break;
			case q1:
				if((data>127)&&BIT_IS_SET(data,0))
				{
					sen1State = q2;
					sen1Sum += data;
				}
				else
				{
					sen1State = q0;
				}
				break;
			case q2:
				//Pleath Reading
				sen1State = q3;
				sen1Sum += data;
				break;
			case q3:
				if(data<127)
				{
					//HRMSB
					hrtRate = ((uint16_t)(data&0x03))<<8;
					sen1State = q4;
					sen1Sum += data;
				}
				else
				{
					sen1State = q0;
				}
				break;
			case q4:
				if(data==sen1Sum)
				{
					//Checksum matched
					sen1State = q5;
					sen1Ctr = 0;
				}
				else
				{
					sen1State = q0;
				}
				break;
			case q5:
				sen1Ctr++;
				if(sen1Ctr==4)
				{
					hrtRate |= data;
				}
				else if(sen1Ctr == 9)
				{
					spo2 = data;
					disp = 1;
				}
				else if(sen1Ctr==120)
				{
					sen1State = q0;					
				}
				break;
		}
	}
	else
	{
		
	}
}
