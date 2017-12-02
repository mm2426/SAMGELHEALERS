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

//#define PLEATH_BUFF_SIZE	50
#define CBUFF_SIZE				100

#define PLEATH_DIFF_THRESHOLD	3
#define PLEATH_DIFF_CYCLES		2

/* System tick frequency in Hz. */
#define SYS_TICK_FREQ        100

#include <asf.h>
#include <string.h>
#include "senBuffIOPDC.h"
#include "cBuff.h"

void InitSystick(void);
void InitDispUart(void);
void SenProcessData(uint8_t senNo);
void SenParseFrame(uint8_t senNo, uint8_t data);
uint8_t GetTrigger(uint8_t currPleath);
void ActivateValves(void);
void SendDispData(void);
uint8_t CalcChkSum (uint8_t * buff, uint8_t len);

enum rxStates 
{
	q0, q1, q2, q3, q4, q5
};

enum valveStates
{
	allOff, trigWait, cuffTighten, pCtrl, cuffHold, cuffRelease
};

struct senState
{
	enum rxStates state;
	uint8_t sum, ctr, pIndex;
}sen1State, sen2State;

struct senData
{
	uint8_t hrMsb, hrLsb, spo2;
	struct cBuff_t pleathBuff;
}sen1Data, sen2Data;

struct cBuff_t trigBuff;
uint8_t trigFound = 0, prevPleath = 0, tempPleath = 0;
uint8_t rising = 1;
enum valveStates ctrlState = allOff;

/* Systick counter */
uint32_t tickCount = 0;

/* Buffers used by display routines */
uint8_t dispPkt[150];

/* Pointer to Disp UART PDC register base */
Pdc *dispUartPdcBase;
pdc_packet_t dispPdcPkt;

int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */
	sysclk_init();
	/* Initialize all peripherals */
	board_init();
	delay_init(sysclk_get_cpu_hz());

	//LED PIN
	gpio_configure_pin(PIO_PC23_IDX, (PIO_OUTPUT_1 | PIO_DEFAULT));

// 	gpio_set_pin_low(PIO_PC23_IDX);
// 	delay_ms(300);
// 	gpio_set_pin_high(PIO_PC23_IDX);
	
	InitDispUart();

	/* Init. variables */
	memset(&sen1Data, 0, sizeof(struct senData));
	memset(&sen2Data, 0, sizeof(struct senData));
	memset(&trigBuff, 0, sizeof(struct cBuff_t));
	sen1State.state = q0;
	sen1State.sum = 0;
	sen1State.ctr = 0;
	sen2State.state = q0;
	sen2State.sum = 0;
	sen2State.ctr = 0;
	
	/* Initialize pressure in reservoir and cuff */

	/* Initialize sensor USARTs */
	SenInitUsart();

	/* Initialize Systick timer to generate interrupts every 10 ms */
	InitSystick();

	/* Enable WDT */

	while (1)
	{
		/* Toggle LED GPIO */
		gpio_toggle_pin(PIO_PC23_IDX);

		/* Emergency switch action */
		
		/* Call frequently to update next pointer in PDC */
		SenPdcManageBuff();

		/* Poll switches */

		/* Collect sensor 1 data and run trigger calculation */
		if(SenGetRxBytes(1)>5)
		{
			SenProcessData(1);
		}

		/* Collect sensor 2 data */
		if(SenGetRxBytes(2)>5)
		{
			SenProcessData(2);
		}

		/* Activate valves */
		ActivateValves();

		/* Send data collected from sensors to display */
		SendDispData();

		/* Clear WDT */
	}
}

/**
 *  \brief Handler for System Tick interrupt.
 *
 *  Process System Tick Event.
 *  Increment the ul_ms_ticks counter.
 */
void SysTick_Handler(void)
{
	tickCount++;
}

/**
 *  Configure system tick to generate an interrupt every 10 ms.
 */

void InitSystick(void)
{
	uint32_t ul_flag;
	ul_flag = SysTick_Config(sysclk_get_cpu_hz() / SYS_TICK_FREQ);
	if (ul_flag) {
		/* Systick configuration error */
		while (1) {
		}
	}
}

/**
 *  Configure display UART for output.
 */
void InitDispUart(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = DISP_UART_BAUDRATE,
		.paritytype = UART_MR_PAR_NO
	};

	sysclk_enable_peripheral_clock(DISP_UART_ID);
	stdio_serial_init(DISP_UART, &uart_serial_options);
	
	uart_enable_tx(DISP_UART);
	uart_disable_rx(DISP_UART);

	dispUartPdcBase = uart_get_pdc_base(DISP_UART);
	pdc_enable_transfer(dispUartPdcBase, PERIPH_PTCR_TXTEN);
}

void SenProcessData(uint8_t senNo)
{
	uint8_t i;
	/* Process only 1 frame at a time i.e. 5B */
	for ( i = 0; i < 5; i++)
	{
		if(SenGetRxBytes(senNo))
		{
			SenParseFrame(senNo, SenGetByte(senNo));
		}
	}
}

void SenParseFrame(uint8_t senNo, uint8_t data)
{
	if(senNo == 1)
	{
		switch(sen1State.state)
		{
			case q0:
				if(data==0x01)
				{
					sen1State.state = q1;
					sen1State.sum = 0x01;
				}
				break;
			case q1:
				if((data>127)&&BIT_IS_SET(data,0))
				{
					sen1State.state = q2;
					sen1State.sum += data;
				}
				else
				{
					sen1State.state = q0;
				}
				break;
			case q2:
				//Pleath Reading
				tempPleath = data;
				sen1State.state = q3;
				sen1State.sum += data;
				sen1State.pIndex = 3;
				break;
			case q3:
				if(data<127)
				{
					//HRMSB
					//hrtRate = ((uint16_t)(data&0x03))<<8;
					sen1Data.hrMsb = data;
					sen1State.state = q4;
					sen1State.sum += data;
				}
				else
				{
					sen1State.state = q0;
				}
				break;
			case q4:
				/* If Checksum matched */
				if(data==sen1State.sum)
				{
					/* Write data value to pleath circular buffer */
					CBuffWriteByte(&sen1Data.pleathBuff, tempPleath);
					//CBuffWriteByte(&trigBuff, tempPleath);

					if ((!trigFound) && (GetTrigger(tempPleath)))
					{
						/* Write data value to trigger circular buffer */
						CBuffWriteByte(&trigBuff, tempPleath);
						/* This flag will be reset in the pressure control loop */
						trigFound = 1;
					}
					else
					{
						/* Write 0 to trigger circular buffer */
						CBuffWriteByte(&trigBuff, 0);
					}
					sen1State.state = q5;
					sen1State.ctr = 0;
				}
				else
				{
					sen1State.state = q0;
				}
				break;
			case q5:
				sen1State.ctr++;
				if(sen1State.ctr == sen1State.pIndex)
				{
					/* Write data value to pleath circular buffer */
					CBuffWriteByte(&sen1Data.pleathBuff, data);
					//CBuffWriteByte(&trigBuff, data);

					if ((!trigFound) && (GetTrigger(data)))
					{
						/* Write data value to trigger circular buffer */
						CBuffWriteByte(&trigBuff, data);
					}
					else
					{
						/* Write 0 to trigger circular buffer */
						CBuffWriteByte(&trigBuff, 0);
					}
					sen1State.pIndex += 5;
				}
				else if(sen1State.ctr == 4)
				{
					sen1Data.hrLsb = data;
					//hrtRate |= data;
				}
				else if(sen1State.ctr == 9)
				{
					sen1Data.spo2 = data;
					//spo2 = data;
					//disp = 1;
				}
				else if(sen1State.ctr==120)
				{
					sen1State.state = q0;					
				}
				break;
		}
	}
	else
	{
		
	}
}

uint8_t GetTrigger(uint8_t currPleath)
{
	int8_t diff = (int8_t)prevPleath - (int8_t)currPleath;
	static uint8_t ctr = 0;
	
	//if (!trigFound)
	{
		/* Find trigger during rising ppg signal */
		if(rising)
		{
			if ((diff <= -PLEATH_DIFF_THRESHOLD) && (!trigFound))
			{
				ctr+=1;
			}
			else
			{
				ctr = 0;
			}

			if((ctr>=PLEATH_DIFF_CYCLES) && (!trigFound))
			{
				/* This flag will be reset in the pressure control loop */
				trigFound = 1;
				rising = 0;
				ctr = 0;
			}
		}
		else
		{
			/* Once trigFound flag is reset, wait till next rising cycle */
			if((diff >= 0) && (diff <= PLEATH_DIFF_THRESHOLD))
			{
				rising = 1;
				ctr = 0;
			}
		}
		prevPleath = currPleath;
	}
	return trigFound;
}

void ActivateValves(void)
{
	if(trigFound)
	{
		switch(ctrlState)
		{
			case allOff:
				tickCount = 0;
				ctrlState = trigWait;
				break;
			case trigWait:
				/* If 200 ms elapsed */
				if(tickCount>=200)
				{
					tickCount = 0;
					trigFound = 0;
					ctrlState = allOff;
				}
				break;
			default:
				break;
		}
	}
}

void SendDispData(void)
{
	uint8_t readS2 = 0;

	/* If previous transfer not complete, return */
	if (!(uart_get_status(DISP_UART) & UART_SR_ENDTX)) 
	{
		return;
	}

	if(CBuffGetRxBytes(&sen2Data.pleathBuff) >= 25)
	{
		readS2 = 1;
	}

	/* Check if data in sen1Pleath buff >= 25 */
	if(CBuffGetRxBytes(&sen1Data.pleathBuff) > 25 && CBuffGetRxBytes(&trigBuff) > 25)
	{
		/* Frame 1*/
		dispPkt[0] = '$';
		/* Sensor Status */
		dispPkt[1] = '1';
		/* Sensor 1 Pleath Data */
		dispPkt[2] = CBuffReadByte(&sen1Data.pleathBuff);
		/* Trigger Data */
		dispPkt[3] = CBuffReadByte(&trigBuff);
		/* Sensor 2 Pleath Data */
		if(readS2)
			dispPkt[4] = CBuffReadByte(&sen2Data.pleathBuff);
		else
			dispPkt[4] = 0;
		/* Checksum */
		dispPkt[5] = CalcChkSum(&dispPkt[0],5);

		/* Frame 2*/
		dispPkt[6] = '$';
		/* S1 Heart Rate MSB */
		dispPkt[7] = sen1Data.hrMsb;
		/* Sensor 1 Pleath Data */
		dispPkt[8] = CBuffReadByte(&sen1Data.pleathBuff);
		/* Trigger Data */
		dispPkt[9] = CBuffReadByte(&trigBuff);
		/* Sensor 2 Pleath Data */
		if(readS2)
			dispPkt[10] = CBuffReadByte(&sen2Data.pleathBuff);
		else
			dispPkt[10] = 0;
		/* Checksum */
		dispPkt[11] = CalcChkSum(&dispPkt[6],5);

		/* Frame 3*/
		dispPkt[12] = '$';
		/* S1 Heart Rate LSB */
		dispPkt[13] = sen1Data.hrLsb;
		/* Sensor 1 Pleath Data */
		dispPkt[14] = CBuffReadByte(&sen1Data.pleathBuff);
		/* Trigger Data */
		dispPkt[15] = CBuffReadByte(&trigBuff);
		/* Sensor 2 Pleath Data */
		if(readS2)
			dispPkt[16] = CBuffReadByte(&sen2Data.pleathBuff);
		else
			dispPkt[16] = 0;
		/* Checksum */
		dispPkt[17] = CalcChkSum(&dispPkt[12],5);

		/* Frame 4*/
		dispPkt[18] = '$';
		/* S1 SpO2 */
		dispPkt[19] = sen1Data.spo2;
		/* Sensor 1 Pleath Data */
		dispPkt[20] = CBuffReadByte(&sen1Data.pleathBuff);
		/* Trigger Data */
		dispPkt[21] = CBuffReadByte(&trigBuff);
		/* Sensor 2 Pleath Data */
		if(readS2)
			dispPkt[22] = CBuffReadByte(&sen2Data.pleathBuff);
		else
			dispPkt[22] = 0;
		/* Checksum */
		dispPkt[23] = CalcChkSum(&dispPkt[18],5);

		/* Frame 5*/
		dispPkt[24] = '$';
		/* S2 Heart Rate MSB */
		dispPkt[25] = sen2Data.hrMsb;
		/* Sensor 1 Pleath Data */
		dispPkt[26] = CBuffReadByte(&sen1Data.pleathBuff);
		/* Trigger Data */
		dispPkt[27] = CBuffReadByte(&trigBuff);
		/* Sensor 2 Pleath Data */
		if(readS2)
			dispPkt[28] = CBuffReadByte(&sen2Data.pleathBuff);
		else
			dispPkt[28] = 0;
		/* Checksum */
		dispPkt[29] = CalcChkSum(&dispPkt[24],5);

		/* Frame 6*/
		dispPkt[30] = '$';
		/* S2 Heart Rate LSB */
		dispPkt[31] = sen2Data.hrLsb;
		/* Sensor 1 Pleath Data */
		dispPkt[32] = CBuffReadByte(&sen1Data.pleathBuff);
		/* Trigger Data */
		dispPkt[33] = CBuffReadByte(&trigBuff);
		/* Sensor 2 Pleath Data */
		if(readS2)
			dispPkt[34] = CBuffReadByte(&sen2Data.pleathBuff);
		else
			dispPkt[34] = 0;
		/* Checksum */
		dispPkt[35] = CalcChkSum(&dispPkt[30],5);

		/* Frame 7*/
		dispPkt[36] = '$';
		/* S2 SpO2 */
		dispPkt[37] = sen2Data.spo2;
		/* Sensor 1 Pleath Data */
		dispPkt[38] = CBuffReadByte(&sen1Data.pleathBuff);
		/* Trigger Data */
		dispPkt[39] = CBuffReadByte(&trigBuff);
		/* Sensor 2 Pleath Data */
		if(readS2)
			dispPkt[40] = CBuffReadByte(&sen2Data.pleathBuff);
		else
			dispPkt[40] = 0;
		/* Checksum */
		dispPkt[41] = CalcChkSum(&dispPkt[36],5);

		/* Frame 8*/
		dispPkt[42] = '$';
		/* Pressure Set Point */
		dispPkt[43] = '0';
		/* Sensor 1 Pleath Data */
		dispPkt[44] = CBuffReadByte(&sen1Data.pleathBuff);
		/* Trigger Data */
		dispPkt[45] = CBuffReadByte(&trigBuff);
		/* Sensor 2 Pleath Data */
		if(readS2)
			dispPkt[46] = CBuffReadByte(&sen2Data.pleathBuff);
		else
			dispPkt[46] = 0;
		/* Checksum */
		dispPkt[47] = CalcChkSum(&dispPkt[42],5);

		/* Frame 9*/
		dispPkt[48] = '$';
		/* Trigger Delay */
		dispPkt[49] = '0';
		/* Sensor 1 Pleath Data */
		dispPkt[50] = CBuffReadByte(&sen1Data.pleathBuff);
		/* Trigger Data */
		dispPkt[51] = CBuffReadByte(&trigBuff);
		/* Sensor 2 Pleath Data */
		if(readS2)
			dispPkt[52] = CBuffReadByte(&sen2Data.pleathBuff);
		else
			dispPkt[52] = 0;
		/* Checksum */
		dispPkt[53] = CalcChkSum(&dispPkt[48],5);

		/* Frame 10 */
		dispPkt[54] = '$';
		/* Cuff Hold Duration */
		dispPkt[55] = '0';
		/* Sensor 1 Pleath Data */
		dispPkt[56] = CBuffReadByte(&sen1Data.pleathBuff);
		/* Trigger Data */
		dispPkt[57] = CBuffReadByte(&trigBuff);
		/* Sensor 2 Pleath Data */
		if(readS2)
			dispPkt[58] = CBuffReadByte(&sen2Data.pleathBuff);
		else
			dispPkt[58] = 0;
		/* Checksum */
		dispPkt[59] = CalcChkSum(&dispPkt[54],5);

		/* Frame 11 */
		dispPkt[60] = '$';
		/* Cuff Pressure MSB */
		dispPkt[61] = '0';
		/* Sensor 1 Pleath Data */
		dispPkt[62] = CBuffReadByte(&sen1Data.pleathBuff);
		/* Trigger Data */
		dispPkt[63] = CBuffReadByte(&trigBuff);
		/* Sensor 2 Pleath Data */
		if(readS2)
			dispPkt[64] = CBuffReadByte(&sen2Data.pleathBuff);
		else
			dispPkt[64] = 0;
		/* Checksum */
		dispPkt[65] = CalcChkSum(&dispPkt[60],5);

		/* Frame 12 */
		dispPkt[66] = '$';
		/* Cuff Pressure LSB */
		dispPkt[67] = '0';
		/* Sensor 1 Pleath Data */
		dispPkt[68] = CBuffReadByte(&sen1Data.pleathBuff);
		/* Trigger Data */
		dispPkt[69] = CBuffReadByte(&trigBuff);
		/* Sensor 2 Pleath Data */
		if(readS2)
			dispPkt[70] = CBuffReadByte(&sen2Data.pleathBuff);
		else
			dispPkt[70] = 0;
		/* Checksum */
		dispPkt[71] = CalcChkSum(&dispPkt[66],5);

		/* Frame 13 */
		dispPkt[72] = '$';
		/* Reservoir Pressure MSB */
		dispPkt[73] = '0';
		/* Sensor 1 Pleath Data */
		dispPkt[74] = CBuffReadByte(&sen1Data.pleathBuff);
		/* Trigger Data */
		dispPkt[75] = CBuffReadByte(&trigBuff);
		/* Sensor 2 Pleath Data */
		if(readS2)
			dispPkt[76] = CBuffReadByte(&sen2Data.pleathBuff);
		else
			dispPkt[76] = 0;
		/* Checksum */
		dispPkt[77] = CalcChkSum(&dispPkt[72],5);

		/* Frame 14 */
		dispPkt[78] = '$';
		/* Reservoir Pressure LSB */
		dispPkt[79] = '0';
		/* Sensor 1 Pleath Data */
		dispPkt[80] = CBuffReadByte(&sen1Data.pleathBuff);
		/* Trigger Data */
		dispPkt[81] = CBuffReadByte(&trigBuff);
		/* Sensor 2 Pleath Data */
		if(readS2)
			dispPkt[82] = CBuffReadByte(&sen2Data.pleathBuff);
		else
			dispPkt[82] = 0;
		/* Checksum */
		dispPkt[83] = CalcChkSum(&dispPkt[78],5);

		/* Frame 15 */
		dispPkt[84] = '$';
		/* Blank */
		dispPkt[85] = '0';
		/* Sensor 1 Pleath Data */
		dispPkt[86] = CBuffReadByte(&sen1Data.pleathBuff);
		/* Trigger Data */
		dispPkt[87] = CBuffReadByte(&trigBuff);
		/* Sensor 2 Pleath Data */
		if(readS2)
			dispPkt[88] = CBuffReadByte(&sen2Data.pleathBuff);
		else
			dispPkt[88] = 0;
		/* Checksum */
		dispPkt[89] = CalcChkSum(&dispPkt[84],5);

		/* Frame 16 */
		dispPkt[90] = '$';
		/* Blank */
		dispPkt[91] = '0';
		/* Sensor 1 Pleath Data */
		dispPkt[92] = CBuffReadByte(&sen1Data.pleathBuff);
		/* Trigger Data */
		dispPkt[93] = CBuffReadByte(&trigBuff);
		/* Sensor 2 Pleath Data */
		if(readS2)
			dispPkt[94] = CBuffReadByte(&sen2Data.pleathBuff);
		else
			dispPkt[94] = 0;
		/* Checksum */
		dispPkt[95] = CalcChkSum(&dispPkt[90],5);

		/* Frame 17 */
		dispPkt[96] = '$';
		/* Blank */
		dispPkt[97] = '0';
		/* Sensor 1 Pleath Data */
		dispPkt[98] = CBuffReadByte(&sen1Data.pleathBuff);
		/* Trigger Data */
		dispPkt[99] = CBuffReadByte(&trigBuff);
		/* Sensor 2 Pleath Data */
		if(readS2)
			dispPkt[100] = CBuffReadByte(&sen2Data.pleathBuff);
		else
			dispPkt[100] = 0;
		/* Checksum */
		dispPkt[101] = CalcChkSum(&dispPkt[96],5);

		/* Frame 18 */
		dispPkt[102] = '$';
		/* Blank */
		dispPkt[103] = '0';
		/* Sensor 1 Pleath Data */
		dispPkt[104] = CBuffReadByte(&sen1Data.pleathBuff);
		/* Trigger Data */
		dispPkt[105] = CBuffReadByte(&trigBuff);
		/* Sensor 2 Pleath Data */
		if(readS2)
			dispPkt[106] = CBuffReadByte(&sen2Data.pleathBuff);
		else
			dispPkt[106] = 0;
		/* Checksum */
		dispPkt[107] = CalcChkSum(&dispPkt[102],5);

		/* Frame 19 */
		dispPkt[108] = '$';
		/* Blank */
		dispPkt[109] = '0';
		/* Sensor 1 Pleath Data */
		dispPkt[110] = CBuffReadByte(&sen1Data.pleathBuff);
		/* Trigger Data */
		dispPkt[111] = CBuffReadByte(&trigBuff);
		/* Sensor 2 Pleath Data */
		if(readS2)
			dispPkt[112] = CBuffReadByte(&sen2Data.pleathBuff);
		else
			dispPkt[112] = 0;
		/* Checksum */
		dispPkt[113] = CalcChkSum(&dispPkt[108],5);

		/* Frame 20 */
		dispPkt[114] = '$';
		/* Blank */
		dispPkt[115] = '0';
		/* Sensor 1 Pleath Data */
		dispPkt[116] = CBuffReadByte(&sen1Data.pleathBuff);
		/* Trigger Data */
		dispPkt[117] = CBuffReadByte(&trigBuff);
		/* Sensor 2 Pleath Data */
		if(readS2)
			dispPkt[118] = CBuffReadByte(&sen2Data.pleathBuff);
		else
			dispPkt[118] = 0;
		/* Checksum */
		dispPkt[119] = CalcChkSum(&dispPkt[114],5);

		/* Frame 21 */
		dispPkt[120] = '$';
		/* Blank */
		dispPkt[121] = '0';
		/* Sensor 1 Pleath Data */
		dispPkt[122] = CBuffReadByte(&sen1Data.pleathBuff);
		/* Trigger Data */
		dispPkt[123] = CBuffReadByte(&trigBuff);
		/* Sensor 2 Pleath Data */
		if(readS2)
			dispPkt[124] = CBuffReadByte(&sen2Data.pleathBuff);
		else
			dispPkt[124] = 0;
		/* Checksum */
		dispPkt[125] = CalcChkSum(&dispPkt[120],5);

		/* Frame 22 */
		dispPkt[126] = '$';
		/* Blank */
		dispPkt[127] = '0';
		/* Sensor 1 Pleath Data */
		dispPkt[128] = CBuffReadByte(&sen1Data.pleathBuff);
		/* Trigger Data */
		dispPkt[129] = CBuffReadByte(&trigBuff);
		/* Sensor 2 Pleath Data */
		if(readS2)
			dispPkt[130] = CBuffReadByte(&sen2Data.pleathBuff);
		else
			dispPkt[130] = 0;
		/* Checksum */
		dispPkt[131] = CalcChkSum(&dispPkt[126],5);

		/* Frame 23 */
		dispPkt[132] = '$';
		/* Blank */
		dispPkt[133] = '0';
		/* Sensor 1 Pleath Data */
		dispPkt[134] = CBuffReadByte(&sen1Data.pleathBuff);
		/* Trigger Data */
		dispPkt[135] = CBuffReadByte(&trigBuff);
		/* Sensor 2 Pleath Data */
		if(readS2)
			dispPkt[136] = CBuffReadByte(&sen2Data.pleathBuff);
		else
			dispPkt[136] = 0;
		/* Checksum */
		dispPkt[137] = CalcChkSum(&dispPkt[132],5);
		
		/* Frame 24 */
		dispPkt[138] = '$';
		/* Blank */
		dispPkt[139] = '0';
		/* Sensor 1 Pleath Data */
		dispPkt[140] = CBuffReadByte(&sen1Data.pleathBuff);
		/* Trigger Data */
		dispPkt[141] = CBuffReadByte(&trigBuff);
		/* Sensor 2 Pleath Data */
		if(readS2)
			dispPkt[142] = CBuffReadByte(&sen2Data.pleathBuff);
		else
			dispPkt[142] = 0;
		/* Checksum */
		dispPkt[143] = CalcChkSum(&dispPkt[138],5);

		/* Frame 25 */
		dispPkt[144] = '$';
		/* Blank */
		dispPkt[145] = '0';
		/* Sensor 1 Pleath Data */
		dispPkt[146] = CBuffReadByte(&sen1Data.pleathBuff);
		/* Trigger Data */
		dispPkt[147] = CBuffReadByte(&trigBuff);
		/* Sensor 2 Pleath Data */
		if(readS2)
			dispPkt[148] = CBuffReadByte(&sen2Data.pleathBuff);
		else
			dispPkt[148] = 0;
		/* Checksum */
		dispPkt[149] = CalcChkSum(&dispPkt[144],5);
		
		/* Initiate PDC Transfer for 150 bytes */
		dispPdcPkt.ul_addr = (uint32_t) dispPkt;
		dispPdcPkt.ul_size = 150;
		pdc_tx_init(dispUartPdcBase, &dispPdcPkt, NULL);
	}
}

uint8_t CalcChkSum (uint8_t * buff, uint8_t len)
{
	uint8_t sum = buff[0], i;
	for (i = 1; i < len; i++)
	{
		sum += buff[i];
	}
	return sum;	
}