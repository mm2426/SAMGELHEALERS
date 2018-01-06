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

/* Size of S1 and S2 pleath buffers */
#define CBUFF_SIZE				200
/* Rate of change (used in trigger calculation algo) */
#define PLEATH_DIFF_THRESHOLD	3
/* No of consecutive cycles (used in trigger calculation algo) */
#define PLEATH_DIFF_CYCLES		2
/* Pressure regulation is done after trigger is found */
#define CTRL_TYPE_PRESSURE		1
/* Run peak detection algorithm for finding trigger */
#define ALGO_TYPE_PK_DET		1
/* Enable algorithm test mode (disables valve operation) */
//#define ALGO_TEST_MODE_EN		1
/* Display average time between pleath cycles (disables sending of pleath data) */
//#define CYCLE_AVG_DISP_EN		1
/* Maximum pressure to be maintained in the reservoir */
#define MAX_RESERVOIR_P			9.5f
#define CUTOFF_RESERVOIR_P		8.5f
#ifndef CTRL_TYPE_PRESSURE
	/* Default time required to fill the cuff */
	#define DEFAULT_FILL_DUR	100	
#else
	/* Default pressure set point value in PSI */
	#define DEFAULT_PRESSURE	5.0f
#endif
/* Default Cuff hold duration in ms */
#define DEFAULT_DURATION		100 

/* Default delay (to be subtracted from avg cycle time (when pk det.) / to be added after trigger found) in ms */
#define DEFAULT_DELAY			200

#define MAX_PSET_POINT			6.0f
#define MIN_PSET_POINT			1.5f
#define MAX_HOLD_DUR			1000
#define MIN_HOLD_DUR			0
#define MAX_DELAY_DUR			2000
#define MIN_DELAY_DUR			0

/* Pressure increments set to 0.1 PSI */
#define PRESSURE_INCR			0.1f
/* Hold increments set to 5 ms */
#define HOLD_INCR				5
/* Delay increments set to 5 ms */
#define DELAY_INCR				5

/* System tick frequency in Hz. */
#define SYS_TICK_FREQ			1000

#include <asf.h>
#include <string.h>
#include "senBuffIOPDC.h"
#include "cBuff.h"
#include "pressureSens.h"

enum rxStates
{
	q0, q1, q2, q3, q4, q5
};

enum valveStates
{
	s1CloseS2Close, s1CloseS2Open, s1OpenS2Close, s1OpenS2Open
};
enum valveStates valveState = s1CloseS2Close;

enum ctrlStates
{
	#if defined(ALGO_TYPE_PK_DET)
		insertDelay, 
	#endif
	fillCuff, waitCycles, holdCuff, releaseCuff
};

struct senState
{
	enum rxStates state;
	uint8_t sum, ctr, pIndex;
	uint8_t tempPleath;
}sen1State, sen2State;

struct senData
{
	uint8_t hrMsb, hrLsb, spo2;
	struct cBuff_t pleathBuff;
}sen1Data, sen2Data;

void InitPeripherals(void);
void InitTWI(void);
void InitSystick(void);
void InitDispUart(void);
void SenProcessData(uint8_t senNo);
void SenParseFrame(uint8_t senNo, uint8_t data);
uint8_t GetTrigger(uint8_t currPleath);
void ActivateValves(void);
void SetValveState(enum valveStates st);
void SendDispData(void);
uint8_t CalcChkSum (uint8_t * buff, uint8_t len);
void WriteEEPROM(Twi * Port, uint8_t chipAddr, uint8_t memPage, uint8_t *dPkt, uint16_t dLen);
void ReadEEPROM(Twi * Port, uint8_t chipAddr, uint8_t memPage, uint8_t *dPkt, uint16_t dLen);
void ManageResP(void);
void PollSwitches(void);

struct cBuff_t trigBuff;
uint8_t trigFound = 0, prevPleath = 0;

#ifndef ALGO_TYPE_PK_DET 
	uint8_t rising = 1;
#else
	uint8_t rising = 0;
	uint32_t prevCycleTime[3]={};
	uint32_t avgCycleTime = 0, cycleTime = 0;
#endif

#if defined(CYCLE_AVG_DISP_EN)
	uint32_t dispTick = 0;
#endif

#if defined(ALGO_TYPE_PK_DET)
	enum ctrlStates ctrlState = insertDelay;
#else
	enum ctrlStates ctrlState = fillCuff;
#endif

enum ctrlStates nextCtrlState = waitCycles;

/* Systick counter */
uint32_t tickCount = 0, tickDur = 0;

/* Buffers used by display routines */
uint8_t dispPkt[150];

/* Pointer to Disp UART PDC register base */
Pdc *dispUartPdcBase;
pdc_packet_t dispPdcPkt;

#ifndef CTRL_TYPE_PRESSURE
	uint32_t fillDur = DEFAULT_FILL_DUR;
#else
	/* Cuff pressure regulation set point */
	float pSetPt = DEFAULT_PRESSURE;
#endif
uint32_t holdDur = DEFAULT_DURATION, delayParam = DEFAULT_DELAY; 
float resPVal = 0;

int main (void)
{
	uint16_t temp = 0;
	uint32_t tempCount = 0;
	float p1Val = 0, p2Val = 0;
	/* Insert system clock initialization code here (sysclk_init()). */
	sysclk_init();
	/* Initialize all peripherals */
	board_init();
	delay_init(sysclk_get_cpu_hz());
	
	SetValveState(s1CloseS2Close);
	gpio_set_pin_low(PIN_AIR_PUMP_IDX);
	delay_ms(500);

	InitPeripherals();

// 	SetValveState(s1OpenS2Open);
// 	while(1);
	
// 	SetValveState(s1CloseS2Close);
// 	while(1)
// 	{
// 		ReadPressureSen(BOARD_TWI, ADDR_PSEN2, dispPkt);
// 		temp = ((((uint16_t)dispPkt[0])<<8)| dispPkt[1]);
// 		p2Val = ((float)temp/16383.0f)*PSEN2_MAXP;
// 	}
		
	#ifndef ALGO_TEST_MODE_EN
		/* Initialize pressure in reservoir */
		SetValveState(s1CloseS2Close);
/*		gpio_set_pin_high(PIN_AIR_PUMP_IDX);*/
		while(p2Val<=5.0f)
		{
			ReadPressureSen(BOARD_TWI, ADDR_PSEN2, dispPkt);
			temp = ((((uint16_t)dispPkt[0])<<8)| dispPkt[1]);
			p2Val = ((float)temp/16383.0f)*PSEN2_MAXP;
		}
	#endif

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

	while (1)
	{
		/* Toggle LED GPIO */
		//gpio_toggle_pin(PIO_PC23_IDX);

		/* Manage reservoir pressure */
		//ManageResP();
		
		/* Emergency switch action already defined in ISR */
		
		/* Call frequently to update next pointer in PDC */
		SenPdcManageBuff();

		/* Poll switches */
		PollSwitches();

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
	cycleTime++;
	#if defined(CYCLE_AVG_DISP_EN)
		dispTick++;
	#endif
}

/**
 *  \brief Handler for External interrupt on SOS Pin.
 *
 *  Process External Interrupt Event.
 *  Open all valves and turn off the compressor.
 *  Hang up in infinite loop until next reset.
 */

void SosIntHandler(uint32_t ul_id, uint32_t ul_mask)
{
	if (PIN_SW_SOS_PIO_ID != ul_id || PIN_SW_SOS_MASK != ul_mask)
		return;

	SetValveState(s1OpenS2Open);
	gpio_set_pin_low(PIN_AIR_PUMP_IDX);

	while(1);
}

void InitPeripherals(void)
{

	/* Initialize Display UART */
	InitDispUart();

	/* Initialize sensor USARTs */
	SenInitUsart();

	/* Init TWI */
	InitTWI();

	/* Initialize Systick timer to generate interrupts every 10 ms */
	InitSystick();

	/* Enable WDT */

}

void InitTWI(void)
{
	twi_options_t twiSettings = {
		sysclk_get_peripheral_hz(),
		100000,
		0,
		0
	};
	/* Enable the peripheral clock in the PMC. */
	sysclk_enable_peripheral_clock(BOARD_TWI_ID);

	/* Enable TWI master mode */
	twi_enable_master_mode(BOARD_TWI);

	/* Initialize TWI peripheral */
	twi_master_init(BOARD_TWI, &twiSettings);
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
	#if defined(BOARD_XPLND)
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
	#elif defined(BOARD_NIRA91)
		const sam_usart_opt_t usart_console_settings = {
			DISP_UART_BAUDRATE,
			US_MR_CHRL_8_BIT,
			US_MR_PAR_NO,
			US_MR_NBSTOP_1_BIT,
			US_MR_CHMODE_NORMAL,
			/* This field is only used in IrDA mode. */
			0
		};

		/* Enable the peripheral clock in the PMC. */
		sysclk_enable_peripheral_clock(DISP_USART_ID);

		/* Configure USART in RS485 mode. */
		usart_init_rs232(DISP_USART, &usart_console_settings,
		sysclk_get_peripheral_hz());

		/* Enable TX function. */
		usart_disable_rx(DISP_USART);
		usart_enable_tx(DISP_USART);

		/* Get board USART PDC base address and enable receiver and transmitter. */
		dispUartPdcBase = usart_get_pdc_base(DISP_USART);
		pdc_enable_transfer(dispUartPdcBase, PERIPH_PTCR_TXTEN);

	#endif

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
				sen1State.tempPleath = data;
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
					CBuffWriteByte(&sen1Data.pleathBuff, sen1State.tempPleath);

					#ifndef ALGO_TYPE_PK_DET
						if ((!trigFound) && (GetTrigger(sen1State.tempPleath)))
						{
							/* Write data value to trigger circular buffer */
							CBuffWriteByte(&trigBuff, sen1State.tempPleath);
							/* This flag will be reset in the pressure control loop */
							trigFound = 1;
						}
						else
						{
							/* Write 0 to trigger circular buffer */
							CBuffWriteByte(&trigBuff, 0);
						}
					#else
						if(trigFound)
						{
							GetTrigger(sen1State.tempPleath);
							/* Write 0 to trigger circular buffer */
							CBuffWriteByte(&trigBuff, 0);
						}
						else
						{
							if(GetTrigger(sen1State.tempPleath))
							{
								/* Write data value to trigger circular buffer */
								CBuffWriteByte(&trigBuff, sen1State.tempPleath);	
							}
							else
							{
								/* Write 0 to trigger circular buffer */
								CBuffWriteByte(&trigBuff, 0);
							}
						}
					#endif
					
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
					
					#ifndef ALGO_TYPE_PK_DET
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
					#else
						if(trigFound)
						{
							GetTrigger(data);
							/* Write 0 to trigger circular buffer */
							CBuffWriteByte(&trigBuff, 0);
						}
						else
						{
							if (GetTrigger(data))
							{
								/* Write data value to trigger circular buffer */
								CBuffWriteByte(&trigBuff, data);
							}
							else
							{
								/* Write 0 to trigger circular buffer */
								CBuffWriteByte(&trigBuff, 0);
							}
						}
					#endif
					
					sen1State.pIndex += 5;
				}
				else if(sen1State.ctr == 4)
				{
					sen1Data.hrLsb = data;
				}
				else if(sen1State.ctr == 9)
				{
					sen1Data.spo2 = data;
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
		switch(sen2State.state)
 		{
			case q0:
				if(data==0x01)
				{
					sen2State.state = q1;
					sen2State.sum = 0x01;
				}
				break;
			case q1:
				if((data>127)&&BIT_IS_SET(data,0))
				{
					sen2State.state = q2;
					sen2State.sum += data;
				}
				else
				{
					sen2State.state = q0;
				}
				break;
			case q2:
				//Pleath Reading
				sen2State.tempPleath = data;
				sen2State.state = q3;
				sen2State.sum += data;
				sen2State.pIndex = 3;
				break;
			case q3:
				if(data<127)
				{
					//HRMSB
					//hrtRate = ((uint16_t)(data&0x03))<<8;
					sen2Data.hrMsb = data;
					sen2State.state = q4;
					sen2State.sum += data;
				}
				else
				{
					sen2State.state = q0;
				}
				break;
			case q4:
				/* If Checksum matched */
				if(data==sen2State.sum)
				{
					/* Write data value to pleath circular buffer */
					CBuffWriteByte(&sen2Data.pleathBuff, sen2State.tempPleath);
					sen2State.state = q5;
					sen2State.ctr = 0;
				}
				else
				{
					sen2State.state = q0;
				}
				break;
			case q5:
				sen2State.ctr++;
				if(sen2State.ctr == sen2State.pIndex)
				{
					/* Write data value to pleath circular buffer */
					CBuffWriteByte(&sen2Data.pleathBuff, data);
					sen2State.pIndex += 5;
				}
				else if(sen2State.ctr == 4)
				{
					sen2Data.hrLsb = data;
				}
				else if(sen2State.ctr == 9)
				{
					sen2Data.spo2 = data;
				}
				else if(sen2State.ctr==120)
				{
					sen2State.state = q0;
				}
				break;
 		}
	}
}

uint8_t GetTrigger(uint8_t currPleath)
{
	int8_t diff = (int8_t)prevPleath - (int8_t)currPleath;
	static uint8_t ctr = 0;
	
	#ifndef ALGO_TYPE_PK_DET
		/* Find trigger during rising ppg signal (no peak detect) */
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
	#else
		/* Use peak detection algorithm for trigger calculation */
		if(rising)
		{
			if(diff > 0)
			{
				ctr += 1;
				if(ctr >= PLEATH_DIFF_CYCLES)
				{
					/* This flag will be reset in the pressure control loop */
					if(!trigFound)
						trigFound = 1;
					rising = 0;
					ctr = 0;

					avgCycleTime = (prevCycleTime[0]+prevCycleTime[1]+prevCycleTime[2]+cycleTime)>>2;
					prevCycleTime[0] = prevCycleTime[1];
					prevCycleTime[1] = prevCycleTime[2];
					prevCycleTime[2] = cycleTime;
					cycleTime = 0;
				}
			}
			else
			{
				ctr = 0;
			}
		}
		else
		{
			if(diff <= -PLEATH_DIFF_THRESHOLD)
			{
				ctr += 1;
				if(ctr >= PLEATH_DIFF_CYCLES)
				{
					rising = 1;
					ctr = 0;
				}
			}
			else
			{
				ctr = 0;
			}
		}
	#endif
	
	prevPleath = currPleath;
	return trigFound;
}

void ActivateValves(void)
{
	#ifdef CTRL_TYPE_PRESSURE
		uint16_t temp = 0;
		float pVal = 0;
		uint8_t pBuff[3];
	#endif

	if(trigFound)
	{
		switch(ctrlState)
		{
			#if defined(ALGO_TYPE_PK_DET) 
			case insertDelay:
				tickDur = avgCycleTime - delayParam;
				/* This indicates overflow occurred */
				if(tickDur>10000)
				{
					tickDur = 0;
					trigFound = 0;
				}
				else
				{
					tickCount = 0;
					tickDur = delayParam;
					ctrlState = waitCycles;
					nextCtrlState = fillCuff;
				}
				break;
			#endif
			case fillCuff:
				#ifndef CTRL_TYPE_PRESSURE
					/* If operating in time control mode */
					#ifndef ALGO_TEST_MODE_EN
						SetValveState(s1OpenS2Close);
					#endif
					tickCount = 0;
					tickDur = fillDur;
					ctrlState = waitCycles;
					nextCtrlState = holdCuff;
				#else
					#ifndef ALGO_TEST_MODE_EN
						/* If operating in pressure control mode */
						ReadPressureSen(BOARD_TWI, ADDR_PSEN1, pBuff);
						temp = ((((uint16_t)pBuff[0])<<8)| pBuff[1]);
						pVal = ((float)temp/16383.0f)*PSEN1_MAXP;
						if((pVal>=(pSetPt-0.5f)) && (pVal<=(pSetPt+0.5f)))
						{
							SetValveState(s1CloseS2Close);
							ctrlState = holdCuff;
						}
						else if(pVal<pSetPt)
						{
							SetValveState(s1OpenS2Close);
						}
						else if(pVal>pSetPt)
						{
							SetValveState(s1CloseS2Open);
						}
					#else
						ctrlState = holdCuff;	
					#endif
				#endif
				break;
			case waitCycles:
				if(tickCount>=tickDur)
				{
					ctrlState = nextCtrlState;
					tickCount = 0;
					tickDur = 0;
				}
				break;
			case holdCuff:
				#ifndef ALGO_TEST_MODE_EN
					SetValveState(s1CloseS2Close);
				#endif
				tickCount = 0;
				tickDur = holdDur;
				ctrlState = waitCycles;
				nextCtrlState = releaseCuff;
				break;
			case releaseCuff:
				#ifndef ALGO_TEST_MODE_EN
					SetValveState(s1CloseS2Open);
				#endif
				#ifndef ALGO_TYPE_PK_DET
					ctrlState = fillCuff;
				#else
					ctrlState = insertDelay; 
				#endif
				trigFound = 0;
				break;
			default:
				break;
		}
	}
}

void SetValveState(enum valveStates st)
{
	switch (st)
	{
		case s1CloseS2Close:
			gpio_set_pin_low(PIN_INAVALVE1_IDX);
			gpio_set_pin_low(PIN_INAVALVE2_IDX);
			break;
		case s1CloseS2Open:
			gpio_set_pin_low(PIN_INAVALVE1_IDX);
			gpio_set_pin_high(PIN_INAVALVE2_IDX);
			break;
		case s1OpenS2Close:
			gpio_set_pin_high(PIN_INAVALVE1_IDX);
			gpio_set_pin_low(PIN_INAVALVE2_IDX);
			break;
		case s1OpenS2Open:
			gpio_set_pin_high(PIN_INAVALVE1_IDX);
			gpio_set_pin_high(PIN_INAVALVE2_IDX);
			break;
	}
}

void SendDispData(void)
{
	/* If previous transfer not complete, return */
	#if defined(BOARD_XPLND)
		if (!(uart_get_status(DISP_UART) & UART_SR_ENDTX)) 
	#elif defined(BOARD_NIRA91)
		if (!(usart_get_status(DISP_USART) & US_CSR_ENDTX)) 
	#endif
	{
		return;
	}

	#ifndef CYCLE_AVG_DISP_EN
		uint8_t readS2 = 0;

		if(CBuffGetRxBytes(&sen2Data.pleathBuff) > 25)
		{
			//readS2 = 1;
			#warning "Sensor2 display commented"
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
	#else
		#ifndef ALGO_TYPE_PK_DET
			#warning "Invalid Algorithm selected..."
		#else
			if(dispTick>1000)
			{
				sprintf((char*)dispPkt,"Avg = %lu\r\n",avgCycleTime);
				/* Initiate PDC Transfer to display avg cycle time */
				dispPdcPkt.ul_addr = (uint32_t) dispPkt;
				dispPdcPkt.ul_size = strlen((char*)dispPkt);
				pdc_tx_init(dispUartPdcBase, &dispPdcPkt, NULL);
				dispTick = 0;
			}
		#endif
	#endif
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

void WriteEEPROM(Twi * Port, uint8_t chipAddr, uint8_t memPage, uint8_t *dPkt, uint16_t dLen)
{
	twi_packet_t pkt;
	/* Set Device Address */
	pkt.chip = chipAddr;
	/* Page Number */
	pkt.addr[0] = memPage;
	/* No Address Bytes to be clocked */
	pkt.addr_length = 1;
	/* Address of buffer where recvd data is to be stored */
	pkt.buffer = dPkt;
	/* No of bytes to read */
	pkt.length = dLen;
	twi_master_write(Port, &pkt);
}

void ReadEEPROM(Twi * Port, uint8_t chipAddr, uint8_t memPage, uint8_t *dPkt, uint16_t dLen)
{
	twi_packet_t pkt;
	/* Set Device Address */
	pkt.chip = chipAddr;
	/* Page Number */
	pkt.addr[0] = memPage;
	/* No Address Bytes to be clocked */
	pkt.addr_length = 1;
	/* Address of buffer where recvd data is to be stored */
	pkt.buffer = dPkt;
	/* No of bytes to read */
	pkt.length = dLen;
	twi_master_read(Port, &pkt);
}

void ManageResP(void)
{
	#ifndef ALGO_TEST_MODE_EN
		uint16_t temp;
		uint8_t pBuff[3];

		ReadPressureSen(BOARD_TWI, ADDR_PSEN2, pBuff);
		temp = ((((uint16_t)pBuff[0])<<8)| pBuff[1]);
		resPVal = ((float)temp/16383.0f)*PSEN2_MAXP;
	
		if(resPVal>=MAX_RESERVOIR_P)
		{
			/* Turn off the compressor */
			gpio_set_pin_low(PIN_AIR_PUMP_IDX);		
		}
		else if(resPVal<CUTOFF_RESERVOIR_P)
		{
			/* Turn on the compressor */
			gpio_set_pin_high(PIN_AIR_PUMP_IDX);
		}
	#endif
}

void PollSwitches(void)
{
	#if defined(BOARD_NIRA91) 
		if(gpio_pin_is_low(PIN_SW_PRESS_UP_IDX))
		{
			pSetPt +=PRESSURE_INCR;
			if(pSetPt>MAX_PSET_POINT)
				pSetPt = MAX_PSET_POINT;
		}
		else if(gpio_pin_is_low(PIN_SW_PRESS_DN_IDX))
		{
			pSetPt -=PRESSURE_INCR;
			if(pSetPt<MIN_PSET_POINT)
				pSetPt = MIN_PSET_POINT;
		}
		else if(gpio_pin_is_low(PIN_SW_DURATION_UP_IDX))
		{
			holdDur += HOLD_INCR;
			if(holdDur>MAX_HOLD_DUR)
				holdDur = MAX_HOLD_DUR;
		}
		else if(gpio_pin_is_low(PIN_SW_DURATION_DN_IDX))
		{
			holdDur -= HOLD_INCR;
			if(holdDur>MIN_HOLD_DUR)
				holdDur = MIN_HOLD_DUR;
		}
		else if(gpio_pin_is_low(PIN_SW_DELAY_UP_IDX))
		{
			delayParam+=DELAY_INCR;
			if(delayParam>MAX_DELAY_DUR)
				delayParam = MAX_DELAY_DUR;
		}
		#ifndef BOARD_NIRA91
			/* Delay down pin cannot be utilized as input as this pin is short with DISP UART RX (Schematic mistake) */
			else if(gpio_pin_is_low(PIN_SW_DELAY_DN_IDX))
			{
				delayParam-=DELAY_INCR;	
				if(holdDur>MIN_DELAY_DUR)
					holdDur = MIN_DELAY_DUR;
			}
		#endif
	#endif
}