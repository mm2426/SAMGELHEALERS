/*
 * testcodes.c
 *
 * Created: 21-Dec-17 7:46:10 PM
 *  Author: Mahesh
 */ 


 //gpio_set_pin_low(PIN_AIR_PUMP_IDX);

//Checks control state machine flow
// 	trigFound = 1;
// 	while(1)
// 	{
// 		ActivateValves();
// 	}

//Packet for Disp uart PDC test
	/*dispPkt[0] = 'H';
	dispPkt[1] = 'e';
	dispPkt[2] = 'l';
	dispPkt[3] = 'l';
	dispPkt[4] = 'o';
	dispPkt[5] = '\r';
	dispPkt[6] = '\n';*/
	
	//Test Pressure sensor and EEPROM
	/*while(1)
	{
		//gpio_set_pin_low(PIO_PA16_IDX);
		
		//Transmit 7 Bytes using PDC
// 		dispPdcPkt.ul_addr = (uint32_t) dispPkt;
// 		dispPdcPkt.ul_size = 7;
// 		pdc_tx_init(dispUartPdcBase, &dispPdcPkt, NULL);

		//ReadPressureSen(BOARD_TWI, ADDR_PSEN1, dispPkt);
		//ReadEEPROM(BOARD_TWI,0x50,0x00,dispPkt,4);
		//WriteEEPROM(BOARD_TWI,0x50,0x00,dispPkt,4);

		//Solenoids and AIR Pump test
		gpio_set_pin_high(PIO_PA16_IDX);
		gpio_set_pin_high(PIN_INAVALVE1_IDX);
		gpio_set_pin_low(PIN_INAVALVE2_IDX);
		gpio_set_pin_high(PIN_AIR_PUMP_IDX);
		delay_ms(5000);
		gpio_set_pin_low(PIN_INAVALVE1_IDX);
		gpio_set_pin_high(PIN_INAVALVE2_IDX);
		gpio_set_pin_low(PIN_AIR_PUMP_IDX);
		gpio_set_pin_low(PIO_PA16_IDX);
		delay_ms(5000);
	}*/

	/*
	//Cuff pressure build up test
	
	SetValveState(s1CloseS2Close);

	//SetValveState(s1CloseS2Open);
	//while(1);

	// Initialize pressure in reservoir
	tickCount = 0;
	gpio_set_pin_high(PIN_AIR_PUMP_IDX);
	//while(1);
	//while (tickCount<=60000);
	while(p2Val<=5.2f)
	//while(1)
	{
		ReadPressureSen(BOARD_TWI, ADDR_PSEN2, dispPkt);
		temp = ((((uint16_t)dispPkt[0])<<8)| dispPkt[1]);
		p2Val = ((float)temp/16383.0f)*PSEN2_MAXP;
		//delay_ms(100);
	}
	// 	gpio_set_pin_low(PIN_AIR_PUMP_IDX);

	SetValveState(s1OpenS2Close);
	tickCount = 0;
	while(p1Val<=4.5f)
	{
		ReadPressureSen(BOARD_TWI, ADDR_PSEN1, dispPkt);
		temp = ((((uint16_t)dispPkt[0])<<8)| dispPkt[1]);
		p1Val = ((float)temp/16383.0f)*PSEN1_MAXP;
	}
	SetValveState(s1CloseS2Close);

	tempCount = tickCount;

	while(1)
	{
		ReadPressureSen(BOARD_TWI, ADDR_PSEN1, dispPkt);
		temp = ((((uint16_t)dispPkt[0])<<8)| dispPkt[1]);
		p1Val = ((float)temp/16383.0f)*PSEN1_MAXP;
	}
	
	while(1)
	{
		ReadPressureSen(BOARD_TWI, ADDR_PSEN2, dispPkt);
		temp = ((((uint16_t)dispPkt[0])<<8)| dispPkt[1]);
		p1Val = ((float)temp/16383.0f)*PSEN2_MAXP;
		delay_ms(100);
	}
	
	*/