/*
 * senBuffIOPDC.c
 *
 * Created: 11-Nov-17 3:01:07 PM
 *  Author: Mahesh
 */ 

 #include <asf.h>
 #include "senBuffIOPDC.h"

 /* Buffers to hold received data from sensor1 and sensor 2*/
 uint8_t sen1Buff[SEN_USART_BUFF_SIZE]={};//, sen1Buff2[SEN_USART_BUFF_SIZE]={};
 uint8_t sen2Buff[SEN_USART_BUFF_SIZE]={};//, sen2Buff2[SEN_USART_BUFF_SIZE]={};

 /* These pointers point to the buffer which is to be used for processing. */
 //uint8_t *sen1Bptr, *sen2Bptr;
 /* Read and write pointers, these index the buffers */
 uint8_t sen1Rptr = 0, sen2Rptr = 0, sen1Wptr = 0, sen2Wptr = 0;
 /* These flags indicate whether buff1/2 is in use by PDC currently */
 //uint8_t sen1BFlag = 0, sen2BFlag = 0;

 /* PDC data packet. */
 pdc_packet_t pdcPkt;// pdcPkt2;
 /* Pointer to PDC register base. */
 Pdc *sen1PdcBase, *sen2PdcBase;

 void SenInitUsart(void)
 {
	 const sam_usart_opt_t usart_console_settings = {
		 SEN1_BAUDRATE,
		 US_MR_CHRL_8_BIT,
		 US_MR_PAR_NO,
		 US_MR_NBSTOP_1_BIT,
		 US_MR_CHMODE_NORMAL,
		 /* This field is only used in IrDA mode. */
		 0
	 };

	 /* Enable the peripheral clock in the PMC. */
	 sysclk_enable_peripheral_clock(SEN1_USART_ID);

	 /* Configure USART in RS485 mode. */
	 usart_init_rs232(SEN1_USART, &usart_console_settings,
	 sysclk_get_peripheral_hz());

	 // 	/* Enable the peripheral clock in the PMC. */
	 // 	sysclk_enable_peripheral_clock(SEN2_USART_ID);
	 //
	 // 	/* Configure USART in RS485 mode. */
	 // 	usart_init_rs232(SEN2_USART, &usart_console_settings,
	 // 	sysclk_get_peripheral_hz());

	 /* Enable RX function. */
	 usart_disable_tx(SEN1_USART);
	 usart_enable_rx(SEN1_USART);
// 	 usart_disable_tx(SEN2_USART);
// 	 usart_enable_rx(SEN2_USART);

	/* Get board USART PDC base address and enable receiver and transmitter. */
	sen1PdcBase = usart_get_pdc_base(SEN1_USART);
	pdc_enable_transfer(sen1PdcBase, PERIPH_PTCR_RXTEN);

// 	sen2PdcBase = usart_get_pdc_base(SEN2_USART);
// 	pdc_enable_transfer(sen2PdcBase, PERIPH_PTCR_RXTEN);
	
	pdcPkt.ul_addr = (uint32_t) sen1Buff;
	pdcPkt.ul_size = SEN_USART_BUFF_SIZE;
	//For circular buffer operation
	pdc_rx_init(sen1PdcBase, &pdcPkt, &pdcPkt);

// 	pdcPkt1.ul_addr = (uint32_t) sen2Buff1;
// 	pdcPkt1.ul_size = SEN_USART_BUFF_SIZE;
// 	pdcPkt2.ul_addr = (uint32_t) sen2Buff2;
// 	pdcPkt2.ul_size = SEN_USART_BUFF_SIZE;
// 	pdc_rx_init(sen2PdcBase, &pdcPkt1, &pdcPkt2);

 }

 void SenPdcManageBuff(void)
 {
	/* If PDC receive next pointer is 0 */
	if(pdc_read_rx_next_counter(sen1PdcBase)==0)
	{
		/* If code reaches here it means current buffer is full and 
		next buffer ptr is assigned to current buffer ptr by PDC. */
		pdcPkt.ul_addr = (uint32_t) sen1Buff;
		pdcPkt.ul_size = SEN_USART_BUFF_SIZE;
		//For circular buffer operation infinitely
		pdc_rx_init(sen1PdcBase, NULL, &pdcPkt);
	}
	sen1Wptr = SEN_USART_BUFF_SIZE - pdc_read_rx_counter(sen1PdcBase);

	/* If PDC receive next pointer is 0 */
// 	if(pdc_read_rx_next_counter(sen2PdcBase)==0)
// 	{
// 		/* If code reaches here it means current buffer is full and 
// 		next buffer ptr is assigned to current buffer ptr by PDC. */
// 		pdcPkt.ul_addr = (uint32_t) sen2Buff;
// 		pdcPkt.ul_size = SEN_USART_BUFF_SIZE;
// 		//For circular buffer operation infinitely
// 		pdc_rx_init(sen2PdcBase, NULL, &pdcPkt);
// 	}
// 	sen2Wptr = SEN_USART_BUFF_SIZE - pdc_read_rx_counter(sen2PdcBase);
 }
 
 /* Returns number of bytes in Rx buffer */
 uint32_t SenGetRxBytes(uint8_t senNo)
 {
	uint32_t recvdBytes;
	if(senNo == 1)
	{
		if(sen1Wptr>sen1Rptr)
		{
			recvdBytes = sen1Wptr;
		}
		else if(sen1Wptr<sen1Rptr)
		{
			recvdBytes = (SEN_USART_BUFF_SIZE - sen1Rptr) + sen1Wptr;
		}
		else
		{
			recvdBytes = 0;
		}
	}
	else
	{
		if(sen2Wptr>sen2Rptr)
		{
			recvdBytes = sen2Wptr;
		}
		else if(sen2Wptr<sen2Rptr)
		{
			recvdBytes = (SEN_USART_BUFF_SIZE - sen2Rptr) + sen2Wptr;
		}
		else
		{
			recvdBytes = 0;
		}
	}
	
	return recvdBytes;
 }

 uint8_t SenGetByte(uint8_t senNo)
 {
	uint8_t dataByte;
	if(senNo==1)
	{
		dataByte = sen1Buff[sen1Rptr++];
		if(sen1Rptr>=SEN_USART_BUFF_SIZE)
			sen1Rptr = 0;
	}
	else
	{
		dataByte = sen2Buff[sen2Rptr++];
		if(sen2Rptr>=SEN_USART_BUFF_SIZE)
			sen2Rptr = 0;
	}
	return dataByte;
 }