/*
 * pressureSens.c
 *
 * Created: 16-Dec-17 6:46:59 AM
 *  Author: Mahesh
 */ 

 #include "pressureSens.h"

 void ReadPressureSen(Twi * Port, uint8_t addr, uint8_t *dPkt)
 {
	twi_packet_t pkt;
	/* Set Device Address */
	pkt.chip = addr;
	/* No Address Bytes to be clocked */
	pkt.addr_length = 0;
	/* Address of buffer where recvd data is to be stored */
	pkt.buffer = dPkt;
	/* No of bytes to read */
	pkt.length = 2;
	twi_master_read(Port, &pkt);
	dPkt[0] &= 0x3F;
 }
