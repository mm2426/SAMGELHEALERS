/*
 * senBuffIOPDC.h
 *
 * Created: 11-Nov-17 2:56:17 PM
 *  Author: Mahesh
 */ 


#ifndef __SENBUFFIOPDC_H__
#define __SENBUFFIOPDC_H__

void SenInitUsart(void);
void SenPdcManageBuff(void);
/* Returns number of bytes in Rx buffer */
uint32_t SenGetRxBytes(uint8_t senNo);
uint8_t SenGetByte(uint8_t senNo);

#endif /* __SENBUFFIOPDC_H__ */