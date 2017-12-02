/*
 * circBuff.c
 *
 * Created: 18-Nov-17 7:06:28 PM
 *  Author: Mahesh
 */ 

 #include <asf.h>
 #include "cBuff.h"

  uint32_t CBuffGetRxBytes(struct cBuff_t *cbuff)
 {
	uint32_t recvdBytes;

	if(cbuff->wPtr>cbuff->rPtr)
	{
		recvdBytes = (cbuff->wPtr - cbuff->rPtr);
	}
	else if(cbuff->wPtr<cbuff->rPtr)
	{
		recvdBytes = (CBUFF_SIZE - cbuff->rPtr) + cbuff->wPtr;
	}
	else
	{
		recvdBytes = 0;
	}
 }
 
 CBUFF_TYPE CBuffReadByte(struct cBuff_t *cbuff)
 {
	CBUFF_TYPE retVal;

	retVal = cbuff->buff[cbuff->rPtr++];
	if(cbuff->rPtr>(CBUFF_SIZE-1))
	{
		cbuff->rPtr = 0;
	}
	return retVal;
 }
  
 void CBuffWriteByte(struct cBuff_t *cbuff, CBUFF_TYPE dByte)
 {
	cbuff->buff[cbuff->wPtr++] = dByte;
	if(cbuff->wPtr>(CBUFF_SIZE-1))
	{
		cbuff->wPtr = 0;
	}
 }