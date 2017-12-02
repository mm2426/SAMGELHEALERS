/*
 * circBuff.h
 *
 * Created: 18-Nov-17 7:07:35 PM
 *  Author: Mahesh
 */ 


#ifndef __CBUFF_H__
#define __CBUFF_H__

#ifndef CBUFF_TYPE
	#define CBUFF_TYPE	uint8_t
#endif

#ifndef CBUFF_PTR_TYPE
	#define CBUFF_PTR_TYPE	uint8_t
#endif

#ifndef CBUFF_SIZE
	#define CBUFF_SIZE 100
#endif

struct cBuff_t
{
	CBUFF_TYPE buff[CBUFF_SIZE];
	CBUFF_PTR_TYPE rPtr, wPtr;
};

uint32_t CBuffGetRxBytes(struct cBuff_t *cbuff);
CBUFF_TYPE CBuffReadByte(struct cBuff_t *cbuff);
void CBuffWriteByte(struct cBuff_t *cbuff, CBUFF_TYPE dByte);

#endif /* CIRCBUFF_H_ */