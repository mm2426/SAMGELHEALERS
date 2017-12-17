/*
 * pressureSens.h
 *
 * Created: 16-Dec-17 6:44:04 AM
 *  Author: Mahesh
 */ 


#ifndef __PRESSURESENS_H__
#define __PRESSURESENS_H__

#include <asf.h>
#include "user_board.h"

#define ADDR_PSEN1	0x28
#define ADDR_PSEN2	0x48

void ReadPressureSen(Twi * Port, uint8_t addr, uint8_t *dPkt);

#endif /* __PRESSURESENS_H__ */