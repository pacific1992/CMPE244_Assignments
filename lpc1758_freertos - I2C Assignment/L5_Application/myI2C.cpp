/*
 * myI2C.cpp
 *
 *  Created on: Apr 17, 2017
 *      Author: prash
 */

#include "stdint.h"
#include "LPC17xx.h"

void slaveMemoryLEDInit()
{
    uint32_t led = 0b1 << 0;
    LPC_GPIO0->FIODIR |= led;
}

uint32_t uint8Length(const uint8_t * cstring)
{
    int count = 0;
    for(count = 0; cstring[count] != 0; count++)
    {
	//keep countttinnnn.
    }
    return count;
}
