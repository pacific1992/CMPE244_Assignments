/*
 * my_eint3_interrupt.h
 *
 *  Created on: Mar 14, 2017
 *      Author: prash
 */

#ifndef MY_EINT3_INTERRUPT_H_
#define MY_EINT3_INTERRUPT_H_

#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include "lpc_sys.h"

void port2_enable_eint3_interrupt(uint8_t pinno,void_func_t my_func);
void port0_enable_eint3_interrupt(uint8_t pinno,void_func_t my_func);


#ifdef __cplusplus
}
#endif
#endif /* MY_EINT3_INTERRUPT_H_ */
