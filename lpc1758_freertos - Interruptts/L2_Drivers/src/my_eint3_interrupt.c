/*
 * my_eint3_interrupt.c
 *
 *  Created on: Mar 11, 2017
 *      Author: prash
 */

#include<stdio.h>
#include "printf_lib.h"
#include "my_eint3_interrupt.h"



static void_func_t port2_isr_array[32];
static void_func_t port0_isr_array[32];
uint8_t port2_pin_no[32];
uint8_t port0_pin_no[32];

void main_eint32_interrupt(void)
{
	uint8_t i=0;
	for(i=0;i<32;i++)
	{
		if ((LPC_GPIOINT->IO2IntStatR & (1 << i)))
		{
			port2_isr_array[i]();
			LPC_GPIOINT->IO2IntClr |= (1 << i);
			return;
		}
	}
	for(i=0;i<32;i++)
	{
		if ((LPC_GPIOINT->IO0IntStatR & (1 << i)))
		{
			port0_isr_array[i]();
			LPC_GPIOINT->IO0IntClr |= (1 <<i);
			return;
		}
	}
}

void port2_enable_eint3_interrupt(uint8_t pinno,void_func_t my_func)
{
	LPC_PINCON->PINSEL4 &= ~(3 << (pinno*2));
	LPC_GPIO2->FIODIR |= (0 << pinno);
	LPC_GPIOINT->IO2IntEnR |=(1 << pinno);

	port2_isr_array[pinno]=my_func;
	port2_pin_no[pinno]=pinno;

	NVIC_EnableIRQ(EINT3_IRQn);
	isr_register(EINT3_IRQn, main_eint32_interrupt);
}

void port0_enable_eint3_interrupt(uint8_t pinno,void_func_t my_func)
{
	if(pinno==0 || pinno==1)
	{

		LPC_PINCON->PINSEL0 &= ~(3 << (pinno*2));
		LPC_GPIO0->FIODIR |= (0 << pinno);
		LPC_GPIOINT->IO0IntEnR |=(1 << pinno);
	}
	else
	{
		LPC_PINCON->PINSEL1 &=~(3<<(pinno*2));
		LPC_GPIO0->FIODIR &= ~(1 << pinno);
		LPC_GPIOINT->IO0IntEnR |= (1<<pinno);

	}
	port0_isr_array[pinno]=my_func;
	port0_pin_no[pinno]=pinno;

	NVIC_EnableIRQ(EINT3_IRQn);
	isr_register(EINT3_IRQn, main_eint32_interrupt);
}
void my_eint3_interrupt()
{
	u0_dbg_printf("my_eint3_interrupt\n");
}
