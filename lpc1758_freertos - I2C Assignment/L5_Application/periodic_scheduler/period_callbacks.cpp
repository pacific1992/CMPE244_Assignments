/*
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */

/**
 * @file
 * This contains the period callback functions for the periodic scheduler
 *
 * @warning
 * These callbacks should be used for hard real-time system, and the priority of these
 * tasks are above everything else in the system (above the PRIORITY_CRITICAL).
 * The period functions SHOULD NEVER block and SHOULD NEVER run over their time slot.
 * For example, the 1000Hz take slot runs periodically every 1ms, and whatever you
 * do must be completed within 1ms.  Running over the time slot will reset the system.
 */

#include <stdint.h>
#include "io.hpp"
#include "periodic_callback.h"
#include "stdio.h"


/// This is the stack size used for each of the period tasks (1Hz, 10Hz, 100Hz, and 1000Hz)
const uint32_t PERIOD_TASKS_STACK_SIZE_BYTES = (512 * 4);

/**
 * This is the stack size of the dispatcher task that triggers the period tasks to run.
 * Minimum 1500 bytes are needed in order to write a debug file if the period tasks overrun.
 * This stack size is also used while calling the period_init() and period_reg_tlm(), and if you use
 * printf inside these functions, you need about 1500 bytes minimum
 */
const uint32_t PERIOD_DISPATCHER_TASK_STACK_SIZE_BYTES = (512 * 3);

/// Called once before the RTOS is started, this is a good place to initialize things once
bool period_init(void)
{
	/* Make direction of PORT2.2 as OUTPUT led pin */
	LPC_GPIO2->FIODIR |= (1 << 2);

	/* Make direction of PORT2.0 as input switch pin */
	LPC_GPIO2->FIODIR &= ~(1 << 0);
//	printf("pin-->%x\n",LPC_GPIO2->FIOPIN);

//	iPin.enablePullDown();
//	 volatile uint32_t *pinmode = &(LPC_PINCON->PINMODE0);
	LPC_PINCON->PINMODE4 = 0x03;


	return true; // Must return true upon success
}

/// Register any telemetry variables
bool period_reg_tlm(void)
{
	// Make sure "SYS_CFG_ENABLE_TLM" is enabled at sys_config.h to use Telemetry
	return true; // Must return true upon success
}



void period_1Hz(void)
{
	if ((LPC_GPIO2->FIOPIN) & 1 << 0) // if switch pressed
	{
		// Switch is logical HIGH
		//		puts("on\n");
		//		printf("%x\n",LPC_GPIO2->FIOPIN);
		LPC_GPIO2->FIOSET |= (1 << 2);
	}
	else
	{
		/* Likewise, reset to 0 */
		LPC_GPIO2->FIOCLR |= (1 << 2);
		//		puts("Off");
	}

	//		bool value = iPin.read(); // Read value of the pin
	//		if(value==true)
	//		{
	//			oPin.setHigh();     // Pin will now be at 3.3v
	//		//	puts("Led on");
	//		}
	//		else
	//		{
	//			oPin.setLow();      // Pin will now be at 0.0v
	//		//	puts("No");
	//		}
	//	//LE.toggle(1);
}

void period_10Hz(void)
{
	//LE.toggle(2);
}

void period_100Hz(void)
{
	//LE.toggle(3);
}

void period_1000Hz(void)
{
	//LE.toggle(4);
}
