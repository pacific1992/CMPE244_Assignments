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
 * @brief This is the application entry point.
 * 			FreeRTOS and stdio printf is pre-configured to use uart0_min.h before main() enters.
 * 			@see L0_LowLevel/lpc_sys.h if you wish to override printf/scanf functions.
 *
 */
#include "tasks.hpp"
#include "examples/examples.hpp"
#include<stdio.h>
#include "printf_lib.h"
#include "string.h"
#include "io.hpp"
#include "utilities.h"
/**
 * The main() creates tasks or "threads".  See the documentation of scheduler_task class at scheduler_task.hpp
 * for details.  There is a very simple example towards the beginning of this class's declaration.
 *
 * @warning SPI #1 bus usage notes (interfaced to SD & Flash):
 *      - You can read/write files from multiple tasks because it automatically goes through SPI semaphore.
 *      - If you are going to use the SPI Bus in a FreeRTOS task, you need to use the API at L4_IO/fat/spi_sem.h
 *
 * @warning SPI #0 usage notes (Nordic wireless)
 *      - This bus is more tricky to use because if FreeRTOS is not running, the RIT interrupt may use the bus.
 *      - If FreeRTOS is running, then wireless task may use it.
 *        In either case, you should avoid using this bus or interfacing to external components because
 *        there is no semaphore configured for this bus and it should be used exclusively by nordic wireless.
 */



QueueHandle_t queue1;
char rx_data;

//
///**************************** UART-2 Overidden Interrupt Handler ******************/
//extern "C"
//{
//void UART2_Interrupt(void)
//{
//	if((LPC_UART2->LSR & 1<<0))
//	{
//		//		puts("Interrupt time");
//		BaseType_t higher_priority_task_woken;
//		higher_priority_task_woken = pdFALSE;
//		rx_data = LPC_UART2 -> RBR;
//		xQueueSendToBackFromISR(queue1,&rx_data,&higher_priority_task_woken);
//		if(higher_priority_task_woken)
//		{
//			puts("Interrupt time");
//			portYIELD_FROM_ISR(0);
//		}
//	}
//}
//}


/**************************** UART-3 Overidden Interrupt Handler ******************/
extern "C"
{
void UART3_Interrupt(void)
{
	if((LPC_UART3->LSR & 1<<0))
	{
		//		puts("Interrupt time");
		BaseType_t higher_priority_task_woken;
		higher_priority_task_woken = pdFALSE;
		rx_data = LPC_UART3 -> RBR;
		xQueueSendToBackFromISR(queue1,&rx_data,&higher_priority_task_woken);

		{
	}
}
}


void UART2_TX_init(void)
{
	const uint32_t baud2=9600;
	const uint8_t uart2_power_control_bit = 24;
	int dll2;

	//Power up UART2
	LPC_SC->PCONP |= 1 << uart2_power_control_bit;

	//PCLKSEL for UART2
	LPC_SC->PCLKSEL1 &= ~(3 << 16);
	LPC_SC->PCLKSEL1 |= 1 << 16;

	//set DLAB to 1
	LPC_UART2->LCR = 1 << 7;

	//DLL value
	dll2=sys_get_cpu_clock() / (16 * baud2);
	LPC_UART2->DLL = dll2 & 0xFF;
	LPC_UART2->DLM = dll2>>8;

	//TXD2 & RXD2
	LPC_PINCON->PINSEL4 &= ~(0xF << 16);
	LPC_PINCON->PINSEL4 |= (0xA << 16);

	//Set 8 bit char length
	LPC_UART2->LCR |= (3<<0);

	//Make DLAB 0 to read and write data to UART
	LPC_UART2->LCR &= ~(1<<7);
}

void UART3_RX_init(void)
{
	const uint32_t baud3=9600;
	const uint8_t uart3_power_control_bit = 25;
	int dll3;
	//powering up UART3
	LPC_SC->PCONP |= 1 << uart3_power_control_bit;

	//PCLKSEL for UART3
	LPC_SC->PCLKSEL1 &= ~(3 << 18);
	LPC_SC->PCLKSEL1 |= 1 << 18;

	//set dlab to 1
	LPC_UART3->LCR = 1 << 7;

	//DLL value
	dll3=sys_get_cpu_clock() / (16 * baud3);
	LPC_UART3->DLL = dll3 & 0xFF;// & 0xFF; //Set LSB for baud
	LPC_UART3->DLM = dll3>>8;// dll3>>16; //Set MSB for baud


	//TXD3 & RXD3
	LPC_PINCON->PINSEL0 &= ~(0xF << 0);
	LPC_PINCON->PINSEL0 |= (0xA << 0);


	//Set 8 bit char length
	LPC_UART3->LCR |= (3<<0);

	//Make DLAB 0 to read and write data to UART
	LPC_UART3->LCR &= ~(1<<7);

	// To enable NVIC interrupt
	NVIC_EnableIRQ(UART3_IRQn);

	LPC_UART3->IER |= 1<<0;        // Enable the RDA interrupts.
	//	LPC_UART3->IER |= 1<<2;        // Enable the RX line status interrupts.

}

/***UART 2 TX Method***/
void u2_putchar(char c1)
{

	LPC_UART2->THR=c1;
	while(1)
	{
		//		puts("putchar while");

		if(LPC_UART2->LSR & (1<<5))
		{
			//		puts("puthar break");
			break;
		}
	}
	//	u0_dbg_printf(" %c ",c1);

}
/*
 * UART 3 RX Method
char u3_getchar(void)
{
	while(1)
	{
		if(LPC_UART3->LSR & (1<<0))
		{
			//	puts("getchar gonna break");
			break;
		}
	}
	char c3=LPC_UART3->RBR;

	return c3;
}
 */


class UARTTask: public scheduler_task
{
public:
	UARTTask(uint8_t priority) :scheduler_task("UARTTask", 2000, priority)
{

}

	bool init(void){
		//Create a queue of 5 characters
		queue1 = xQueueCreate(5, 1*sizeof(char));

		//	isr_register(UART2_IRQn, UART2_Interrupt);
		isr_register(UART3_IRQn, UART3_Interrupt);

		//Initialize the UART2 TX and UART3 RX registers
		UART2_TX_init();
		UART3_RX_init();

		return true;
	}

	bool run(void *p)
	{

		//				u2_putchar('A');
		//				char ch=u3_getchar();
		//				vTaskDelay(1000);
		//
		//				printf(" %c ",ch);
		//
		//				puts("i have work to do");
		//				return true;

		char ch='@';
		if(xQueueReceiveFromISR(queue1,&ch,0))
		{
			// Received data
			u0_dbg_printf(" %c ",ch);
		}
		u2_putchar(++ch);

		vTaskDelay(1000);
		u0_dbg_printf("Printing\n");
		return true;
	}
};


int main(void)
{
	/**
	 * A few basic tasks for this bare-bone system :
	 *      1.  Terminal task provides gateway to interact with the board through UART terminal.
	 *      2.  Remote task allows you to use remote control to interact with the board.
	 *      3.  Wireless task responsible to receive, retry, and handle mesh network.
	 *
	 * Disable remote task if you are not using it.  Also, it needs SYS_CFG_ENABLE_TLM
	 * such that it can save remote control codes to non-volatile memory.  IR remote
	 * control codes can be learned by typing the "learn" terminal command.
	 */
#if 1
	scheduler_add_task(new UARTTask(PRIORITY_HIGH));
#endif
	//	scheduler_add_task(new terminalTask(PRIORITY_HIGH));

	/* Consumes very little CPU, but need highest priority to handle mesh network ACKs */
	scheduler_add_task(new wirelessTask(PRIORITY_CRITICAL));

	/* Change "#if 0" to "#if 1" to run period tasks; @see period_callbacks.cpp */
#if 0
	scheduler_add_task(new periodicSchedulerTask());
#endif

	/* The task for the IR receiver */
	// scheduler_add_task(new remoteTask  (PRIORITY_LOW));

	/* Your tasks should probably used PRIORITY_MEDIUM or PRIORITY_LOW because you want the terminal
	 * task to always be responsive so you can poke around in case something goes wrong.
	 */

	/**
	 * This is a the board demonstration task that can be used to test the board.
	 * This also shows you how to send a wireless packets to other boards.
	 */
#if 0
	scheduler_add_task(new example_io_demo());
#endif

	/**
	 * Change "#if 0" to "#if 1" to enable examples.
	 * Try these examples one at a time.
	 */
#if 0
	scheduler_add_task(new example_task());
	scheduler_add_task(new example_alarm());
	scheduler_add_task(new example_logger_qset());
	scheduler_add_task(new example_nv_vars());
#endif

	/**
	 * Try the rx / tx tasks together to see how they queue data to each other.
	 */
#if 0
	scheduler_add_task(new queue_tx());
	scheduler_add_task(new queue_rx());
#endif

	/**
	 * Another example of shared handles and producer/consumer using a queue.
	 * In this example, producer will produce as fast as the consumer can consume.
	 */
#if 0
	scheduler_add_task(new producer());
	scheduler_add_task(new consumer());
#endif

	/**
	 * If you have RN-XV on your board, you can connect to Wifi using this task.
	 * This does two things for us:
	 *   1.  The task allows us to perform HTTP web requests (@see wifiTask)
	 *   2.  Terminal task can accept commands from TCP/IP through Wifly module.
	 *
	 * To add terminal command channel, add this at terminal.cpp :: taskEntry() function:
	 * @code
	 *     // Assuming Wifly is on Uart3
	 *     addCommandChannel(Uart3::getInstance(), false);
	 * @endcode
	 */
#if 0
	Uart3 &u3 = Uart3::getInstance();
	u3.init(WIFI_BAUD_RATE, WIFI_RXQ_SIZE, WIFI_TXQ_SIZE);
	scheduler_add_task(new wifiTask(Uart3::getInstance(), PRIORITY_LOW));
#endif

	scheduler_start(); ///< This shouldn't return
	return -1;
}
