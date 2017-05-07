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
#include "io.hpp"
#include "handlers.hpp"
#include "utilities.h"
#include "printf_lib.h"
#include "wireless.h"
//#include "src/mesh.h"

void ping_and_set_led(uint8_t addr)
{
	/* Sending NULL packet is a "PING" packet.
	 * No special code is required at the other node since the
	 * other node will automatically send the ACK back.
	 */
	const char max_hops = 2;
	mesh_packet_t pkt;
	mesh_send(addr, mesh_pkt_ack, NULL, 0, max_hops);

	/* Turn LED on or off based on if we get ACK packet within 100ms */
	if (wireless_get_ack_pkt(&pkt, 100)) {
		LE.on(1);
		u0_dbg_printf("Got it");
	}
	else {
		LE.off(1);
		u0_dbg_printf("No");

	}
}

class txTask : public scheduler_task
{
public:
	txTask (uint8_t priority) : scheduler_task("txTask", 2048, priority)
{
		/* Nothing to init */
}
	bool init(void)
	{
		return true;
	}
	bool run(void *p)
	{
		char hops=1;
		char addr=107;
		mesh_packet_t pkt;

		//wireless_send(addr,mesh_pkt_nack,"hello",5,hops);
		//wireless_send(MESH_BROADCAST_ADDR,mesh_pkt_nack,"hello",5,hops);
		//	wireless_send(addr,mesh_pkt_ack,"HELLO",5,hops);
		//
		int var1=10;
		float var2=10.098;
		wireless_form_pkt(&pkt,addr,mesh_pkt_ack,hops,2,&var1,sizeof(var1),&var2,sizeof(var2));
		wireless_send_formed_pkt(&pkt);
		if(wireless_get_ack_pkt(&pkt,1000))
		{
			u0_dbg_printf("Got ack\n");
		}
		//wireless_flush_rx();
		return true;
	}
};

class rxTask : public scheduler_task
{
public:
	rxTask (uint8_t priority) : scheduler_task("rxTask", 2048, priority)
{
		/* Nothing to init */
}
	bool init(void)
	{

		return true;
	}
	bool run(void *p)
	{
		u0_dbg_printf(" . ");
		mesh_packet_t pkt;
		char var1=0;
		float  var2=0;
		if(wireless_get_rx_pkt(&pkt,100))
		{
			wireless_deform_pkt(&pkt,2,&var1,sizeof(var1),&var2,sizeof(var2));
			u0_dbg_printf("%c\n",var1);
			u0_dbg_printf("%f\n",var2);
		}

		return true;
	}
};

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

void receiver(void)
{
	mesh_packet_t pkt;
	int var1=0;
	float  var2=0;
	while(1)
	{
		if(wireless_get_rx_pkt(&pkt,100))
		{
			wireless_deform_pkt(&pkt,2,&var1,sizeof(var1),&var2,sizeof(var2));
			u0_dbg_printf("%d\n",var1);
			u0_dbg_printf("%f\n",var2);
		}
	}
}

void sender(void)
{
	u0_dbg_printf(" . ");
	char hops=1;
	char addr=107;
	mesh_packet_t pkt;

	//wireless_send(addr,mesh_pkt_nack,"hello",5,hops);
	//wireless_send(MESH_BROADCAST_ADDR,mesh_pkt_nack,"hello",5,hops);
	//	wireless_send(addr,mesh_pkt_ack,"HELLO",5,hops);
	//

	int var1=10;
	float var2=10.098;
	wireless_form_pkt(&pkt,addr,mesh_pkt_ack,hops,2,&var1,sizeof(var1),&var2,sizeof(var2));
	wireless_send_formed_pkt(&pkt);
	if(wireless_get_ack_pkt(&pkt,1000))
	{
		u0_dbg_printf("Got ack\n");
	}
	//wireless_flush_rx();
}

int main(void)
{


	//txTask *t1 = new txTask(PRIORITY_MEDIUM);
	//	consumerTask *t2 = new consumerTask(PRIORITY_MEDIUM);
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
	//	scheduler_add_task(new txTask(PRIORITY_HIGH));
	scheduler_add_task(new rxTask(PRIORITY_HIGH));

	//		scheduler_add_task(new terminalTask(PRIORITY_HIGH));

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
