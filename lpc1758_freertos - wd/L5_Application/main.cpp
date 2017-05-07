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
#include "stdio.h"
#include "utilities.h"
#include "light_sensor.hpp"
#include "storage.hpp"
#include "event_groups.h"
#include <string.h>
#include "io.hpp"
#include "printf_lib.h"
#include "rtc.h"


SoftTimer writeCPUTimer(60000); //1 minute=60000
SoftTimer consumerTimer(1000);

QueueHandle_t lightQueue = 0;
EventGroupHandle_t lightSensorGroup = 0;
//EventBits_t lightSensorBit = 0;

TaskHandle_t producerHandle = NULL;
TaskHandle_t consumerHandle = NULL;

const uint32_t prodBit = (1 << 0);
const uint32_t consBit = (1 << 1);
const uint32_t task_all_bits = ( prodBit | consBit );

const char *sensorFileStr="1:sensor8.txt";
const char *stuckFileStr="1:stuck8.txt";
const char *cpuFileStr="1:cpu8.txt";




#if 0
void producerTask(void* p)
{
	SoftTimer sensorTimer(100);
	uint16_t lightSensor = 0;
	uint16_t lightSensorSum = 0;
	while(1)
	{
		for(int i = 1; i <= 100; i++)
		{
			lightSensor = LS.getRawValue();
			lightSensorSum =lightSensorSum + lightSensor;
			if(i == 100)
			{
				uint16_t average = lightSensorSum/100;
				//				sensorTimer.reset(100);
				//				while(1)
				//				{
				//					if(sensorTimer.expired())
				//						break;
				//
				//				}
				xQueueSend(lightQueue, &average, portMAX_DELAY);
				lightSensorSum = 0;
			}
			xEventGroupSetBits(lightSensorGroup, prodBit);
		}
	}
}
#endif

#if 0
void consumerTask(void* p)
{
	uint16_t sensorRxavg = 0;
	char buffer[50] = {};
	uint16_t sizeOfData = 0;
	/* Option 1 : C library I/O (less efficient)
	 * 0: is for SPI Flash
	 * 1: is for SD Card
	 */
	FILE* sensorFile = fopen("1:sensor.txt","w+");

	if(sensorFile)
		printf("File sensor.txt created\n");
	else
		printf("creating sensor.txt failed\n");

	while(1)
	{

		if(xQueueReceive(lightQueue, &sensorRxavg, portMAX_DELAY))
		{
			sizeOfData = sprintf(buffer, "%s %u\n",rtc_get_date_time_str(), sensorRxavg);
		}

		if(Storage::append("1:sensor.txt", &buffer,sizeOfData,0) == FR_OK)
		{

		}
		else
		{
			u0_dbg_printf("writing to sensor.txt failed.\n");
		}
		//u0_dbg_printf("bEFORE %d \n",lightSensorBit);

		xEventGroupSetBits(lightSensorGroup, consBit);
		//	fclose(sensorFile);
		//		u0_dbg_printf("After %d \n",lightSensorBit);

		//			vTaskDelay(1000);

	}
}
#endif

//preets info task
void fileWriteCPUInfo(const char* fileName)
{
	char buffer[1024] = {};
	uint16_t bufferSize = 0;
	char newLine[1] = {'\n'};


	const char * const taskStatusTbl[] = { "RUN", "RDY", "BLK", "SUS", "DEL" };
	// Limit the tasks to avoid heap allocation.
	const unsigned portBASE_TYPE maxTasks = 16;
	TaskStatus_t status[maxTasks];
	uint32_t totalRunTime = 0;
	uint32_t tasksRunTime = 0;
	const unsigned portBASE_TYPE uxArraySize = uxTaskGetSystemState(&status[0], maxTasks, &totalRunTime);

	bufferSize = sprintf(buffer,"%10s Sta Pr Stack CPU%%          Time\n", "Name");
	Storage::append(fileName, &buffer,bufferSize,0);
	for(unsigned priorityNum = 0; priorityNum < configMAX_PRIORITIES; priorityNum++)
	{
		/* Print in sorted priority order */
		for (unsigned i = 0; i < uxArraySize; i++)
		{
			TaskStatus_t *e = &status[i];
			if (e->uxBasePriority == priorityNum)
			{
				tasksRunTime += e->ulRunTimeCounter;

				const uint32_t cpuPercent = (0 == totalRunTime) ? 0 : e->ulRunTimeCounter / (totalRunTime/100);
				const uint32_t timeUs = e->ulRunTimeCounter;
				const uint32_t stackInBytes = (4 * e->usStackHighWaterMark);

				bufferSize = sprintf(buffer,"%10s %s %2lu %5lu %4lu %10lu us\n", e->pcTaskName, taskStatusTbl[e->eCurrentState], e->uxBasePriority,stackInBytes, cpuPercent, timeUs);
				Storage::append(fileName, &buffer,bufferSize,0);
				Storage::append(fileName, &newLine,1,0);
			}
		}
	}
}

class producerTask : public scheduler_task
{
public:
	producerTask (uint8_t priority) : scheduler_task("producerTask", 2048, priority)
{
		/* Nothing to init */
}
	bool init(void)
	{
		producerHandle = getTaskHandle();
		return true;
	}
	bool run(void *p)
	{
		SoftTimer sensorTimer(1);
		uint16_t lightSensor = 0;
		uint32_t lightSensorSum = 0;
		uint8_t i=0;
		while(i<=100)
		{
			if(sensorTimer.expired())
			{
				i++;
				lightSensor = LS.getRawValue();
				lightSensorSum =lightSensorSum + lightSensor;
				sensorTimer.reset();
			}

			if(i == 100)
			{
				uint16_t average = lightSensorSum/100;
				//printf(" %d ",average);
				xQueueSend(lightQueue, &average,5);
				lightSensorSum = 0;
				i=0;
			}
			//			u0_dbg_printf(" %d ",prodBit);
			xEventGroupSetBits(lightSensorGroup, prodBit);
		}
		return true;
	}
};

class consumerTask : public scheduler_task
{
public:
	consumerTask (uint8_t priority) : scheduler_task("consumerTask", 2048, priority)
{
		/* Nothing to init */
}
	bool init(void)
	{
		consumerHandle = getTaskHandle();

		return true;
	}
	bool run(void *p)
	{
		FILE* sensorFile = fopen(sensorFileStr,"w+");
		if(!sensorFile)
			printf("creating sensor.txt failed\n");
		uint16_t sensorRxavg;
		char buffer[50] = {};
		uint16_t sizeOfData = 0;
		/* Option 1 : C library I/O (less efficient)
		 * 0: is for SPI Flash
		 * 1: is for SD Card
		 */
		if(xQueueReceive(lightQueue, &sensorRxavg, 5))
		{
			sizeOfData = sprintf(buffer, "%lu %u\n",sys_get_uptime_ms(), sensorRxavg);
			//printf(" %d ",sensorRxavg);
		}

		if(consumerTimer.expired())
		{
			//puts("hey");
			if(Storage::append(sensorFileStr, &buffer,sizeOfData,0) == FR_OK)
			{

			}
			else
			{
				u0_dbg_printf("writing to sensor.txt failed.\n");
			}
			consumerTimer.reset();
		}

		fclose(sensorFile);
		xEventGroupSetBits(lightSensorGroup, consBit);

		return true;
	}
};

class watchdog_task : public scheduler_task
{
public:
	watchdog_task (uint8_t priority) : scheduler_task("watchdog", 3072, priority)
{
		/* Nothing to init */
}
	bool init(void)
	{
		return true;
	}
	bool run(void *p)
	{
		FILE* stuckFile = fopen(stuckFileStr,"w+");
		if(!stuckFile)
			printf("creating stuck.txt failed\n");

		char prodString[] = "New Producer is Stuck\n";
		char consString[] = "New Consumer is Stuck\n";

		//writeCPUTimer.reset();
		//		while(1)
		//		{
		uint32_t lightSensorBit = xEventGroupWaitBits(lightSensorGroup, task_all_bits, pdTRUE, pdTRUE, 2000);

		if((lightSensorBit & task_all_bits) == task_all_bits)
		{
			//					u0_dbg_printf(" %d ",lightSensorBit);
			//					u0_dbg_printf(" Prod-->%d ",prodBit);
			//					u0_dbg_printf(" Cons-->%d ",consBit);
		}
		//		else{
		////		u0_dbg_printf("lightSensorBit--> %d\n",lightSensorBit);
		////		u0_dbg_printf(" Prod-->%d\n",prodBit);
		////		u0_dbg_printf(" Cons-->%d\n",consBit);
		////		u0_dbg_printf(" %d\n",lightSensorBit & prodBit);
		//		}
		if (!(lightSensorBit & prodBit))
		{
		//	u0_dbg_printf("prod is stuck\n");
			//			u0_dbg_printf("lightSensorBit-->%d\n",lightSensorBit);
			//			u0_dbg_printf(" Prod-->%d\n",prodBit);
			//			u0_dbg_printf(" Cons-->%d\n",consBit);

			if(Storage::append(stuckFileStr, &prodString, strlen(prodString),0) == FR_OK)
			{
				//u0_dbg_printf("Appended\n");
			}
			else
			{
				u0_dbg_printf("Failed to write to file(Producer is stuck).\n");
			}
		}
		if(!(lightSensorBit & consBit))
		{
			//u0_dbg_printf("cons is stuck\n");
			//			u0_dbg_printf("lightSensorBit-->%d\n",lightSensorBit);
			//			u0_dbg_printf(" Prod-->%d\n",prodBit);
			//			u0_dbg_printf(" Cons-->%d\n",consBit);

			if(Storage::append(stuckFileStr, &consString, strlen(consString),0) == FR_OK)
			{
				//Good
			}

			else
			{
				printf("consumer is not stuck, but failed to write to file.\n");
			}
		}

		//	puts("Hey");
		//Check timer to see if we need to write CPU info to file
		if(writeCPUTimer.expired())
		{
			FILE* cpuFile = fopen(cpuFileStr,"w+");//write to SD Card
			if(!cpuFile)
				printf("creating cpu.txt failed\n");
			//		u0_dbg_printf("CPU\n");
			fileWriteCPUInfo(cpuFileStr);
			writeCPUTimer.reset();
			fclose(cpuFile);
		}
		fclose(stuckFile);
		//	vTaskDelay(1000);
		return true;
	}
};



#if 0
void watchdogTask(void* p)
{
	SoftTimer writeCPUTimer(60000); //1 minute=60000
	FILE* cpuFile = fopen("1:cpu.txt","w+");//write to SD Card
	FILE* stuckFile = fopen("1:stuck2.txt","w+");
	char prodString[] = "New Producer is Stuck\n";
	char consString[] = "New Consumer is Stuck\n";
	char newLine[1] = {'\n'};


	//Following if/else is just to check if the files were created successfully
	if(!cpuFile)
		printf("creating cpu.txt failed\n");
	else
		printf("created cpu.txt\n");

	if(!stuckFile)
		printf("creating stuck.txt failed\n");
	else
		printf("created stuck.txt\n");

	writeCPUTimer.reset();
	while(1)
	{
		uint32_t lightSensorBit = xEventGroupWaitBits(lightSensorGroup, task_all_bits, pdTRUE, pdTRUE, 2000);

		//puts("hey");
		//get group bits

		//		u0_dbg_printf(" %d ",lightSensorBit);
		//Check if sensor producer bit is set, if not set something went wrong.
		if((lightSensorBit & task_all_bits) == task_all_bits)
		{
			//u0_dbg_printf("Its Working\n");
		}
		else
		{
			if((lightSensorBit & prodBit) == 0)
			{
				u0_dbg_printf("prod is stuck\n");

				//			if(Storage::append("1:stuck2.txt", &prodString, strlen(prodString),0) == FR_OK)
				//			{
				//				//u0_dbg_printf("Appended\n");
				//			}
				//			else
				//			{
				//				u0_dbg_printf("Failed to write to file(Producer is stuck).\n");
				//			}
			}
			else if((lightSensorBit & consBit) == 0)
			{
				u0_dbg_printf("cons is stuck\n");

				//			if(Storage::append("1:stuck.txt", &consString, strlen(consString),0) == FR_OK)
				//			{
				//				//Good
				//			}
				//
				//			else
				//			{
				//				printf("consumer is not stuck, but failed to write to file.\n");
				//			}
			}
		}
		//Check timer to see if we need to write CPU info to file
		if(writeCPUTimer.expired())
		{
			writeCPUUsage("1:cpu.txt");
			Storage::append("1:cpu.txt", &newLine,1,0);
			writeCPUTimer.reset();
		}
		//	vTaskDelay(1000);
	}
}

#endif
CMD_HANDLER_FUNC(taskResumeSuspend)
{
	char *command = NULL;
	char *taskName = NULL;
	if(2 != cmdParams.tokenize(" ", 2, &command, &taskName))
	{
		return false;
	}
	else
	{
		//output.printf("Move '%s' -> '%s' : %s\n",command, taskName);
		if(strcmp(command,"suspend") == 0 && strcmp(taskName,"prod") == 0)
		{
			//suspend producer
			vTaskSuspend(producerHandle);
			printf("Producer Task was suspended!\n");
			return true;
		}
		if(strcmp(command,"suspend") == 0 && strcmp(taskName,"cons") == 0)
		{
			//suspend producer
			vTaskSuspend(consumerHandle);
			printf("Consumer Task was suspended!\n");
			return true;
		}
		if(strcmp(command,"resume") == 0 && strcmp(taskName,"prod") == 0)
		{
			//resume producer
			vTaskResume(producerHandle);
			printf("Producer Task was resumed!\n");
			return true;
		}
		if(strcmp(command,"resume") == 0 && strcmp(taskName,"cons") == 0)
		{
			//resume cons
			vTaskResume(consumerHandle);
			printf("Consumer Task was resumed!\n");
			return true;
		}
	}
	return true;
}


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


	lightQueue = xQueueCreate(5, sizeof(uint16_t));
	lightSensorGroup = xEventGroupCreate();
	//	xTaskCreate(watchdogTask, "watchdog", STACK_BYTES(2048*2),0,PRIORITY_HIGH,0);
	//	xTaskCreate(consumerTask, "consumer", STACK_BYTES(2048), 0,PRIORITY_MEDIUM,&consumerHandle);
	//	xTaskCreate(producerTask, "producer", STACK_BYTES(2048), 0,PRIORITY_MEDIUM,&producerHandle);

	//		scheduler_add_task(new producer(PRIORITY_MEDIUM));
	//		scheduler_add_task(new consumerTask(PRIORITY_MEDIUM));
	producerTask *t1 = new producerTask(PRIORITY_MEDIUM);
	consumerTask *t2 = new consumerTask(PRIORITY_MEDIUM);
	scheduler_add_task(t1);
	scheduler_add_task(t2);

	scheduler_add_task(new watchdog_task(PRIORITY_HIGH));
	producerHandle = t1->getTaskHandle();
	consumerHandle = t2->getTaskHandle();

	scheduler_add_task(new terminalTask(PRIORITY_HIGH));

	/* Consumes very little CPU, but need highest priority to handle mesh network ACKs */
	//	scheduler_add_task(new wirelessTask(PRIORITY_CRITICAL));

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
