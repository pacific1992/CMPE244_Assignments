/*
 * eint3_new.cpp
 *
 *  Created on: Mar 14, 2017
 *      Author: prash
 */


#include "eint3_new.hpp"
#include <stdio.h>
#include "LPC17xx.h"
//#include "startup.cpp"



extern "C"
    {
        void external_interrupt_handler()
        {
            /* Your ISR */
        	// set mode
        	// activate run loop
        }
    }

    void callback1() {

    }

    void callback2() {

    }



	EINTMode eintMode;

	testEINT::testEINT() : scheduler_task("test_EINT", 4 * 512, PRIORITY_LOW)
 {

	}

    bool testEINT::init(void) {

    	return true;
    }

    bool testEINT::run(void *p) {
    	// task is activated on interrupt
    	// gets called with a slow enough delay for a second switch signal not to mess it up


    	return true;
    }

    void testEINT::enable_external_interrupts() {

    //	g_isr_array[EINT3_IRQn] = &external_interrupt_handler; // set handler
    }

    void testEINT::disable_external_interrupt() {

    }

    void testEINT::setMode(EINTMode mode) {

    }
