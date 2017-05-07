/*
 * eint3_new.hpp
 *
 *  Created on: Mar 14, 2017
 *      Author: prash
 */

#ifndef L5_APPLICATION_EINT3_NEW_HPP_
#define L5_APPLICATION_EINT3_NEW_HPP_


#include "scheduler_task.hpp"
#include "stdint.h"
#include "str.hpp"

enum EINTMode {
	OFF_MODE,
	ON_MODE,
};

class testEINT : public scheduler_task {
private:
	static EINTMode currMode;
	static EINTMode prevMode;

public:
    testEINT();
    bool init(void);
    bool run(void *p);
    void enable_external_interrupts();
    void disable_external_interrupt();
    static void setMode(EINTMode mode);
};


#endif /* L5_APPLICATION_EINT3_NEW_HPP_ */
