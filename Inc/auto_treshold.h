/*
 * auto_treshold.h
 *
 *  Created on: Feb 8, 2018
 *      Author: kalmargy
 */

#ifndef AUTO_TRESHOLD_H_
#define AUTO_TRESHOLD_H_

//includes
#include <stdint.h>
#include "consts.h"

//public variables
extern uint16_t adcBuffer[numOfMicSamples];
enum{
	EVNTSRC_TIMER=1,
	EVNTSRC_AW=2,
};

//public functions
uint8_t init();
uint8_t registerEvent(uint8_t source);
uint16_t getHt();
uint16_t getLt();

#endif /* AUTO_TRESHOLD_H_ */
