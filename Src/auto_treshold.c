/*
 * auto_treshold.c
 *
 *  Created on: Feb 8, 2018
 *      Author: kalmargy
 */
#include "auto_treshold.h"

//private variables
static uint16_t currLt;
static uint16_t prevLt;
static uint16_t currHt;
static uint16_t prevHt;
static uint16_t meanSignalValue;

typedef enum ATStates {
	NO_INIT=0,
	WAIT=1,
	EVNT_TIMER=2,
	EVNT_AW=3,
} ATStates;
static ATStates sm;

//private functions
static uint16_t getMean();
static void widenWindow();
static void narrowWindow();

////////////////////
//private functions/
////////////////////
/*
 * brief: Get the average signal value
 * TODO: Calculate mean?
 * */
static uint16_t getMean() {
	return 4096/2;
}


////////////////////
//public functions//
////////////////////
uint8_t init() {
	meanSignalValue = getMean();
	return 0u;
}

uint8_t registerEvent(uint8_t source) {
	return 0u;
}

uint16_t getHt() {
	return 0u;
}

uint16_t getLt() {
	return 0u;
}

