/*
 * consts.h
 *
 *  Created on: Feb 8, 2018
 *      Author: kalmargy
 */

#ifndef CONSTS_H_
#define CONSTS_H_

#define USE_SD_CARD 1
#define FATFS_MKFS_ALLOWED 1

#define numOfMicSamples 32*1024u
#define elementLength 2u
#define bufferSize (elementLength*numOfMicSamples) //bytes
#define numOfBufferWrites (5u*2u) //every minute -> 5*2*16KB
#define micFileSize (numOfBufferWrites*bufferSize/2u) //bytes

#define numOfAccelSamples 512
#define accelFileSize (numOfAccelSamples*elementLength*3u) //bytes: numOfAccelSamples*2B*[x,y,z]

/*
 * auto treshold consts
 */
//change granularity
#define CHANGE_GRAN 500
#define WINDOW_MIN_WIDTH 200
#define MIN_VALUE 0
#define MAX_VALUE 4095

#endif /* CONSTS_H_ */
