/*
 * G8RTOS_FIFO.h
 *
 *  Created on: Mar 8, 2021
 *      Author: troth
 */

#ifndef G8RTOS_G8RTOS_FIFO_H_
#define G8RTOS_G8RTOS_FIFO_H_

#include <stdint.h>
#include "G8RTOS.h"
#include "G8RTOS_Semaphores.h"

/*  CONSTANT DEFINITIONS        */
#define MAX_FIFO    4
#define FIFOSIZE    16

/*  DATA STRUCTURE DEFINITION   */
struct fifo_t {
    int32_t Buffer[FIFOSIZE];
    int32_t* Head;
    int32_t* Tail;
    uint32_t LostData;
    semaphore_t CurrentSize;
    semaphore_t Mutex;
};

/*  FIFO functions              */

/*
 * Initializes a new FIFO
 * Parameter indexes the FIFO which is being initialized
 * Returns error code if index is greater than the maximum number of fifos allowed
 */
int InitFifo(uint32_t index);

/*
 * Reads data from the FIFO indexed by the parameter
 * returns the head of the FIFO
 * Checks Mutex and current size semaphores
 * updates head pointer and signals mutex semaphore while done reading
 */
int32_t ReadFifo(int32_t index);

/*
 * Writes data to the FIFO indexed by the parameter
 * Current size semaphore value should be compared with FIFOSIZE-1
 * Provides one buffer cell in case interrupt happens between reading FIFO and incrementing its head
 * If value is larger than FIFOSIZE-1, increment lost data count and overwrite old data
 * Signal the CurrentSize semaphore
 */
void WriteFifo(int32_t index, int32_t data);

/*
 * waits on mutex semaphore for index parameter
 */
void WaitMutex(int32_t index);

/*
 * signals mutex semaphore for index parameter
 */
void SignalMutex(int32_t index);

//FIXME: delete later
void FIFO_debug(void);

#endif /* G8RTOS_G8RTOS_FIFO_H_ */
