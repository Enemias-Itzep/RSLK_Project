/*
 * G8RTOS_FIFO.c
 *
 *  Created on: Mar 8, 2021
 *      Author: troth
 */

#include "G8RTOS_FIFO.h"

/*  STATIC ARRAY DECLARATIONS   */
static struct fifo_t FIFOs[MAX_FIFO];

/*
 * Initializes a new FIFO
 * Parameter indexes the FIFO which is being initialized
 * Returns error code if index is greater than the maximum number of fifos allowed
 */
int InitFifo(uint32_t index) {
    if (index > MAX_FIFO-1) {
        return 8;
    }
    struct fifo_t* fifo_p = &FIFOs[index];

    //initialize variables
    fifo_p->Head = &fifo_p->Buffer[0];
    fifo_p->Tail = &fifo_p->Buffer[0];
    G8RTOS_InitSemaphore(&fifo_p->CurrentSize, 0); //FIFO starts with size 0
    G8RTOS_InitSemaphore(&fifo_p->Mutex, 1); //FIFO starts not being read from
    fifo_p->LostData = 0; //FIFO starts with no lost data
    return 0;
}

/*
 * Reads data from the FIFO indexed by the parameter
 * returns the head of the FIFO
 * Checks Mutex and current size semaphores
 * updates head pointer and signals mutex semaphore while done reading
 */
int32_t ReadFifo(int32_t index) {
    struct fifo_t* fifo_p = &FIFOs[index];

    //check Mutex semaphore in case FIFO is being read by another thread
    G8RTOS_WaitSemaphore(&fifo_p->Mutex);

    //check current size semaphore in case FIFO is empty
    G8RTOS_WaitSemaphore(&fifo_p->CurrentSize);

    //Read from the FIFO
    int32_t readData = *fifo_p->Head;

    //Update the head pointer
    fifo_p->Head = fifo_p->Head + 1;
    if (fifo_p->Head == &fifo_p->Buffer[0] + FIFOSIZE) {
        //if Head overflows, set back to 0
        fifo_p->Head = &fifo_p->Buffer[0];
    }

    //signals mutex semaphore
    G8RTOS_SignalSemaphore(&fifo_p->Mutex);

    //return read value
    return readData;
}

/*
 * Writes data to the FIFO indexed by the parameter
 * Current size semaphore value should be compared with FIFOSIZE-1
 * Provides one buffer cell in case interrupt happens between reading FIFO and incrementing its head
 * If value is larger than FIFOSIZE-1, increment lost data count and overwrite old data
 * Signal the CurrentSize semaphore
 */
void WriteFifo(int32_t index, int32_t data) {
    //create fifo pointer
    struct fifo_t* fifo_p = &FIFOs[index];

    //compare current size semaphore value to fifosize - 1
    //if the value is larger than fifosize-1, increment lost data count
    if (fifo_p->CurrentSize > FIFOSIZE-1) {
        fifo_p->LostData++;
    }
    else {

        //write data and increment the tail pointer.
        *fifo_p->Tail = data;
        fifo_p->Tail = fifo_p->Tail + 1;
        if (fifo_p->Tail == &fifo_p->Buffer[0] + FIFOSIZE) {
            //if Tail overflows, set back to 0
            fifo_p->Tail = &fifo_p->Buffer[0];
        }

        //signal current size semaphore
        G8RTOS_SignalSemaphore(&fifo_p->CurrentSize);
    }
}

/*
 * waits on mutex semaphore for index parameter
 */
void WaitMutex(int32_t index) {
    G8RTOS_WaitSemaphore(&FIFOs[index].Mutex);
}

/*
 * signals mutex semaphore for index parameter
 */
void SignalMutex(int32_t index) {
    G8RTOS_SignalSemaphore(&FIFOs[index].Mutex);
}

//FIXME: delete later
void FIFO_debug(void) {
    while(1);
}
