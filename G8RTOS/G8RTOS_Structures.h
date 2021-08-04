/*
 * G8RTOS_Structure.h
 *
 *  Created on: Jan 12, 2017
 *      Author: Raz Aloni
 */

#ifndef G8RTOS_STRUCTURES_H_
#define G8RTOS_STRUCTURES_H_

#include "G8RTOS.h"
#include <stdbool.h>

/*********************************************** Data Structure Definitions ***********************************************************/

/*
 *  Thread Control Block:
 *      - Every thread has a Thread Control Block
 *      - The Thread Control Block holds information about the Thread Such as the Stack Pointer, Priority Level, and Blocked Status
 *      - For Lab 2 the TCB will only hold the Stack Pointer, next TCB and the previous TCB (for Round Robin Scheduling)
 */

/* Create tcb struct here */
struct tcb_t {
    int32_t* sp; //holds the stack pointer for the thread
    struct tcb_t* next; //holds the pointer for the next thread to schedule
    struct tcb_t* prev; //holds the pointer to the previous thread in the scheduling

    semaphore_t* blocked;
    bool alive;
    bool asleep;
    uint8_t priority;
    uint32_t sleepCount;
    threadId_t threadID;
    char threadName[MAX_NAME_LENGTH];
};

/* Create Periodic Event struct here */
struct pe_t {
    void (*Handler) (void);
    uint32_t period;
    uint32_t executeTime;
    struct pe_t* next;
    struct pe_t* prev;
};

/*********************************************** Data Structure Definitions ***********************************************************/


/*********************************************** Public Variables *********************************************************************/

struct tcb_t * CurrentlyRunningThread;

/*********************************************** Public Variables *********************************************************************/




#endif /* G8RTOS_STRUCTURES_H_ */
