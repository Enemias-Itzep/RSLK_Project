/*
 * G8RTOS_Semaphores.c
 */

/*********************************************** Dependencies and Externs *************************************************************/

#include <stdint.h>
#include "msp.h"
#include "G8RTOS_Semaphores.h"
#include "G8RTOS_CriticalSection.h"
#include "G8RTOS_Scheduler.h"
#include "G8RTOS_Structures.h"

/*********************************************** Dependencies and Externs *************************************************************/

extern struct tcb_t* CurrentlyRunningThread;

/*********************************************** Public Functions *********************************************************************/

/*
 * Initializes a semaphore to a given value
 * Param "s": Pointer to semaphore
 * Param "value": Value to initialize semaphore to
 * THIS IS A CRITICAL SECTION
 */
void G8RTOS_InitSemaphore(semaphore_t *s, int32_t value)
{
    int32_t primask_state = StartCriticalSection();

    *s = value;

    EndCriticalSection(primask_state);
}

/*
 * Waits for a semaphore to be available (value greater than 0)
 * 	- Decrements semaphore when available
 * 	- Spinlocks to wait for semaphore
 * Param "s": Pointer to semaphore to wait on
 * THIS IS A CRITICAL SECTION
 */
void G8RTOS_WaitSemaphore(semaphore_t *s)
{
    int32_t primask_state = StartCriticalSection();

    //if semaphore is not available, initialize currentlyrunningthread's blocked pointer to this semaphore
    if (*s <= 0) {
        CurrentlyRunningThread->blocked = s;
        (*s)--;
        EndCriticalSection(primask_state);
        //yield control to the OS until blocked semaphore is available
        OS_Yield();
    }
    else {
        (*s)--;
        EndCriticalSection(primask_state);
    }

}

/*
 * Signals the completion of the usage of a semaphore
 * 	- Increments the semaphore value by 1
 * Param "s": Pointer to semaphore to be signalled
 * THIS IS A CRITICAL SECTION
 */
void G8RTOS_SignalSemaphore(semaphore_t *s)
{
    int32_t primask_state = StartCriticalSection();

    //if *s < 0, go through linked list of TCBs and unblock first thread with that semaphore blocked
    struct tcb_t* nodePtr = CurrentlyRunningThread->next;
    if (*s < 0) {
        while (nodePtr != CurrentlyRunningThread) {
            //check if the blocked pointer of the current node is equal to s
            if (nodePtr->blocked == s) {
                //if it is, unblock nodePtr and break
                nodePtr->blocked = 0;
                break;
            }

            //iterate to the next node in the linked list
            nodePtr = nodePtr->next;
        }
    }
    (*s)++;
    EndCriticalSection(primask_state);
}

/*********************************************** Public Functions *********************************************************************/
