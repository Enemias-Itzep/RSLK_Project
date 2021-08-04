/*
 * G8RTOS_Scheduler.h
 */

#ifndef G8RTOS_SCHEDULER_H_
#define G8RTOS_SCHEDULER_H_

#include "BSP.h"
#include "msp432p401r.h"

/*********************************************** Sizes and Limits *********************************************************************/
#define MAX_THREADS 30
#define STACKSIZE 256
#define MAX_PeriodicEvent 6
#define OSINT_PRIORITY 7
#define MAX_NAME_LENGTH 16

#define threadId_t uint32_t

/*********************************************** Sizes and Limits *********************************************************************/

/*********************************************** Error Codes *********************************************************************/

typedef enum
{
    NO_ERROR                    = 0, //returned when no error occurs
    THREAD_LIMIT_REACHED        = -1,
    NO_THREADS_SCHEDULED        = -2,
    THREADS_INCORRECTLY_ALIVE   = -3, //returned from addthread when no TCBs are dead despite number of threads not being at max
    THREAD_DOES_NOT_EXIST       = -4, //returned from kill thread when the system cannot find the thread with the given ID
    CANNOT_KILL_LAST_THREAD     = -5, //returned from kill thread when attempting to kill the last running thread
    IRQn_INVALID                = -6,
    HWI_PRIORITY_INVALID        = -7
} sched_ErrCode_t;

/*********************************************** Error Codes *********************************************************************/

/*********************************************** Public Variables *********************************************************************/

/* Holds the current time for the whole System */
extern uint32_t SystemTime;

/*********************************************** Public Variables *********************************************************************/


/*********************************************** Public Functions *********************************************************************/

/*
 * Initializes variables and hardware for G8RTOS usage
 */
void G8RTOS_Init();

/*
 * Starts G8RTOS Scheduler
 * 	- Initializes Systick Timer
 * 	- Sets Context to first thread
 * Returns: Error Code for starting scheduler. This will only return if the scheduler fails
 */
int32_t G8RTOS_Launch();

/*
 * Adds threads to G8RTOS Scheduler
 * 	- Checks if there are stil available threads to insert to scheduler
 * 	- Initializes the thread control block for the provided thread
 * 	- Initializes the stack for the provided thread
 * 	- Sets up the next and previous tcb pointers in a round robin fashion
 * Param "threadToAdd": Void-Void Function to add as preemptable main thread
 * Returns: Error code for adding threads
 */
int32_t G8RTOS_AddThread(void (*threadToAdd)(void), uint8_t priority, char* name);

/* Kill's the thread indicated by this threadID passed into the function
 *   -enters a critical section
 *   -returns error code if only one thread running
 *   -search for thread with the same thread ID
 *   -return error code if the thread does not exist
 *   -set thread's alive bit to false
 *   -update thread pointers
 *   -Decrement number of threads
 *   -end critical section. If thread being killed is currently running thread, context switch
 *    */
sched_ErrCode_t G8RTOS_KillThread(threadId_t threadId);

/* Kills the CurrentlyRunningThread
 *  -enters a critical section
 *  -return CANNOT_KILL_LAST_THREAD if only one thread running
 *  -change alive to false
 *  -update thread pointers
 *  -start context switch
 *  -decrement number of threads
 *  -end critical section
 */
sched_ErrCode_t G8RTOS_KillSelf();

/*
* Kills all threads in the scheduler other than CurrentlyRunningThread
* returns if G8RTOS_KillThread returns an error, stop the process and return that error code
*/
sched_ErrCode_t G8RTOS_KillAllThreads();

/* Adds an aperiodic event thread to the scheduler
 *  -verify the IRQn is within valid user IRQn range
 *  -verify priority is not greater than 6, the greatest user priority number
 *  -use NVIC funcitons
 */
sched_ErrCode_t G8RTOS_AddAPeriodicEvent(void (*AThreadToAdd)(void), uint8_t priority, IRQn_Type IRQn);

/*
 * Adds Periodic threads to G8RTOS
 * -takes in a pointer to a thread handler and a period
 * Returns: Error code for adding periodic threads
 */
int32_t G8RTOS_AddPE(void (*Handler)(void), uint32_t period);

/*
 * Yields control to the operating system by triggering the PendSV handler
 * This will begin a context switch and call the OS scheduler
 */
inline void OS_Yield();

/*takes in a time to sleep in milliseconds
 * Sets currentlyrunningthread's sleep count to current system time + duration
 * sets asleep to true
 * yield control to the operating system
 */

void OS_Sleep(uint32_t duration);

/*
 * Returns the CurrentlyRunningThread's thread ID
 */
threadId_t G8RTOS_GetThreadId();

void G8RTOS_debug(void);

/*********************************************** Public Functions *********************************************************************/

#endif /* G8RTOS_SCHEDULER_H_ */
