/*
 * G8RTOS_Scheduler.c
 */

/*********************************************** Dependencies and Externs *************************************************************/

#include <stdint.h>
#include "msp.h"
#include "G8RTOS_Scheduler.h"
#include "BSP.h"
#include "G8RTOS_Structures.h"
#include "driverlib.h"
#include "G8RTOS_CriticalSection.h"

/*
 * G8RTOS_Start exists in asm
 */
extern void G8RTOS_Start();

/* System Core Clock From system_msp432p401r.c */
extern uint32_t SystemCoreClock;

/*
 * Pointer to the currently running Thread Control Block
 */
extern struct tcb_t * CurrentlyRunningThread;

/*********************************************** Dependencies and Externs *************************************************************/


/*********************************************** Defines ******************************************************************************/

/* Status Register with the Thumb-bit Set */
#define THUMBBIT 0x01000000

/*********************************************** Defines ******************************************************************************/


/*********************************************** Data Structures Used *****************************************************************/

/* Thread Control Blocks
 *	- An array of thread control blocks to hold pertinent information for each thread
 */
static struct tcb_t threadControlBlocks[MAX_THREADS];

/*
 * Holds a linked list of period events
 */
static struct pe_t periodicEvents[MAX_PeriodicEvent];

/* Thread Stacks
 *	- An array of arrays that will act as invdividual stacks for each thread
 */
static int32_t threadStacks[MAX_THREADS][STACKSIZE];


/*********************************************** Data Structures Used *****************************************************************/


/*********************************************** Private Variables ********************************************************************/

/*
 * Current Number of Threads currently in the scheduler
 */
static uint32_t NumberOfThreads;

/*
 * Current number of periodic events currently in the OS
 */
static uint32_t NumberOfPEs;

/*
 * ID counter for thread creation
 */
static uint16_t IDCounter;

/*********************************************** Private Variables ********************************************************************/


/*********************************************** Private Functions ********************************************************************/

/*
 * Initializes the Systick and Systick Interrupt
 * The Systick interrupt will be responsible for starting a context switch between threads
 * Param "numCycles": Number of cycles for each systick interrupt
 */
static void InitSysTick(unsigned int numCycles)
{
	SysTick_Config((uint32_t) numCycles);
	SysTick_enableInterrupt();
}

/*
 * Chooses the next thread to run.
 * Lab 2 Scheduling Algorithm:
 * 	- Simple Round Robin: Choose the next running thread by selecting the currently running thread's next pointer
 */
void G8RTOS_Scheduler()
{

    //write round-robin based priority scheduler. 0 is max priority, 255 is lowest priority
    uint16_t currentMaxPriority = 256;
    struct tcb_t* tempNextThread = CurrentlyRunningThread->next;
    struct tcb_t* startThread = CurrentlyRunningThread;

    //FIXME: check if functionality is correct
    //iterate tempNextThread to check priorities/blocked/sleeping status. Once the beginning of the loop is reached, terminate
    do {
        if (tempNextThread->blocked == 0 && tempNextThread->asleep == false) {
            //if tempNextThread is not blocked and not asleep, compare its priority to currentMaxPriority
            if (tempNextThread->priority < currentMaxPriority) {
                CurrentlyRunningThread = tempNextThread;
                currentMaxPriority = (uint16_t) CurrentlyRunningThread->priority;
            }
        }
        tempNextThread = tempNextThread->next;
    } while(tempNextThread != startThread->next);

}

/*
 * SysTick Handler
 * Currently the Systick Handler will only increment the system time
 * and set the PendSV flag to start the scheduler
 *
 * In the future, this function will also be responsible for sleeping threads and periodic threads
 */
void SysTick_Handler()
{

	//handle periodic event threads
	struct pe_t* Pptr = &periodicEvents[0];
	uint32_t i = 0;
	while (i < NumberOfPEs) {
	    if (Pptr->executeTime == SystemTime) {
	        //assigns next execution time to current system time plus period
	        Pptr->executeTime = SystemTime + Pptr->period;
	        //run the event handler
	        Pptr->Handler();
	    }
	    //move to next periodic event
	    Pptr = Pptr->next;
	    i++;
	}

	SystemTime++;

	//Handle sleeping threads. wake up all threads whose sleep time has been reached.
	struct tcb_t* ptr = CurrentlyRunningThread;
	i = 0;
	while (i < NumberOfThreads) {
	    if (ptr->asleep == true) {
	        if (ptr->sleepCount <= SystemTime) {
	            ptr->asleep = false;
	        }
	    }
	    ptr = ptr->next;
	    i++;
	}

	//set the PendSV flag last
	SCB->ICSR |= 1<<28;
}

/*********************************************** Private Functions ********************************************************************/


/*********************************************** Public Variables *********************************************************************/

/* Holds the current time for the whole System */
uint32_t SystemTime;

/*********************************************** Public Variables *********************************************************************/


/*********************************************** Public Functions *********************************************************************/

/*
 * Sets variables to an initial state (system time and number of threads)
 * Enables board for highest speed clock and disables watchdog
 */
void G8RTOS_Init()
{
    SystemTime = 0;
    NumberOfThreads = 0;
    NumberOfPEs = 0;
    IDCounter = 0;
    uint32_t newVTORTable = 0x20000000;
    memcpy((uint32_t*)newVTORTable,(uint32_t*)SCB->VTOR,57*4);
    SCB->VTOR = newVTORTable;
    BSP_InitBoard();
}

/*
 * Starts G8RTOS Scheduler
 * 	- Initializes the Systick
 * 	- Sets Context to first thread
 * Returns: Error Code for starting scheduler. This will only return if the scheduler fails
 */
int G8RTOS_Launch()
{

    //if there are no threads in the scheduler, return a scheduling error
    if (NumberOfThreads < 1) {
        return 2;
    }

    //Set CurrentlyRunningThread to the thread with highest priority
    //FIXME: check for correctness
    CurrentlyRunningThread = &threadControlBlocks[0];
    struct tcb_t* tempNextThread = CurrentlyRunningThread->next;
    uint16_t currentMaxPriority = 256;
    int i = 0;
    //check each thread
    for (i = 0; i < NumberOfThreads; i++) {
        if (tempNextThread->priority < currentMaxPriority) {
            CurrentlyRunningThread = tempNextThread;
            currentMaxPriority = (uint16_t) tempNextThread->priority;
        }
        tempNextThread = tempNextThread->next;
    }

    //Enable PendSV interrupts
    SCB->SHP[10] = 255; //set the priority of PendSV to lowest
    //Arm SysTick and PendSV exceptions.
    InitSysTick(ClockSys_GetSysFreq()/1000); //calculate the cycles required to trigger SysTick every 1ms
    SCB->SHP[11] = 0;

    //this function should not return
    //load the context of CurrentlyRunningThread into the CPU
    G8RTOS_Start();

    while(1);
    return 0;
}

/*
* Yields control to the operating system by triggering the PendSV handler
* This will begin a context switch and call the OS scheduler
*/
inline void OS_Yield() {
    //set PendSV flag
    SCB->ICSR |= 1<<28;
}


/*takes in a time to sleep in milliseconds
* Sets currentlyrunningthread's sleep count to current system time + duration
* sets asleep to true
* yield control to the operating system
*/
void OS_Sleep(uint32_t duration) {
    CurrentlyRunningThread->sleepCount = SystemTime + duration;
    CurrentlyRunningThread->asleep = true;
    OS_Yield();
}

/*
 * Adds threads to G8RTOS Scheduler
 * 	- Checks if there are still available threads to insert to scheduler
 * 	- Initializes the thread control block for the provided thread
 * 	- Initializes the stack for the provided thread to hold a "fake context"
 * 	- Sets stack tcb stack pointer to top of thread stack
 * 	- Sets up the next and previous tcb pointers in a round robin fashion
 * Param "threadToAdd": Void-Void Function to add as preemptable main thread
 * Returns: Error code for adding threads
 */
//XME: update return type to be new error code
int32_t G8RTOS_AddThread(void (*threadToAdd)(void), uint8_t priority, char* name)
{
    int32_t primask = StartCriticalSection();
    uint16_t tcbToInitialize = 0;
    bool foundThread = false;
    int i = 0;
    
    if (NumberOfThreads >= MAX_THREADS) { //if all threads are occupied, return thread error code
        EndCriticalSection(primask);
        return 1;
    }
    else if (IDCounter == 0) { //if first thread being added, set next and prev to itself
        //initialize the linked list when the first thread is added
        //FIXME: this initial linked list stuff may not be necessary here
        threadControlBlocks[0].next = &threadControlBlocks[0];
        threadControlBlocks[0].prev = &threadControlBlocks[0];
        tcbToInitialize = 0;

        //loop through all TCBs to initialize all of their alives to false except for index 0 upon first thread creation

        for (i = 1; i < MAX_THREADS; i++) {
            threadControlBlocks[i].alive = false;
        }
    }

    //find the next dead thread to add to the linked list
    else {
        for (i = 0; i < MAX_THREADS; i++) {
            if (threadControlBlocks[i].alive == false) {
                //if tcb at index i is available, set to initialize it and set found to true
                tcbToInitialize = i;
                foundThread = true;
                break;
            }
        }

        //FIXME:
        //if there are no dead threads available, return THREADS_INCORRECTLY_ALIVE error code
        if (!foundThread) {
            EndCriticalSection(primask);
            return THREADS_INCORRECTLY_ALIVE;
        }
    }

    threadControlBlocks[tcbToInitialize].alive = true;

    //implement logic to determine the new linked list connections formed from adding this new TCB

    //iterate forward as circular array until alive tcb is found
    uint16_t nextTcb = 0;
    //determine starting nextTcb with circular array logic
    if(tcbToInitialize == MAX_THREADS-1) {
        nextTcb = 0;
    }
    else {
        nextTcb = tcbToInitialize + 1;
    }
    //increment forward to find next alive tcb
    for (i = 0; i < MAX_THREADS; i++) {
        //check if tcb at nextTcb is alive
        if (threadControlBlocks[nextTcb].alive) {
            i = i;
            break;
        }
        else {
            //circular array incrementing logic
            if(nextTcb == MAX_THREADS-1) {
                nextTcb = 0;
            }
            else {
                nextTcb = nextTcb + 1;
            }
        }
    }

    //linked list operations
    //set nextTcb to tcbToInitialize's next and set tcbToInitialize to nextTcb prev's next
    //set tcbToInitialize's prev to nextTcb's prev
    //set nextTcb's prev to tcbToInitialize

    threadControlBlocks[tcbToInitialize].next = &threadControlBlocks[nextTcb];
    threadControlBlocks[nextTcb].prev->next = &threadControlBlocks[tcbToInitialize];
    threadControlBlocks[tcbToInitialize].prev = threadControlBlocks[nextTcb].prev;
    threadControlBlocks[nextTcb].prev = &threadControlBlocks[tcbToInitialize];

    //finish TCB initialization

    threadControlBlocks[tcbToInitialize].asleep = false;
    threadControlBlocks[tcbToInitialize].blocked = 0;
    threadControlBlocks[tcbToInitialize].sleepCount = 0;
    threadControlBlocks[tcbToInitialize].priority = priority;
    threadControlBlocks[tcbToInitialize].threadID = ((IDCounter++) << 16) | tcbToInitialize;

    //set name by iterating through input name string
    char* iter = &name[0];
    i = 0;
    while (*iter != 0 && i < 16) {
        threadControlBlocks[tcbToInitialize].threadName[i] = *iter;
        iter++;
        i++;
    }
    if (*iter == 0) {
        threadControlBlocks[tcbToInitialize].threadName[i] = *iter;
    }

    threadControlBlocks[tcbToInitialize].sp = &threadStacks[tcbToInitialize][STACKSIZE-2]; //set the stack pointer to point to the thread's stack
    *threadControlBlocks[tcbToInitialize].sp = 1<<24;
    threadControlBlocks[tcbToInitialize].sp--;
    *threadControlBlocks[tcbToInitialize].sp = (int32_t) threadToAdd; //initialize the program counter to the parameter function
    threadControlBlocks[tcbToInitialize].sp--;
    *threadControlBlocks[tcbToInitialize].sp = 0; //initialize the link register.

    i = 0;

    for (i = 0; i < 13; i++) { //initialize the scratch and variable core registers to 0. :ShiroBliss:
        threadControlBlocks[tcbToInitialize].sp--;
        *threadControlBlocks[tcbToInitialize].sp = 0;
    }

    NumberOfThreads++;
    //return normal error code
    EndCriticalSection(primask);
    //FIXME:
    return 0;
}

/*
 * Adds Periodic threads to G8RTOS
 * -takes in a pointer to a thread handler and a period
 * Returns: Error code for adding periodic threads
 */
int32_t G8RTOS_AddPE(void (*Handler)(void), uint32_t period) {
    int32_t primask = StartCriticalSection();
    if (NumberOfPEs >= MAX_PeriodicEvent) { //if all PEs are occupied, return PE error code
            //FIXME: change error code
           EndCriticalSection(primask);
           return 7;
       }
       else if (NumberOfPEs == 0) { //if first PE being added, set next and prev to itself
           //initialize the linked list when the first thread is added
           periodicEvents[0].next = &periodicEvents[0];
           periodicEvents[0].prev = &periodicEvents[0];
       }

       else { //reorganize the linked list when a new thread is added
           //only need to change the head (located at periodicEvents[0]) and the tail (located at periodicEvents[NumberOfPEs-1])
           int head_index = 0; //index where the head is located
           int prevtail_index = NumberOfPEs-1; //index where the previous tail is located
           int tail_index = NumberOfPEs; //index where the new tail is located

           //new tail's prev will point to the old tail
           periodicEvents[tail_index].prev = &periodicEvents[prevtail_index];
           //new tail's next will point to the head
           periodicEvents[tail_index].next = &periodicEvents[head_index];

           //change prevtail's next to point to the new tail
           periodicEvents[prevtail_index].next = &periodicEvents[tail_index];

           //head's previous will always point to the list's tail
           periodicEvents[head_index].prev = &periodicEvents[tail_index];

       }

    //initialize the rest of the PEs state
    periodicEvents[NumberOfPEs].Handler = Handler;

    periodicEvents[NumberOfPEs].executeTime = 0;

    periodicEvents[NumberOfPEs].period = period;

    NumberOfPEs++;
    EndCriticalSection(primask);
    return 0;
}

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
sched_ErrCode_t G8RTOS_KillThread(threadId_t threadId) {
    int32_t primask = StartCriticalSection();
    if (NumberOfThreads == 1) {
        EndCriticalSection(primask);
        return CANNOT_KILL_LAST_THREAD;
    }

    bool foundThread = false;
    int i;

    //search for threadID in the tcb linked list
    struct tcb_t* thread_ptr = CurrentlyRunningThread;
    for (i = 0; i < NumberOfThreads; i++) {
        if (thread_ptr->threadID == threadId) {
            foundThread = true;
            break;
        }
        thread_ptr = thread_ptr->next;
    }

    //if the thread being searched was not found in the linked list, return not found error code
    if (!foundThread) {
        EndCriticalSection(primask);
        return THREAD_DOES_NOT_EXIST;
    }

    thread_ptr->alive = false;

    //increment semaphore if blocked
    if (thread_ptr->blocked != 0) {
        thread_ptr->blocked++;
    }

    //update thread pointers.
    //Update thread_ptr's next's prev to be thread_ptr's prev
    //update thread_ptr's prev's next to be thread_ptr's next
    thread_ptr->next->prev = thread_ptr->prev;
    thread_ptr->prev->next = thread_ptr->next;

    NumberOfThreads--;

    if (thread_ptr == CurrentlyRunningThread) {
           OS_Yield(); //immediately call context switch when critical section exits
    }

    EndCriticalSection(primask);

    return NO_ERROR;

}

/* Kills the CurrentlyRunningThread
 *  -enters a critical section
 *  -return CANNOT_KILL_LAST_THREAD if only one thread running
 *  -change alive to false
 *  -update thread pointers
 *  -start context switch
 *  -decrement number of threads
 *  -end critical section
 */
sched_ErrCode_t G8RTOS_KillSelf() {
    int32_t primask = StartCriticalSection();
    if (NumberOfThreads == 1) {
        EndCriticalSection(primask);
        return CANNOT_KILL_LAST_THREAD;
    }

    CurrentlyRunningThread->alive = false;

    CurrentlyRunningThread->next->prev = CurrentlyRunningThread->prev;
    CurrentlyRunningThread->prev->next = CurrentlyRunningThread->next;

    OS_Yield();
    NumberOfThreads--;

    EndCriticalSection(primask);

    return NO_ERROR;
}

/*
* Kills all threads in the scheduler other than CurrentlyRunningThread
* returns if G8RTOS_KillThread returns an error, stop the process and return that error code
*/
sched_ErrCode_t G8RTOS_KillAllThreads() {
    int32_t primask = StartCriticalSection();
    sched_ErrCode_t errorCd = NO_ERROR;

    struct tcb_t* currTCB = CurrentlyRunningThread->next;
    while (currTCB != CurrentlyRunningThread && errorCd == NO_ERROR) {
        errorCd = G8RTOS_KillThread(currTCB->threadID);
        currTCB = currTCB->next;
    }

    EndCriticalSection(primask);
    
    return errorCd;
}

/* Adds an aperiodic event thread to the scheduler
 *  -verify the IRQn is within valid user IRQn range
 *  -verify priority is not greater than 6, the greatest user priority number
 *  -use NVIC funcitons
 */
sched_ErrCode_t G8RTOS_AddAPeriodicEvent(void (*AThreadToAdd)(void), uint8_t priority, IRQn_Type IRQn) {
    int32_t primask = StartCriticalSection();
    if (IRQn < PSS_IRQn || IRQn > PORT6_IRQn) {
        //IRQn is invalid for user IRQn
        EndCriticalSection(primask);
        return IRQn_INVALID;
    }

    //FIXME: check priority
    if (priority > 6) {
        EndCriticalSection(primask);
        return HWI_PRIORITY_INVALID;
    }

    __NVIC_SetVector(IRQn, (uint32_t)AThreadToAdd);
    __NVIC_SetPriority(IRQn, priority);
    NVIC_EnableIRQ(IRQn);
    EndCriticalSection(primask);
    return NO_ERROR;
}

/*
 * Returns the CurrentlyRunningThread's thread ID
 */
threadId_t G8RTOS_GetThreadId() {
    return CurrentlyRunningThread->threadID;
}

//FIXME:
void G8RTOS_debug(void) {
    while(1);
}



/*********************************************** Public Functions *********************************************************************/
