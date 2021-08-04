/*
 * threads.h
 *
 *  Created on: Mar 23, 2021
 *      Author: enemi
 */

#ifndef THREADS_H_
#define THREADS_H_

/*********** Init Functions ***********/

//Name: InitRSLK
//Type: Initialization function, run once before booting OS
//Purpose: Initializes semaphores and runs any other relevant initialization functions; ie G8RTOS, motor, BSP. Run on robot only
//Author: Ryan Roth
void InitRSLK(void);

/*********** Init Functions ***********/

/*********** Structure Definitions ***********/



typedef struct
{
    uint32_t IP_address;    //IP address of client
    bool ready;             //Is ready to play
    bool joined;            //Joined the game
    bool acknowledge;       //Acknowledge to be sent from Host
} ClientData_t;

typedef struct
{
    ClientData_t client;
    bool newValue;
    int32_t Xavg;         //Left Right
    int32_t Yavg;         //Forward Backward
} JoyStickData_t;


//********************************************************************************************?//
/*********** Controller Thread Declarations ***********/

//Name: JoystickRead_th
//Type: normal controller thread
//Purpose: Reads new X and Y values from the joystick. Updates the running average after checking for semaphores and sets new joystick value flag
//         then sleeps for desired sampling rate.
void JoyStickRead_th(void);

//Name: JoystickSend_th
//Type: normal controller thread
//Purpose: takes joystick semaphore and checks new joystick value flag. If it is set, send joystick averages to the robot. Else, sleep
//         Upon reaching the end of the thread, clear the new joystick value flag
void JoyStickSend_th(void);

//CONTROLLER INITIALIZATION THREAD, BEGIN COMMUNICATION

//Name: JoyStickInit
//Type: Initial controller thread, launch with it in the scheduler and kill self on completion
//Purpose: Initialize joystick and joystick data values and connect to the robot via wifi. Schedule normal controller threads then kill self
void ControllerInit(void);

/*********** Controller Thread Declarations ***********/

/*********** Robot Thread Declarations ***********/

//Name: Motor_th
//Type: Normal Thread
//Purpose: checks for new joystick data, run the motor controller if there is new data. Otherwise, sleep. Set new data flag to 0 and sleep after running control routine
//Author: Ryan Roth
void Motor_th(void);

//Name: JoystickRcv_th
//Type: Normal Thread
//Purpose: tries to receive data from the joystick device. If nothing is received, sleep for 2ms.
//         if data is received, set new data flag to 1.
void JoystickRcv_th(void);

/*
//Name: TimerIntLeft
//Type: Aperiodic interrupt thread tied to timerA
//Purpose: Toggles the left motor's PWM pin.
void TimerIntLeft(void);

//Name: TimerIntRight
//Type: Aperiodic interrupt thread tied to timerA
//Purpose: Toggles the right motor's PWM pin.
void TimerIntRight(void);
*/

//HAVE THREADS THAT INITIALIZE THE CONNECTION AND GET KILLED FROM THE SCHEDULER LIKE LAB 5
//Name: RobotInit
//Type: Robot Initialization Thread, run once and kill thread
//Purpose: Initialize motor and timers and connect to the controller via wifi. Schedule normal robot threads then kill self.
void RobotInit(void);



/*
 * Initializes Host
 */
void HostThread(void);

/*
 * Initializes Client
 */
void ClientThread(void);

/*
 * Idle thread that runs when others do not
 */
void Idle(void);

/*
 * Reads Joystick and saves data
 */
void ReadJoystick(void);


void ReceiveDataFromHost(void);


void SendDataToHost(void);

void SendDataToClient(void);

void ReceiveDataFromClient(void);

void Idle(void);

#endif /* THREADS_H_ */
