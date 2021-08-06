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
    bool newBump;
    bool bump0;
    bool bump1;
    bool bump2;
    bool bump3;
    bool bump4;
    bool bump5;
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

//Name: HostThread
//Type: Initial controller thread, launch with it in the scheduler and kill self on completion
//Purpose: Initialize joystick and joystick data values and connect to the robot via wifi. Schedule normal controller threads then kill self
void HostThread(void);

//Name: LCD_th
//Type: normal controller thread
//Purpose: Uses the bump sensor data from the robot to draw or erase the red rectangles at each position that indicate a collision
//         sleep for about 50ms
void LCD_th(void);

//Name: BumperRecv_th
//Type: normal controller thread
//Purpose: Receives packets from the robot which contain information on the bumpers
void BumperRecv_th(void);

/*
 * Reads Joystick and saves data
 */
/*
void ReadJoystick(void);

void SendDataToClient(void);
*/

/*********** Controller Thread Declarations ***********/

/*********** Robot Thread Declarations ***********/

//Name: Motor_th
//Type: Normal Robot Thread
//Purpose: checks for new joystick data, run the motor controller if there is new data. Otherwise, sleep. Set new data flag to 0 and sleep after running control routine
//Author: Ryan Roth
void Motor_th(void);

//Name: JoystickRcv_th
//Type: Normal Robot Thread
//Purpose: tries to receive data from the joystick device. If nothing is received, sleep for 2ms.
//         if data is received, set new data flag to 1.
void JoystickRcv_th(void);

//Name: ClientThread
//Type: Robot Initialization Thread, run once and kill thread
//Purpose: Initialize motor and timers and connect to the controller via wifi. Schedule normal robot threads then kill self.
void ClientThread(void);

//Name: BumperRead_th
//Type: Normal Robot Thread
//Purpose: Read new values from the bumpers on the front of the robot. Sleep for about 40ms.
void BumperRead_th(void);

//Name: BumperSend_th
//Type: Normal Robot Thread
//Purpose: Send packet containing bumper values to the controller to display on the LCD
void BumperSend_th(void);

/*
 * Idle thread that runs when others do not
 */
void Idle(void);

#endif /* THREADS_H_ */
