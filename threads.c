/*
 * threads.c
 *
 *  Created on: Mar 23, 2021
 *      Author: enemi
 */
#include "G8RTOS.h"
#include "G8RTOS_Scheduler.h"
#include <DriverLib.h>
#include "BSP.h"
#include "LCDLib.h"
#include <time.h>
#include <stdlib.h>
#include "threads.h"
#include "motor.h"
#include <cc3100_usage.h>
#include "demo_sysctl.h"
#include "bumpers.h"

/*********** Semaphores ***********/
semaphore_t WIFIMutex;
semaphore_t s_JoyStick;
semaphore_t s_Bumper;

/*********** Semaphores ***********/



//Variables that will hold coordinates
int16_t x, y;

/******************Inline Functions *********************/
inline void emptyHostPacket()
{
    JoyStickData_t temp;
    int32_t retVal;
    //while(ReceiveData((uint8_t*)&temp, sizeof(temp)) < 0);
    while(true)
    {
        G8RTOS_WaitSemaphore(&WIFIMutex);
        retVal = ReceiveData((uint8_t*)&temp, sizeof(temp));
        if(retVal >= 0)
        {
            G8RTOS_SignalSemaphore(&WIFIMutex);
            break;
        }
        G8RTOS_SignalSemaphore(&WIFIMutex);
        OS_Sleep(1);
    }
    JoyStickData = temp;
}

inline void emptyClientPacket()
{
    ClientData_t temp;
    int32_t retVal;
    //while(ReceiveData((uint8_t*)&temp, sizeof(temp)) < 0);
    while(true)
    {
        G8RTOS_WaitSemaphore(&WIFIMutex);
        retVal = ReceiveData((uint8_t*)&temp, sizeof(temp));
        if(retVal >= 0)
        {
            G8RTOS_SignalSemaphore(&WIFIMutex);
            break;
        }
        G8RTOS_SignalSemaphore(&WIFIMutex);
        OS_Sleep(1);
    }
    c = temp;
}

inline void sendHostPacket()
{
    JoyStickData_t temp = JoyStickData;
    G8RTOS_WaitSemaphore(&WIFIMutex);
    SendData((uint8_t*)&temp, temp.client.IP_address, sizeof(temp));
    G8RTOS_SignalSemaphore(&WIFIMutex);
}

inline void sendClientPacket()
{
    ClientData_t temp = c;
    G8RTOS_WaitSemaphore(&WIFIMutex);
    SendData((uint8_t*)&temp, HOST_IP_ADDR, sizeof(temp));
    G8RTOS_SignalSemaphore(&WIFIMutex);
}


/*********** Init Functions ***********/
/*********** Controller Thread Definitions ***********/

//Name: JoystickRead_th
//Type: normal controller thread
//Purpose: Reads new X and Y values from the joystick. Updates the running average after checking for semaphores and sets new joystick value flag
//         then sleeps for desired sampling rate.
void JoystickRead_th(void) {
    while(1)
    {
        //Gets Joystick coordinates
        GetJoystickCoordinates(&x, &y);

        G8RTOS_WaitSemaphore(&s_JoyStick);

        //Calculates decayed average value for coordinate X and Y
        JoyStickData.Xavg = (JoyStickData.Xavg + x) >> 1;
        JoyStickData.Yavg = (JoyStickData.Yavg + y) >> 1;

        JoyStickData.newValue = true;

        G8RTOS_SignalSemaphore(&s_JoyStick);

        OS_Sleep(10);
    }

}

//Name: JoystickSend_th
//Type: normal controller thread
//Purpose: takes joystick semaphore and checks new joystick value flag. If it is set, send joystick averages to the robot. Else, sleep
//         Upon reaching the end of the thread, clear the new joystick value flag
void JoystickSend_th(void) {
    while(1)
    {
        G8RTOS_WaitSemaphore(&s_JoyStick);

        if (JoyStickData.newValue)
        {
            sendHostPacket();
            JoyStickData.newValue = false;
        }

        G8RTOS_SignalSemaphore(&s_JoyStick);

        OS_Sleep(5);
    }
}

void BumpersSend_th(void)
{
    while(1)
    {
        G8RTOS_WaitSemaphore(&s_Bumper);

        if(c.newBump)
        {
            sendClientPacket();
            c.newBump = false;
        }
        G8RTOS_SignalSemaphore(&s_Bumper);

        OS_Sleep(20);
    }
}

void BumpersReceive_th(void)
{
    while(1)
    {
        G8RTOS_WaitSemaphore(&s_Bumper);
        emptyClientPacket();
        G8RTOS_SignalSemaphore(&s_Bumper);

        OS_Sleep(10);
    }
}

/*********** Controller Thread Definitions ***********/

/*********** Robot Thread Definitions ***********/

//Name: RobotInit
//Type: Robot Initialization Thread, run once and kill thread
//Purpose: Initialize motor and timers and connect to the controller via wifi. Schedule normal robot threads then kill self.
void RobotInit(void) {
    Motor_Init();

    //Connect to controller via WiFi

    //add robot threads to G8RTOS schedule
    G8RTOS_AddThread(Motor_th, 2, "Motor_th");
    G8RTOS_AddThread(JoystickRcv_th, 2, "Joy_Rcv");

    G8RTOS_KillSelf();
}

//Name: Motor_th
//Type: Normal Thread
//Purpose: checks for new joystick data, run the motor controller if there is new data. Otherwise, sleep. Set new data flag to 0 and sleep after running control routine
//Author: Ryan Roth
void Motor_th(void) {
    while(1) {
        G8RTOS_WaitSemaphore(&s_JoyStick);
        if (JoyStickData.newValue) {
            Motor_CTRL(JoyStickData.Xavg, JoyStickData.Yavg);
            JoyStickData.newValue = false;
            G8RTOS_SignalSemaphore(&s_JoyStick);
            OS_Sleep(10); //FIXME: determine sleep time. Should be longer than below sleep time
        }
        else {
            G8RTOS_SignalSemaphore(&s_JoyStick);
            OS_Sleep(5); //FIXME: determine sleep time. SHould be shorter than above sleep time
        }
    }
}

//Name: JoystickRcv_th
//Type: Normal Thread
//Purpose: tries to receive data from the controller device. If nothing is received, sleep for 2ms.
//         if data is received, set new data flag to 1.
void JoystickRcv_th(void) {
    int Status;
    while(1) {
        //continually receive data until valid data has been read (greater than 0)
        //cycle the semaphore and sleep to prevent deadlock like in the host's function
        Status = NOTHING_RECEIVED;

        while(Status == NOTHING_RECEIVED) {

               //WAIT FOR JoyStick SEMAPHORE
               G8RTOS_WaitSemaphore(&s_JoyStick);
               emptyHostPacket();
               //RELEASE JoyStick Semaphore
               G8RTOS_SignalSemaphore(&s_JoyStick);
               OS_Sleep(2);
          }
    }
}

/*
//Name: TimerIntLeft
//Type: Aperiodic interrupt thread tied to timerA
//Purpose: Toggles the left motor's PWM pin.
void TimerIntLeft(void) {
    //FIXME: clear whatever interrupt flags need to be cleared
    TGL_PWML();
}

//Name: TimerIntRight
//Type: Aperiodic interrupt thread tied to timerA
//Purpose: Toggles the right motor's PWM pin.
void TimerIntRight(void) {
    //FIXME: clear interrupt flags
    TGL_PWMR();
}
*/


void HostThread(void)
{
    //initializes semaphores:
    G8RTOS_InitSemaphore(&WIFIMutex, 1);
    G8RTOS_InitSemaphore(&s_JoyStick, 1);
    G8RTOS_InitSemaphore(&s_Bumper, 1);

    //Initializes to default state
    JoyStickData.Xavg = 0;
    JoyStickData.Yavg = 0;
    JoyStickData.newValue = false;

    //Waits for client to be ready
    while(!c.ready)
    {
        ClientData_t temp;
        int32_t retVal;
        while(true)
        {
            DelayMs(5);
            retVal = ReceiveData((uint8_t*)&temp, sizeof(temp));
            if(retVal >= 0)
            {
                break;
            }
        }
        c = temp;
    }
    P2->DIR |= BIT0; //Makes button 0 an output
    P2->OUT |= BIT0; //Turns it on

    //Sends acknowledgment
    JoyStickData.client = c;
    JoyStickData.client.acknowledge = true;


    //Confirm that client joined
    while(!c.joined)
    {


        ClientData_t temp;
        int32_t retVal;
        while(true)
        {
            //Sends Acknowledgement
            SendData((uint8_t*)&JoyStickData, JoyStickData.client.IP_address, sizeof(JoyStickData));
            DelayMs(5);
            retVal = ReceiveData((uint8_t*)&temp, sizeof(temp));
            if(retVal >= 0)
            {
                break;
            }
            DelayMs(5);
        }
        c = temp;
    }
    P2->DIR |= BIT2; //Makes button 0 an output
    P2->OUT |= BIT2; //Turns it on

    JoyStickData.client = c;


    //Adds threads needed for the game
    G8RTOS_AddThread(JoystickSend_th, 200, "SndDtaClient");
    G8RTOS_AddThread(JoystickRead_th, 200, "JoyStickRead");
    G8RTOS_AddThread(BumpersReceive_th, 200, "Bump_Rcv");
    //G8RTOS_AddThread(ReceiveDataFromClient, 200, "RecDataFromClient");
    G8RTOS_AddThread(Idle, 255, "Idle");

    srand(time(0));

    //Kills self
    G8RTOS_KillSelf();
    //Sets PendSV flag
    SCB->ICSR |= (1<<28);
}

void ClientThread(void)
{
    //Initializes Semaphores
    G8RTOS_InitSemaphore(&WIFIMutex, 1);
    G8RTOS_InitSemaphore(&s_JoyStick, 1);
    G8RTOS_InitSemaphore(&s_Bumper, 1);

    //Fills packet with client info
    c.IP_address = getLocalIP();
    c.ready = true;
    c.joined = false;
    c.acknowledge = false;
    c.bump0 = false;
    c.bump1 = false;
    c.bump2 = false;
    c.bump3 = false;
    c.bump4 = false;
    c.bump5 = false;
    c.newBump = false;

    //Sends player info to host
    sendClientPacket();

    //Sets acknowledge to false by default
    JoyStickData.client.acknowledge = false;

    //Receives data from host and acknowledgment
    JoyStickData_t temp;
    int32_t retVal;

    //While acknowledge is false
    while(!JoyStickData.client.acknowledge)
    {
        //Sends player info and receives continuously until a acknowledgment is received
        while(true)
        {
            sendClientPacket();
            DelayMs(5);

            retVal = ReceiveData((uint8_t*)&temp, sizeof(temp));
            if(retVal >= 0)
            {
                break;
            }
            DelayMs(5);
        }
        JoyStickData = temp;
    }

    P2->DIR |= BIT0; //Makes button 0 an output
    P2->OUT |= BIT0; //Turns it on

    c = JoyStickData.client;
    c.joined = true;

    //Sends acknowledgment of joined game
    sendClientPacket();

    //Adds threads needed for the game
    G8RTOS_AddThread(BumpersSend_th, 2, "Bump_Send");
    G8RTOS_AddThread(JoystickRcv_th, 2, "Joy_Rcv");
    G8RTOS_AddThread(bumper_Check, 2, "Bumpers");
    G8RTOS_AddThread(Motor_th, 2, "Motor_th");
    G8RTOS_AddThread(Idle, 255, "Idle");

    //Kill this thread
    G8RTOS_KillSelf();
    SCB->ICSR |= (1<<28); //set the pendsv bit to switch tasks
}

/*
 * Idle thread that runs when others do not
 */
void Idle(void)
{
    while(1);
}
