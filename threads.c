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
#include "image.h"

/*********** Semaphores ***********/
semaphore_t WIFIMutex;
semaphore_t s_JoyStick;
semaphore_t s_Bumper;

/*********** Semaphores ***********/

/*********** Private Defintions ***********/

#define BMP0_XPos   178
#define BMP0_YPos   154
#define BMP1_XPos   205
#define BMP1_YPos   164
#define BMP2_XPos   232
#define BMP2_YPos   180
#define BMP3_XPos   137
#define BMP3_YPos   154
#define BMP4_XPos   110
#define BMP4_YPos   164
#define BMP5_XPos   83
#define BMP5_YPos   180
#define BMP_WIDTH   5
#define BMP_HEIGHT  5

/*********** Private Defintions ***********/

/*********** JoyStick Data ***********/
JoyStickData_t JoyStickData;
ClientData_t c;

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

inline void drawCollision() {
    //draw collision markers on the LCD. Red for collision, white for no collision
    if (c.bump2) {
        LCD_DrawRectangle(BMP0_XPos, BMP0_XPos + BMP_WIDTH, BMP0_YPos, BMP0_YPos + BMP_HEIGHT, LCD_RED);
    }
    else {
        LCD_DrawRectangle(BMP0_XPos, BMP0_XPos + BMP_WIDTH, BMP0_YPos, BMP0_YPos + BMP_HEIGHT, LCD_WHITE);
    }
    if (c.bump1) {
        LCD_DrawRectangle(BMP1_XPos, BMP1_XPos + BMP_WIDTH, BMP1_YPos, BMP1_YPos + BMP_HEIGHT, LCD_RED);
    }
    else {
        LCD_DrawRectangle(BMP1_XPos, BMP1_XPos + BMP_WIDTH, BMP1_YPos, BMP1_YPos + BMP_HEIGHT, LCD_WHITE);
    }
    if (c.bump0) {
        LCD_DrawRectangle(BMP2_XPos, BMP2_XPos + BMP_WIDTH, BMP2_YPos, BMP2_YPos + BMP_HEIGHT, LCD_RED);
    }
    else {
        LCD_DrawRectangle(BMP2_XPos, BMP2_XPos + BMP_WIDTH, BMP2_YPos, BMP2_YPos + BMP_HEIGHT, LCD_WHITE);
    }
    if (c.bump3) {
        LCD_DrawRectangle(BMP3_XPos, BMP3_XPos + BMP_WIDTH, BMP3_YPos, BMP3_YPos + BMP_HEIGHT, LCD_RED);
    }
    else {
        LCD_DrawRectangle(BMP3_XPos, BMP3_XPos + BMP_WIDTH, BMP3_YPos, BMP3_YPos + BMP_HEIGHT, LCD_WHITE);
    }
    if (c.bump4) {
        LCD_DrawRectangle(BMP4_XPos, BMP4_XPos + BMP_WIDTH, BMP4_YPos, BMP4_YPos + BMP_HEIGHT, LCD_RED);
    }
    else {
        LCD_DrawRectangle(BMP4_XPos, BMP4_XPos + BMP_WIDTH, BMP4_YPos, BMP4_YPos + BMP_HEIGHT, LCD_WHITE);
    }
    if (c.bump5) {
        LCD_DrawRectangle(BMP5_XPos, BMP5_XPos + BMP_WIDTH, BMP5_YPos, BMP5_YPos + BMP_HEIGHT, LCD_RED);
    }
    else {
        LCD_DrawRectangle(BMP5_XPos, BMP5_XPos + BMP_WIDTH, BMP5_YPos, BMP5_YPos + BMP_HEIGHT, LCD_WHITE);
    }
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

        //x *= -1;

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

void SendDataToClient()
{
    while(true)
    {
        //Send packet
        sendHostPacket();

        //Sleep
        OS_Sleep(5);
    }
}

//Name: BumperRecv_th
//Type: normal controller thread
//Purpose: Receives packets from the robot which contain information on the bumpers
void BumperRecv_th(void)
{
    while(true)
    {
        G8RTOS_WaitSemaphore(&s_Bumper);
        emptyClientPacket();
        G8RTOS_SignalSemaphore(&s_Bumper);

        //Sleeping for 2ms
        OS_Sleep(2);
    }
}

//Name: LCD_th
//Type: normal controller thread
//Purpose: Uses the bump sensor data from the robot to draw or erase the red rectangles at each position that indicate a collision
//         sleep for about 50ms
void LCD_th(void) {
    while(1) {
        //wait for bumper semaphore
        G8RTOS_WaitSemaphore(&s_Bumper);

        //check new bump flag
        //if it is set, draw on LCD
        if(c.newBump) {
            drawCollision();
            G8RTOS_SignalSemaphore(&s_Bumper);
            OS_Sleep(25);
        }
        //if the flag is not set, sleep a shorter time
        else {
            G8RTOS_SignalSemaphore(&s_Bumper);
            OS_Sleep(10);
        }
    }
}

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
        //Sends Acknowledgement
        SendData((uint8_t*)&JoyStickData, JoyStickData.client.IP_address, sizeof(JoyStickData));

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
    P2->DIR |= BIT2; //Makes button 0 an output
    P2->OUT |= BIT2; //Turns it on

    JoyStickData.client = c;

    //Draw robot image on LCD
    uint16_t xOffset = 80;
    uint16_t yOffset = 172;
    uint16_t size = 4;

    uint16_t k = 0;

    LCD_Clear(LCD_WHITE);
    for (uint8_t j = yOffset; j < 17*size + yOffset; j+=size)
    {
        for (uint8_t i = xOffset; i < 40*size + xOffset; i+=size)
        {
            //2 times size
            LCD_SetPoint(i, j, (image[k]<<8) | image[k+1]);
            LCD_SetPoint(i+1, j, (image[k]<<8) | image[k+1]);
            LCD_SetPoint(i, j+1, (image[k]<<8) | image[k+1]);
            LCD_SetPoint(i+1, j+1, (image[k]<<8) | image[k+1]);

            //3 times size
            LCD_SetPoint(i+2, j, (image[k]<<8) | image[k+1]);
            LCD_SetPoint(i, j+2, (image[k]<<8) | image[k+1]);
            LCD_SetPoint(i+2, j+2, (image[k]<<8) | image[k+1]);
            LCD_SetPoint(i+1, j+2, (image[k]<<8) | image[k+1]);
            LCD_SetPoint(i+2, j+1, (image[k]<<8) | image[k+1]);


            //4 times size
            LCD_SetPoint(i+3, j, (image[k]<<8) | image[k+1]);
            LCD_SetPoint(i, j+3, (image[k]<<8) | image[k+1]);
            LCD_SetPoint(i+3, j+3, (image[k]<<8) | image[k+1]);
            LCD_SetPoint(i+1, j+3, (image[k]<<8) | image[k+1]);
            LCD_SetPoint(i+3, j+1, (image[k]<<8) | image[k+1]);
            LCD_SetPoint(i+3, j+2, (image[k]<<8) | image[k+1]);
            LCD_SetPoint(i+2, j+3, (image[k]<<8) | image[k+1]);


            k+=2;
        }
    }

    //Adds threads needed for the game
    G8RTOS_AddThread(JoystickSend_th, 200, "SndDtaClient");
    G8RTOS_AddThread(JoystickRead_th, 200, "JoyStickRead");
    G8RTOS_AddThread(BumperRecv_th, 200, "BMP_Recv");
    G8RTOS_AddThread(LCD_th, 200, "LCD_th");
    //G8RTOS_AddThread(ReceiveDataFromClient, 200, "RecDataFromClient");
    G8RTOS_AddThread(Idle, 255, "Idle");

    srand(time(0));

    //Kills self
    G8RTOS_KillSelf();
    //Sets PendSV flag
    SCB->ICSR |= (1<<28);
}



/*********** Controller Thread Definitions ***********/

/*********** Robot Thread Definitions ***********/

/*
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
*/

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

    //OS_Sleep(6);
}

//Name: BumperSend_th
//Type: Normal Robot Thread
//Purpose: Send packet containing bumper values to the controller to display on the LCD
void BumperSend_th(void) {
    while(true)
    {
        G8RTOS_WaitSemaphore(&s_Bumper);
        //Send packet
        sendClientPacket();
        G8RTOS_SignalSemaphore(&s_Bumper);

        c.newBump = false;

        //Sleep
        OS_Sleep(5);
    }
}

//Name: BumperRead_th
//Type: Normal Robot Thread
//Purpose: Read new values from the bumpers on the front of the robot. Sleep for about 40ms.
void BumperRead_th(void) {
    while(1) {
        G8RTOS_WaitSemaphore(&s_Bumper);
        bumper_Check(&c);
        G8RTOS_SignalSemaphore(&s_Bumper);
        OS_Sleep(40);
    }
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
    //G8RTOS_AddThread(SendDataToHost, 200, "SndDataToHost");
    G8RTOS_AddThread(JoystickRcv_th, 2, "Joy_Rcv");
    G8RTOS_AddThread(Motor_th, 2, "Motor_th");
    G8RTOS_AddThread(BumperSend_th, 2, "BMP_Send");
    G8RTOS_AddThread(BumperRead_th, 2, "BMP_Read");
    G8RTOS_AddAPeriodicEvent(PORT4_Handler, 1, PORT4_IRQn);
    G8RTOS_AddThread(Idle, 255, "Idle");

    //Kill this thread
    G8RTOS_KillSelf();
    SCB->ICSR |= (1<<28); //set the pendsv bit to switch tasks
}

/*
 * Thread that receives game state packets from host
 */
/*
void ReceiveDataFromHost(void)
{
    while(true)
    {
        emptyHostPacket();
        OS_Sleep(5);
    }
}
*/

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

/*********** Robot Thread Definitions ***********/



/*
 * Thread that sends UDP packets to host
 */
//void SendDataToHost(void)
//{
//    while(true)
//    {
//        sendClientPacket();
//        OS_Sleep(2);
//    }
//}

/*
 * Thread that sends game state to client
 */

/*
 * Idle thread that runs when others do not
 */
void Idle(void)
{
    while(1);
}
