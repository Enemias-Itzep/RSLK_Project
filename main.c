#include "msp.h"
#include <G8RTOS.h>
#include <driverlib.h>
#include "BSP.h"
#include <cc3100_usage.h>
#include "demo_sysctl.h"
#include "threads.h"
#include "motor.h"

#define OFF 0x00
#define ON 0xFF

volatile bool tap = false;
volatile bool tap_11 = false;
volatile bool tap_14 = false;

void PORT1_IRQHandler(void)
{
    //Clear IFG flag
    P1->IFG &= ~BIT4;
    P1->IFG &= ~BIT1;

    tap = true;
}

/**
 * main.c
 */
void main(void)
{
    //Initialize G8RTOS
    G8RTOS_Init();

    //Initializing Semaphores
    //G8RTOS_InitSemaphore(&sensorMutex, 1);
    //G8RTOS_InitSemaphore(&LCDMutex, 1);

    NVIC_EnableIRQ(PORT1_IRQn);


    //Configures the button
    P1->DIR &= ~BIT1; //Makes button 0 an input
    P1->IFG &= ~BIT1; //Clears interrupt flag for button 0
    P1->IE |= BIT1;   //Enables interrupt for Button 0
    P1->IES |= BIT1;  //Flag set on falling edge
    P1->REN |= BIT1;  //Pullup or Pulldown enabled
    P1->OUT |= BIT1;  //Input configured as pullup

    //Configures the button
    P1->DIR &= ~BIT4; //Makes button 0 an input
    P1->IFG &= ~BIT4; //Clears interrupt flag for button 0
    P1->IE |= BIT4;   //Enables interrupt for Button 0
    P1->IES |= BIT4;  //Flag set on falling edge
    P1->REN |= BIT4;  //Pullup or Pulldown enabled
    P1->OUT |= BIT4;  //Input configured as pullup

    tap = false;

    while(true)
    {
        if (tap)
        {
            tap = false;
            DelayMs(50); //debounce the tap
            if((P1->IN & BIT1) >> 1)
            {
                P2->DIR |= BIT1; //Makes button 0 an output
                P2->OUT |= BIT1; //Turns it on
                //RobotInit();
                initCC3100(Host);
                G8RTOS_AddThread(HostThread, 125, "Host Thread");
                break;
            }
            else if ((P1->IN & BIT4) >> 4)
            {
                P1->DIR |= BIT0; //Makes button 0 an output
                P1->OUT |= BIT0; //Turns it on
                Motor_Init();
                initCC3100(Client);
                G8RTOS_AddThread(ClientThread, 125, "Client Thread");
                break;
            }
        }
    }

    //Start GatorOS
    G8RTOS_Launch();
}
