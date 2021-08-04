#include "msp.h"
#include <G8RTOS.h>
#include <driverlib.h>
#include "BSP.h"
#include <cc3100_usage.h>
#include "demo_sysctl.h"
#include "threads.h"
#include "motor.h"
#include "image.h"
#include "bumpers.h"

#define OFF 0x00
#define ON 0xFF

volatile bool tap1 = false;

void PORT1_IRQHandler(void)
{
    //Clear IFG flag
    P1->IFG &= ~BIT4;
    P1->IFG &= ~BIT1;

    tap1 = true;
}



/**
 * main.c
 */
void main(void)
{
    //Initialize G8RTOS
    G8RTOS_Init();

    NVIC_EnableIRQ(PORT1_IRQn);
    NVIC_EnableIRQ(PORT4_IRQn);

    //Configures the button
    P1->DIR &= ~BIT1; //Makes button 0 an input
    P1->IES |= BIT1;  //Flag set on falling edge
    P1->REN |= BIT1;  //Pullup or Pulldown enabled
    P1->OUT |= BIT1;  //Input configured as pullup
    P1->IFG &= ~BIT1; //Clears interrupt flag for button 0
    P1->IE |= BIT1;   //Enables interrupt for Button 0

    //Configures the button
    P1->DIR &= ~BIT4; //Makes button 0 an input
    P1->IES |= BIT4;  //Flag set on falling edge
    P1->REN |= BIT4;  //Pullup or Pulldown enabled
    P1->OUT |= BIT4;  //Input configured as pullup
    P1->IFG &= ~BIT4; //Clears interrupt flag for button 0
    P1->IE |= BIT4;   //Enables interrupt for Button 0

    tap1 = false;

    while(true)
    {
        if (tap1)
        {
            tap1 = false;
            DelayMs(50); //debounce the tap
            if(!((P1->IN & BIT1) >> 1))
            {
                P2->DIR |= BIT1; //Makes button 0 an output
                P2->OUT |= BIT1; //Turns it on

                LCD_Init(false);

                uint16_t xOffset = 80;
                uint16_t yOffset = 172;
                uint16_t size = 4;

                uint16_t k = 0;
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

                initCC3100(Host);
                G8RTOS_AddThread(HostThread, 125, "Host Thread");
                break;
            }
            else if (!((P1->IN & BIT4) >> 4))
            {
                P1->DIR |= BIT0; //Makes button 0 an output
                P1->OUT |= BIT0; //Turns it on
                Motor_Init();
                init_Bumpers();
                initCC3100(Client);
                G8RTOS_AddThread(ClientThread, 125, "Client Thread");
                break;
            }
        }
    }

    //Start GatorOS
    G8RTOS_Launch();
}
