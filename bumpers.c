/*
 * bumpers.c
 *
 *  Created on: Aug 4, 2021
 *      Author: enemi
 */
#include <stdint.h>
#include <G8RTOS.h>
#include <driverlib.h>
#include "bumpers.h"
#include "threads.h"

volatile bool tap4 = false;

void PORT4_Handler(void)
{
    //Clear IFG flag
    P4->IFG &= ~BIT0;
    P4->IFG &= ~BIT2;
    P4->IFG &= ~BIT3;
    P4->IFG &= ~BIT5;
    P4->IFG &= ~BIT6;
    P4->IFG &= ~BIT7;

    tap4 = true;
}



void init_Bumpers(void)
{
    //Bumper 0
    P4->DIR &= ~BIT0; //Makes button 0 an input
    P4->IES |= BIT0;  //Flag set on falling edge
    P4->REN |= BIT0;  //Pullup or Pulldown enabled
    P4->OUT |= BIT0;  //Input configured as pullup
    P4->IFG &= ~BIT0; //Clears interrupt flag for button 0
    P4->IE |= BIT0;   //Enables interrupt for Button 0

    //Bumper 1
    P4->DIR &= ~BIT2; //Makes button 0 an input
    P4->IES |= BIT2;  //Flag set on falling edge
    P4->REN |= BIT2;  //Pullup or Pulldown enabled
    P4->OUT |= BIT2;  //Input configured as pullup
    P4->IFG &= ~BIT2; //Clears interrupt flag for button 0
    P4->IE |= BIT2;   //Enables interrupt for Button 0

    //Bumper 2
    P4->DIR &= ~BIT3; //Makes button 0 an input
    P4->IES |= BIT3;  //Flag set on falling edge
    P4->REN |= BIT3;  //Pullup or Pulldown enabled
    P4->OUT |= BIT3;  //Input configured as pullup
    P4->IFG &= ~BIT3; //Clears interrupt flag for button 0
    P4->IE |= BIT3;   //Enables interrupt for Button 0

    //Bumper 3
    P4->DIR &= ~BIT5; //Makes button 0 an input
    P4->IES |= BIT5;  //Flag set on falling edge
    P4->REN |= BIT5;  //Pullup or Pulldown enabled
    P4->OUT |= BIT5;  //Input configured as pullup
    P4->IFG &= ~BIT5; //Clears interrupt flag for button 0
    P4->IE |= BIT5;   //Enables interrupt for Button 0

    //Bumper 4
    P4->DIR &= ~BIT6; //Makes button 0 an input
    P4->IES |= BIT6;  //Flag set on falling edge
    P4->REN |= BIT6;  //Pullup or Pulldown enabled
    P4->OUT |= BIT6;  //Input configured as pullup
    P4->IFG &= ~BIT6; //Clears interrupt flag for button 0
    P4->IE |= BIT6;   //Enables interrupt for Button 0

    //Bumper 5
    P4->DIR &= ~BIT7; //Makes button 0 an input
    P4->IES |= BIT7;  //Flag set on falling edge
    P4->REN |= BIT7;  //Pullup or Pulldown enabled
    P4->OUT |= BIT7;  //Input configured as pullup
    P4->IFG &= ~BIT7; //Clears interrupt flag for button 0
    P4->IE |= BIT7;   //Enables interrupt for Button 0
}

void bumper_Check(ClientData_t* c)
{
    if(tap4)
    {
        tap4 = false;

        c->newBump = true;

        c->bump0 = false;
        c->bump1 = false;
        c->bump2 = false;
        c->bump3 = false;
        c->bump4 = false;
        c->bump5 = false;

        if (!((P4->IN & BIT0) >> 0))
        {
            c->bump0 = true;
        }
        if (!((P4->IN & BIT2) >> 2))
        {
            c->bump1 = true;
        }
        if (!((P4->IN & BIT3) >> 3))
        {
            c->bump2 = true;
        }
        if (!((P4->IN & BIT5) >> 5))
        {
            c->bump3 = true;
        }
        if (!((P4->IN & BIT6) >> 6))
        {
            c->bump4 = true;
        }
        if (!((P4->IN & BIT7) >> 7))
        {
            c->bump5 = true;
        }
    }
}

