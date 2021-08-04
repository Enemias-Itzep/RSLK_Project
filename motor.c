/*
 * motor.c
 *
 *  Created on: Jul 5, 2021
 *      Author: troth
 */

#include "motor.h"
#include <stdint.h>
#include <driverlib.h>

/*********** PRIVATE FUNCTION AND VARIABLE DECLARATIONS ***********/

//Name: calcSpeed
//Type: Private Helper Function
//Purpose: Takes in X and Y inputs and returns the speed value and direction via pointer.
//FIXME: Figure out type for PWM value. Probably uint16_t
uint16_t calcSpeed(int16_t X, int16_t Y, enum MotorDir* direction);

//Name: Motor_ChangePWM
//Type: Private Helper Function
//Purpose: Takes in a speed value
//         stops TimerA, then writes a compare values to corresponding comparison registers in TA0. Then starts TimerA
void Motor_ChangePWM(uint16_t speed);

Timer_A_CompareModeConfig TA0_compare_config = {
    TIMER_A_CAPTURECOMPARE_REGISTER_3,
    TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,
    TIMER_A_OUTPUTMODE_RESET_SET,
    0
};

/*********** PRIVATE FUNCTION AND VARIABLE DECLARATIONS ***********/


/*********** PUBLIC FUNCTION DEFINITIONS ***********/

/*********** PUBLIC FUNCTION DEFINITIONS ***********/

//Name: Motor_Init
//Type: Initialization function, call once in main
//Purpose: Initializes all of the motor pins and timers. set DIRs to be out (1) and set all output pins to 0 to initialize
//Author: Ryan Roth
void Motor_Init() {
    //Initialize DIR, SLP, and PWM pins as outputs
    DIR_port &= ~(DIRR_bm | DIRL_bm);
    SLP_port &= ~(SLPR_bm | SLPL_bm);
    PWM_port &= ~(PWMR_bm | PWML_bm);
    P5->DIR |= DIRR_bm | DIRL_bm;
    P3->DIR |= SLPL_bm | SLPR_bm;
    P2->DIR |= PWMR_bm | PWML_bm;

    //Use P2SEL0 and P2SEL1 to select the TA0 out function for the PWM pins
    P2->SEL1 &= ~(PWMR_bm | PWML_bm);
    P2->SEL0 |= PWMR_bm | PWML_bm;

    //intialize TimerA in continuous mode
    uint_fast16_t clockSource;
    uint_fast16_t clockSourceDivider;
    uint_fast16_t timerInterruptEnable_TAIE;
    uint_fast16_t timerClear;
    Timer_A_ContinuousModeConfig TA0_ModeConfig = {
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_8, //FIXME: Change to appropriate value
        TIMER_A_TAIE_INTERRUPT_DISABLE,
        TIMER_A_DO_CLEAR
    };

    Timer_A_configureContinuousMode(TIMER_A0_BASE, &TA0_ModeConfig);
    Timer_A_initCompare(TIMER_A0_BASE, &TA0_compare_config);
    TA0_compare_config.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_4;
    Timer_A_initCompare(TIMER_A0_BASE, &TA0_compare_config);
    TA0_compare_config.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3;
}

///Name: Motor_CTRL
//Type: Normal thread
//Purpose: processes new joystick values and changes motor control accordingly
//Tasks:   uses joystick running averages to determine PWM speeds (write to timers that control PWM interrupts)
//         and direction.
//         For forward and back directions on joystick:
//              Holding back should brake or reverse (DIR = 1, SLP = 1) and holding forward should move the robot forwards (SLP = 1, DIR = 0)
//         For left and right directions on joystick:
//              Holding left should set SLP to 1, DIRR to 0 and DIRL to 1
//              Holding right should set SLP to 1, DIRR to 1 and DIRL to 0
//         For holding no directions on joystick:
//              Coast, set SLP pins to 0
//         clear buffer value flag
//         Sleep thread
void Motor_CTRL(int16_t Xavg, int16_t Yavg) {
    enum MotorDir direction = COAST;
    uint16_t PWMR_val;
    uint16_t PWML_val;
    uint16_t speed = calcSpeed(Xavg, Yavg, &direction);

    switch(direction) {
    case LEFT:
        //set SLPL = 0 SLPR = 1, DIRR = 0, DIRL = 1
        DIR_port &= ~DIRR_bm;
        DIR_port |= DIRL_bm;
        SLP_port |= SLPR_bm | SLPL_bm;
        /*
        SLP_port &= ~SLPL_bm; //coast the left motor
        */

        break;
    case RIGHT:
        //set SLPL = 1, SLPR = 0, DIRR = 1, DIRL = 0
        DIR_port &= ~DIRL_bm;
        DIR_port |= DIRR_bm;
        SLP_port |= SLPL_bm | SLPR_bm;
        /*
        SLP_port &= ~SLPR_bm;
        */

        break;

    case BACK:
        //set both sleeps to 1 and both dirs to 1
        SLP_port |= SLPL_bm | SLPR_bm;
        DIR_port |= DIRL_bm | DIRR_bm;
        break;
    case FORWARD:
        SLP_port |= SLPL_bm | SLPR_bm;
        DIR_port &= ~(DIRL_bm | DIRR_bm);
        break;
    case COAST:
        SLP_port &= ~(SLPL_bm | SLPR_bm);
        break;
    default:
        break;
    }

    //FIXME:
    //use speed to determine PWM values if the speed is not 0
    if (!(speed = 0)) {
        Motor_ChangePWM(speed);
    }
}

/*********** PRIVATE FUNCTION DEFINITIONS ***********/

uint16_t calcSpeed(int16_t X, int16_t Y, enum MotorDir* direction) {
    //determine if X or Y is larger. The larger one takes precedence for direction
    //positive X is left and positive Y is down/back

    uint16_t speed;

    if (abs(X) > abs(Y)) { //determine right vs left and determine speed
        X /= 2000;
        if (X > 0) {
            *direction = LEFT;
        }
        else if (X < 0) {
            *direction = RIGHT;
        }
        else {
            *direction = COAST;
        }

        speed = abs(X);

        /*
        if (X >= LEFT1_MIN) {
            *direction = LEFT;
            if (X >= LEFT1_MAX && X <= LEFT2_MAX) {
                speed = 2;
            }
            else if (X > LEFT2_MAX) {
                speed = 3;
            }
            else {
                speed = 1;
            }
        }
        else if (X <= RIGHT1_MIN) {
            *direction = RIGHT;
            speed = 1;
            if (X <= RIGHT1_MAX && X >= RIGHT2_MAX) {
                  speed = 2;
            }
            else if (X < RIGHT2_MAX) {
                  speed = 3;
            }
            else {
                speed = 1;
            }
        }
        else {
            *direction = COAST;
            speed = 0;
        }
        */
    }
    else { //determine forward vs back and determine speed

        Y /= 2000;
        if (Y > 0) {
            *direction = BACK;
        }
        else if (Y < 0) {
            *direction = FORWARD;
        }
        else {
            *direction = COAST;
        }

        speed = abs(Y);

        /*
        if (Y >= BACK1_MIN) {
            *direction = BACK;
            speed = 1;
            if (Y <= BACK2_MAX && Y >= BACK1_MAX) {
                speed = 2;
            }
            else if (Y > BACK2_MAX){
                speed = 3;
            }
            else {
                speed = 1;
            }
        }
        else if (Y <= FORWARD1_MIN) {
            *direction = FORWARD;
            speed = 1;
            if (Y >= FORWARD2_MAX && Y <= FORWARD1_MAX) {
                speed = 2;
            }
            else if (Y < FORWARD2_MAX){
                speed = 3;
            }
            else {
                speed = 1;
            }
        }
        else {
            *direction = COAST;
            speed = 0;
        }
        */
    }
    return speed;
}

//Name: Motor_ChangePWM
//Type: Private Helper Function
//Purpose: Takes in a speed value
//         stops TimerA, then writes a compare values to corresponding comparison registers in TA0. Then starts TimerA
void Motor_ChangePWM(uint16_t speed) {
    //Stop timer A0
    Timer_A_stopTimer(TIMER_A0_BASE);

    uint16_t compareValue = SPEED1_PWM;

    //change corresponding capture register according to speed
    switch(speed) {
        case 1:
            compareValue = SPEED1_PWM;
            break;
        case 2:
            compareValue = SPEED2_PWM;
            break;
        case 3:
        case 4:
            compareValue = SPEED3_PWM;
            break;
        default:
            break;
    }

    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3, compareValue);
    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4, compareValue);

    //start Timer A
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_CONTINUOUS_MODE);
}

/*********** PRIVATE FUNCTION DEFINITIONS ***********/
