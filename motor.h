/*
 * motor.h
 *
 *  Created on: Jul 5, 2021
 *      Author: troth
 */

#ifndef DRIVERS_MOTOR_MOTOR_H_
#define DRIVERS_MOTOR_MOTOR_H_

#include "msp.h"
#include "BSP.h"

/*********** DEFINITIONS ***********/

//motor ports
#define DIR_port P5->OUT //DIR on port 5
#define PWM_port P2->OUT //PWM on port 2
#define SLP_port P3->OUT //SLP on port 3

//Motor pin bitmasks
#define DIRR_bm BIT5
#define DIRL_bm BIT4
#define PWMR_bm BIT6
#define PWML_bm BIT7
#define SLPR_bm BIT6
#define SLPL_bm BIT7

//SPEED PWM VALUES
//FIXME: determine appropriate values
#define SPEED1_PWM  0xFFFF / 3
#define SPEED2_PWM  0xFFFF * 5 / 3
#define SPEED3_PWM  0xFFFF * 11 / 10

enum MotorDir {LEFT, RIGHT, FORWARD, BACK, COAST};

//6 speeds for straight, 3 speeds forward, 3 speeds back
//FIXME: Determine values
/*
#define FORWARD1_MIN -2500
#define FORWARD1_MAX -4500
#define FORWARD2_MAX -6300

#define BACK1_MIN 2500
#define BACK1_MAX 4500
#define BACK2_MAX 6300

//6 speeds for turning, 3 speeds for each turning direction
#define LEFT1_MIN 2500
#define LEFT1_MAX 4500
#define LEFT2_MAX 6300

#define RIGHT1_MIN -2000
#define RIGHT1_MAX -4000
#define RIGHT2_MAX -5500
*/

//U refers to duty cycle

/*********** DEFINITIONS ***********/

/*********** PUBLIC FUNCTIONS ***********/

//Name: Motor_Init
//Type: Initialization function, call once in main
//Purpose: Initializes all of the motor pins and timers. set DIRs to be out (1) and set all output pins to 0 to initialize
//Author: Ryan Roth
void Motor_Init();

//Name: Motor_CTRL
//Type: Normal thread
//Purpose: processes new joystick values and changes motor control accordingly
//Tasks:   check joystick value flag from buffer. Sleep if there are no new values
//         takes joystick values and averages them together (running avg) to determine PWM speeds (write to timers that control PWM interrupts)
//         and direction.
//         For forward and back directions on joystick:
//              Holding back should brake or reverse (DIR = 1, SLP = 1) and holding forward should move the robot forwards (SLP = 1, DIR = 0)
//         For left and right directions on joystick:
//              Holding left should set SLP to 1, DIRR to 0 and DIRL to 1
//              Holding right should set SLP to 1, DIRR to 1 and DIRL to 0
//         For holding no directions on joystick:
//              Coast, set SLP pins to 0
//         clear buffer value flag
//
void Motor_CTRL(int16_t Xavg, int16_t Yavg);

/*********** PUBLIC FUNCTIONS ***********/

#endif /* DRIVERS_MOTOR_MOTOR_H_ */
