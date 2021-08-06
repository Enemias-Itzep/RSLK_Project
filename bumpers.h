/*
 * bumpers.h
 *
 *  Created on: Aug 4, 2021
 *      Author: enemi
 */

 #include "threads.h"

#ifndef BUMPERS_H_
#define BUMPERS_H_



void init_Bumpers(void);

void bumper_Check(ClientData_t* c);

void PORT4_Handler(void);

#endif /* BUMPERS_H_ */
