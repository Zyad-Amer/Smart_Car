 /******************************************************************************
 *
 * Module: ICU
 *
 * File Name: icu.h
 *
 * Description: Header file for the AVR ICU driver
 *
 * Author: Zyad Montaser
 *
 *******************************************************************************/
#include "std_types.h"

#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

/*******************************************************************************
 *                         Types Declaration                                   *
 *******************************************************************************/

/*******************************************************************************
 *                      Functions Prototypes                                   *
 *******************************************************************************/

/*function responsible for initialize the icu and set its callback function */
void Ultrasonic_init(uint8 ULTRASONIC_number_TIRGGER_PORT_ID,
					 uint8 ULTRASONIC_number_TIRGGER_PIN_ID,
					 uint8 ULTRASONIC_number_ECHO_PORT_ID,
					 uint8 ULTRASONIC_number_ECHO_PIN_ID);

/*function responsible for send trigger pulse on the trigger pin*/
void Ultrasonic_Trigger(uint8 ULTRASONIC_number_TIRGGER_PORT_ID,uint8 ULTRASONIC_number_TIRGGER_PIN_ID);

/*function responsible for calculate the distance*/
uint16 Ultrasonic_readDistance(uint8 ULTRASONIC_number_TIRGGER_PORT_ID,uint8 ULTRASONIC_number_TIRGGER_PIN_ID);

/*call back function which is responsible for calculate the high time (pulse time) on echo pin*/
void Ultrasonic_edgeProcessing(void);



#endif /* ULTRASONIC_H_ */
