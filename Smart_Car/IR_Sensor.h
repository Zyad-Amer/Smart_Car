/*
 * IR_Sensor.h
 *
 *  Created on: Jul 9, 2024
 *      Author: Zyad Montaser
 */

#ifndef IR_SENSOR_H_
#define IR_SENSOR_H_

#include "std_types.h"

#define detect		   0
#define Undetect       1

/* function to configure the OUT pin as input and initially output 1 (Undetect) */
void IR_Sensor_Init(uint8 IR_number_OUT_PORT_ID,uint8 IR_number_OUT_PIN_ID);

/*function to read the ir sensor response*/
uint8 IR_Sensor_Reading(uint8 IR_number_OUT_PORT_ID,uint8 IR_number_OUT_PIN_ID);

#endif /* IR_SENSOR_H_ */
