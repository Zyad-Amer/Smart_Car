/*
 * IR_Sensor.c
 *
 *  Created on: Jul 9, 2024
 *      Author: Zyad Montaser
 */

#include "IR_Sensor.h"
#include "gpio.h"

void IR_Sensor_Init(uint8 IR_number_OUT_PORT_ID,uint8 IR_number_OUT_PIN_ID)
{
	/* configure the OUT pin as input */
	GPIO_setupPinDirection(IR_number_OUT_PORT_ID,IR_number_OUT_PIN_ID,PIN_INPUT);
}

uint8 IR_Sensor_Reading(uint8 IR_number_OUT_PORT_ID,uint8 IR_number_OUT_PIN_ID)
{
	/* if the ir sensor detect an object */
	if(GPIO_readPin(IR_number_OUT_PORT_ID,IR_number_OUT_PIN_ID)==LOGIC_LOW)
	{
		return detect;
	}
	else
	{
		return Undetect;
	}
}
