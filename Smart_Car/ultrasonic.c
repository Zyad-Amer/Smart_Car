 /******************************************************************************
 *
 * Module: ICU
 *
 * File Name: icu.c
 *
 * Description: Source file for the AVR ICU driver
 *
 * Author: Zyad Montaser
 *
 *******************************************************************************/

#include "ultrasonic.h"
#include "icu.h"
#include"gpio.h" /*to use GPIO_setupPinDirection function*/
#include <util/delay.h> /* for delay function*/

/********************************************************************************
 * 								Global variables
 ********************************************************************************/
uint8 g_edgeCount = 0;
uint16 g_timeHigh = 0;
uint16 g_distance=0;
/*******************************************************************************
 *                      Functions Definitions                                  *
 *******************************************************************************/

/*function responsible for intitialize the icu and set its callback function */
void Ultrasonic_init(uint8 ULTRASONIC_number_TIRGGER_PORT_ID,
					 uint8 ULTRASONIC_number_TIRGGER_PIN_ID,
					 uint8 ULTRASONIC_number_ECHO_PORT_ID,
					 uint8 ULTRASONIC_number_ECHO_PIN_ID)
{
	/* Create configuration structure for ICU driver */
	ICU_ConfigType ICU_Configurations = {F_CPU_8,RAISING};

	/* Configure the direction for trigger pin as output pin */
	GPIO_setupPinDirection(ULTRASONIC_number_TIRGGER_PORT_ID,ULTRASONIC_number_TIRGGER_PIN_ID,PIN_OUTPUT);

	/* Configure ECHO pin as input pin (this line not necessary because icu do it automatically)*/
	GPIO_setupPinDirection(ULTRASONIC_number_ECHO_PORT_ID,ULTRASONIC_number_ECHO_PIN_ID,PIN_INPUT);

	/* Initialize the ICU driver by passing the configuration structure to it*/
	ICU_init(&ICU_Configurations);

	/* Set the Call back function pointer in the ICU driver */
	ICU_setCallBack(Ultrasonic_edgeProcessing);
}

/*function responsible for send trigger pulse on the trigger pin*/
void Ultrasonic_Trigger(uint8 ULTRASONIC_number_TIRGGER_PORT_ID,uint8 ULTRASONIC_number_TIRGGER_PIN_ID)
{
	/* output a trigger pulse on trigger pin for 10 us */
	GPIO_writePin(ULTRASONIC_number_TIRGGER_PORT_ID,ULTRASONIC_number_TIRGGER_PIN_ID,LOGIC_HIGH);
	_delay_us(10);
	GPIO_writePin(ULTRASONIC_number_TIRGGER_PORT_ID,ULTRASONIC_number_TIRGGER_PIN_ID,LOGIC_LOW);
}

/*function responsible for calculate the distance*/
uint16 Ultrasonic_readDistance(uint8 ULTRASONIC_number_TIRGGER_PORT_ID,uint8 ULTRASONIC_number_TIRGGER_PIN_ID)
{
	/*send the trigger pulse on the trigger pin*/
	 Ultrasonic_Trigger(ULTRASONIC_number_TIRGGER_PORT_ID,ULTRASONIC_number_TIRGGER_PIN_ID);

	/* timer increment every 1us
	 * distance=(340000*timer*1*10^-6)/2 cm =0.017*timer cm =timer/58.8 cm
	 * */
	g_distance=g_timeHigh/58;

	return g_distance;
}

/*call back function which is responsible for calculate the high time (pulse time) on echo pin*/
void Ultrasonic_edgeProcessing(void)
{
	g_edgeCount++;
	if(g_edgeCount == 1)/*here ,detected the rising edge*/
	{
		ICU_clearTimerValue();/*start calculate the time from here*/

		/* Detect falling edge */
		ICU_setEdgeDetectionType(FALLING);
	}
	else if(g_edgeCount == 2)/*here,detected the falling edge*/
	{
		/*store the timer value (time take by the ultrasonic wave to travel and return)*/
		g_timeHigh=ICU_getInputCaptureValue();

		/* Detect rising edge */
		ICU_setEdgeDetectionType(RAISING);

		/*clear the edge counter for the new measurement*/
		g_edgeCount=0;
	}
}
