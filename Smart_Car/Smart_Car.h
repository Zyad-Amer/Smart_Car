/*
 * Smart_Car.h
 *
 *  Created on: Jul 8, 2024
 *      Author: Zyad Montaser
 */

#ifndef SMART_CAR_H_
#define SMART_CAR_H_

#include "Motor.h"
#include "std_types.h"
#include "common_macros.h"
#include "gpio.h"
#include "IR_Sensor.h"
#include "ultrasonic.h"
#include "pwm.h"
#include "lcd.h"
#include "uart.h"
/*******************************************************************************
 *                                Definitions                                  *
 *******************************************************************************/
/* Car Cases To Move*/
#define FORWARD_CASE	   'F'
#define BACKWARD_CASE	   'B'
#define RIGHT_CASE		   'R'
#define LEFT_CASE		   'L'
#define STOP_CASE          'S'

/* Car Modes */
#define MANUAL_MODE        'a'
#define AUTO_MODE          'A'
#define LINE_FOLLOWER_MODE 'I'

/* Car Speed */
#define LOWEST_SPEED        20
#define MINIMUM_SPEED		40
#define DEFAULT_SPEED		60
#define AVERAGE_SPEED		80
#define MAXIMUM_SPEED		90

/* Car Speed Mode */
#define MINIMUM_SPEED_PWM   'Q'
#define MAXIMUM_SPEED_PWM   'H'
#define DEFAULT_SPEED_PWM   'D'
#define AVERAGE_SPEED_PWM   'M'

/* -------------------------------------- */
#define DISTANCE_THRESHOLD 25
/* -------------------------------------- */

/* IR Macros */
#define BLACK 1
#define WHITE 0

#define center_IR_OUT_PORT_ID      PORTB_ID
#define center_IR_OUT_PIN_ID	   PIN1_ID

#define right_IR_OUT_PORT_ID       PORTB_ID
#define right_IR_OUT_PIN_ID	       PIN6_ID

#define left_IR_OUT_PORT_ID        PORTB_ID
#define left_IR_OUT_PIN_ID	       PIN0_ID

/*front ultrasonic*/
#define front_ULTRASONIC_TIRGGER_PORT_ID   PORTB_ID
#define front_ULTRASONIC_TIRGGER_PIN_ID    PIN5_ID
#define front_ULTRASONIC_ECHO_PORT_ID      PORTD_ID
#define front_ULTRASONIC_ECHO_PIN_ID	   PIN6_ID

/* Back ultrasonic*/
#define Back_ULTRASONIC_TIRGGER_PORT_ID    PORTC_ID
#define Back_ULTRASONIC_TIRGGER_PIN_ID     PIN3_ID
#define Back_ULTRASONIC_ECHO_PORT_ID       PORTC_ID
#define Back_ULTRASONIC_ECHO_PIN_ID		   PIN2_ID

/*right ultrasonic*/
#define right_ULTRASONIC_TIRGGER_PORT_ID   PORTA_ID
#define right_ULTRASONIC_TIRGGER_PIN_ID    PIN2_ID
#define right_ULTRASONIC_ECHO_PORT_ID      PORTA_ID
#define right_ULTRASONIC_ECHO_PIN_ID	   PIN3_ID

/*left ultrasonic*/
#define left_ULTRASONIC_TIRGGER_PORT_ID    PORTB_ID
#define left_ULTRASONIC_TIRGGER_PIN_ID     PIN4_ID
#define left_ULTRASONIC_ECHO_PORT_ID       PORTB_ID
#define left_ULTRASONIC_ECHO_PIN_ID		   PIN2_ID

/*******************************************************************************
 *                              Global Variables                               *
 *******************************************************************************/
// unsigned char Right_Motors_Speed = DEFAULT_SPEED;
// unsigned char left_Motors_Speed = DEFAULT_SPEED;
/* */

/* -------------------------------------- */
uint8 Choice; // Bluetooth Module reading
/* UltraSonic Reading */
uint8 Front_UltraSonic_Reading;
uint8 Back_UltraSonic_Reading;
uint8 Right_UltraSonic_Reading;
uint8 Left_UltraSonic_Reading;

/* -------------------------------------- */
/* Car Switch Mode */
uint8 AutoMode = 0; // flag
uint8 Line_Follower_Mode = 0;

/* IR Sensors */
uint8 Center_IR_Reading;
uint8 Left_IR_Reading;
uint8 Right_IR_Reading;

/*******************************************************************************
 *                             Functions Prototypes                            *
 *******************************************************************************/
void Move_Forward(void);
void Move_Backward(void);
void Move_Left_Or_Right(void);
void Stop(void);

void Move_Left(void);
void Move_Right(void);

#endif /* SMART_CAR_H_ */
