/*
 * Motor.h
 *
 *  Created on: Jul 8, 2024
 *      Author: Zyad Montaser
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "std_types.h"

/*******************************************************************************
 *                                Definitions                                  *
 *******************************************************************************/

/* Up_Right_DC_Motor */
#define Up_Right_DC_Motor_IN_PORT        PORTC    /* Register name */
#define Up_Right_DC_Motor_IN_PORT_ID     PORTC_ID /* id of the port */
#define Up_Right_DC_Motor_IN1_PIN_ID     PIN0_ID  /* id of the pin */
#define Up_Right_DC_Motor_IN2_PIN_ID     PIN1_ID  /* id of the pin */

#define Up_Right_DC_Motor_EN_PORT_ID     PORTB_ID /* id of the port */
#define Up_Right_DC_Motor_EN1_PIN_ID     PIN3_ID  /* id of the pin */

/* Down_Right_DC_Motor */
#define Down_Right_DC_Motor_IN_PORT      PORTC    /* Register name */
#define Down_Right_DC_Motor_IN_PORT_ID   PORTC_ID /* id of the port */
#define Down_Right_DC_Motor_IN1_PIN_ID   PIN6_ID  /* id of the pin */
#define Down_Right_DC_Motor_IN2_PIN_ID   PIN7_ID  /* id of the pin */

#define Down_Right_DC_Motor_EN_PORT_ID   PORTB_ID /* id of the port */
#define Down_Right_DC_Motor_EN1_PIN_ID   PIN3_ID  /* id of the pin */

/* Up_Left_DC_Motor */
#define Up_Left_DC_Motor_IN_PORT         PORTA    /* Register name */
#define Up_Left_DC_Motor_IN_PORT_ID      PORTA_ID /* id of the port */
#define Up_Left_DC_Motor_IN1_PIN_ID      PIN0_ID  /* id of the pin */
#define Up_Left_DC_Motor_IN2_PIN_ID      PIN1_ID  /* id of the pin */

#define Up_Left_DC_Motor_EN_PORT_ID      PORTB_ID /* id of the port */
#define Up_Left_DC_Motor_EN1_PIN_ID      PIN3_ID  /* id of the pin */

/* Down_Left_DC_Motor */
#define Down_Left_DC_Motor_IN_PORT       PORTA    /* Register name */
#define Down_Left_DC_Motor_IN_PORT_ID    PORTA_ID /* id of the port */
#define Down_Left_DC_Motor_IN1_PIN_ID    PIN6_ID  /* id of the pin */
#define Down_Left_DC_Motor_IN2_PIN_ID    PIN7_ID  /* id of the pin */

#define Down_Left_DC_Motor_EN_PORT_ID    PORTB_ID /* id of the port */
#define Down_Left_DC_Motor_EN1_PIN_ID    PIN3_ID  /* id of the pin */

/*******************************************************************************
 *                               Types Declaration                             *
 *******************************************************************************/

/* enumeration defines 3 states of the DC_motor
 * IN1 --> 0  & IN2 --> 0 : stop the motor
 * IN1 --> 1  & IN2 --> 0 : rotate the motor clockwise
 * IN1 --> 0  & IN2 --> 1 : rotate the motor anticlockwise
 * */
typedef enum
{
    DcMotor_OFF, DcMotor_CW, DcMotor_A_CW
} DcMotor_State;

/*******************************************************************************
 *                              Functions Prototypes                           *
 *******************************************************************************/

/* Function responsible for setup direction of the two pins of motor and initially stop it */
void DcMotors_Init(void);

/* Function responsible for rotate the motor CW or ACW and speed */
void Up_Right_DC_Motor_Rotate(DcMotor_State state/*, uint8 speed_percent*/);
void Down_Right_DC_Motor_Rotate(DcMotor_State state/*, uint8 speed_percent*/);
void Up_Left_DC_Motor_Rotate(DcMotor_State state/*, uint8 speed_percent*/);
void Down_Left_DC_Motor_Rotate(DcMotor_State state/*, uint8 speed_percent*/);

#endif /* MOTOR_H_ */
