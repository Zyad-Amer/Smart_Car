/*
 * Motor.C
 *
 *  Created on: Jul 8, 2024
 *      Author: Zyad Montaser
 */

#include "Motor.h"
#include "gpio.h" /* to use GPIO_setupPinDirection and GPIO_writePin functions */
#include <avr/io.h> /* to use PORT registers */
#include "pwm.h" /* to use PWM_Timer0_Generate function to control speed of the motor */

void DcMotors_Init(void)
{
	DDRD |= (1<<PD5);	/* Make OC1A pin as output */

	/* Up_Right_DC_Motor */
	GPIO_setupPinDirection(Up_Right_DC_Motor_IN_PORT_ID,Up_Right_DC_Motor_IN1_PIN_ID,PIN_OUTPUT);
    GPIO_setupPinDirection(Up_Right_DC_Motor_IN_PORT_ID, Up_Right_DC_Motor_IN2_PIN_ID, PIN_OUTPUT);
    GPIO_writePin(Up_Right_DC_Motor_IN_PORT_ID, Up_Right_DC_Motor_IN1_PIN_ID, LOGIC_LOW);
    GPIO_writePin(Up_Right_DC_Motor_IN_PORT_ID, Up_Right_DC_Motor_IN2_PIN_ID, LOGIC_LOW);

    /* Up_Left_DC_Motor */
    GPIO_setupPinDirection(Up_Left_DC_Motor_IN_PORT_ID, Up_Left_DC_Motor_IN1_PIN_ID, PIN_OUTPUT);
    GPIO_setupPinDirection(Up_Left_DC_Motor_IN_PORT_ID, Up_Left_DC_Motor_IN2_PIN_ID, PIN_OUTPUT);
    GPIO_writePin(Up_Left_DC_Motor_IN_PORT_ID, Up_Left_DC_Motor_IN1_PIN_ID, LOGIC_LOW);
    GPIO_writePin(Up_Left_DC_Motor_IN_PORT_ID, Up_Left_DC_Motor_IN2_PIN_ID, LOGIC_LOW);

    /* Down_Right_DC_Motor */
    GPIO_setupPinDirection(Down_Right_DC_Motor_IN_PORT_ID, Down_Right_DC_Motor_IN1_PIN_ID, PIN_OUTPUT);
    GPIO_setupPinDirection(Down_Right_DC_Motor_IN_PORT_ID, Down_Right_DC_Motor_IN2_PIN_ID, PIN_OUTPUT);
    GPIO_writePin(Down_Right_DC_Motor_IN_PORT_ID, Down_Right_DC_Motor_IN1_PIN_ID, LOGIC_LOW);
    GPIO_writePin(Down_Right_DC_Motor_IN_PORT_ID, Down_Right_DC_Motor_IN2_PIN_ID, LOGIC_LOW);

    /* Down_Left_DC_Motor */
    GPIO_setupPinDirection(Down_Left_DC_Motor_IN_PORT_ID, Down_Left_DC_Motor_IN1_PIN_ID, PIN_OUTPUT);
    GPIO_setupPinDirection(Down_Left_DC_Motor_IN_PORT_ID, Down_Left_DC_Motor_IN2_PIN_ID, PIN_OUTPUT);
    GPIO_writePin(Down_Left_DC_Motor_IN_PORT_ID, Down_Left_DC_Motor_IN1_PIN_ID, LOGIC_LOW);
    GPIO_writePin(Down_Left_DC_Motor_IN_PORT_ID, Down_Left_DC_Motor_IN2_PIN_ID, LOGIC_LOW);
}

void Up_Right_DC_Motor_Rotate(DcMotor_State state/*, uint8 speed_percent*/)
{
    GPIO_setupPinDirection(Up_Right_DC_Motor_EN_PORT_ID, Up_Right_DC_Motor_EN1_PIN_ID, PIN_OUTPUT);
    GPIO_writePin(Up_Right_DC_Motor_IN_PORT_ID, Up_Right_DC_Motor_IN1_PIN_ID, state >> 1);
    GPIO_writePin(Up_Right_DC_Motor_IN_PORT_ID, Up_Right_DC_Motor_IN2_PIN_ID, state & 1);
    //Timer1_Fast_PWM_Init(speed_percent);
    //PWM_Timer0_Generate(speed_percent);
}

void Down_Right_DC_Motor_Rotate(DcMotor_State state/*, uint8 speed_percent*/)
{
    GPIO_setupPinDirection(Down_Right_DC_Motor_EN_PORT_ID, Down_Right_DC_Motor_EN1_PIN_ID, PIN_OUTPUT);
    GPIO_writePin(Down_Right_DC_Motor_IN_PORT_ID, Down_Right_DC_Motor_IN1_PIN_ID, state >> 1);
    GPIO_writePin(Down_Right_DC_Motor_IN_PORT_ID, Down_Right_DC_Motor_IN2_PIN_ID, state & 1);
    //Timer1_Fast_PWM_Init(speed_percent);
    //PWM_Timer0_Generate(speed_percent);
}

void Up_Left_DC_Motor_Rotate(DcMotor_State state/*, uint8 speed_percent*/)
{
    GPIO_setupPinDirection(Up_Left_DC_Motor_EN_PORT_ID, Up_Left_DC_Motor_EN1_PIN_ID, PIN_OUTPUT);
    GPIO_writePin(Up_Left_DC_Motor_IN_PORT_ID, Up_Left_DC_Motor_IN1_PIN_ID, state >> 1);
    GPIO_writePin(Up_Left_DC_Motor_IN_PORT_ID, Up_Left_DC_Motor_IN2_PIN_ID, state & 1);
    //PWM_Timer0_Generate(speed_percent);
    //Timer1_Fast_PWM_Init(speed_percent);
}

void Down_Left_DC_Motor_Rotate(DcMotor_State state/*, uint8 speed_percent*/)
{
    GPIO_setupPinDirection(Down_Left_DC_Motor_EN_PORT_ID, Down_Left_DC_Motor_EN1_PIN_ID, PIN_OUTPUT);
    GPIO_writePin(Down_Left_DC_Motor_IN_PORT_ID, Down_Left_DC_Motor_IN1_PIN_ID, state >> 1);
    GPIO_writePin(Down_Left_DC_Motor_IN_PORT_ID, Down_Left_DC_Motor_IN2_PIN_ID, state & 1);
    //PWM_Timer0_Generate(speed_percent);
    //Timer1_Fast_PWM_Init(speed_percent);
}
