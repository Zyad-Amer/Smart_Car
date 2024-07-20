/*
 * Smart_Car.c
 *
 *  Created on: Jul 8, 2024
 *      Author: Zyad Montaser
 */

#include "Smart_Car.h"
#include "Motor.c"
#include <util/delay.h> /* for delay function*/


int main()
{
	/* Enable Global Interrupt I-Bit */
	SREG |= (1<<7);

	/* set the configuration structure of the UART frame*/
	UART_ConfigType UART_Frame={Eight_Bits,Disable_Parity,One_StopBit,9600};
	UART_init(&UART_Frame);

	DcMotors_Init();

	/*initialize the four ultrasonic sensors with their pins*/

	Ultrasonic_init(front_ULTRASONIC_TIRGGER_PORT_ID,front_ULTRASONIC_TIRGGER_PIN_ID,
			        front_ULTRASONIC_ECHO_PORT_ID,front_ULTRASONIC_ECHO_PIN_ID);

	Ultrasonic_init(Back_ULTRASONIC_TIRGGER_PORT_ID,Back_ULTRASONIC_TIRGGER_PIN_ID,
			        Back_ULTRASONIC_ECHO_PORT_ID,Back_ULTRASONIC_ECHO_PIN_ID);

	Ultrasonic_init(right_ULTRASONIC_TIRGGER_PORT_ID,right_ULTRASONIC_TIRGGER_PIN_ID,
			        right_ULTRASONIC_ECHO_PORT_ID,right_ULTRASONIC_ECHO_PIN_ID);

	Ultrasonic_init(left_ULTRASONIC_TIRGGER_PORT_ID,left_ULTRASONIC_TIRGGER_PIN_ID,
			        left_ULTRASONIC_ECHO_PORT_ID,left_ULTRASONIC_ECHO_PIN_ID);

	/*initalize the three IR sensors */

	IR_Sensor_Init(center_IR_OUT_PORT_ID,center_IR_OUT_PIN_ID);
	IR_Sensor_Init(right_IR_OUT_PORT_ID,right_IR_OUT_PIN_ID);
	IR_Sensor_Init(left_IR_OUT_PORT_ID,left_IR_OUT_PIN_ID);

	/* generate the 2 pwm signals from timer 0 and timer 2*/
	PWM_Timer0_Generate(AVERAGE_SPEED); /*for the left motors*/
	PWM_Timer2_Generate(AVERAGE_SPEED); /*for the right motors*/

	while(1)
	{
		// Receiving the instructions from Bluetooth Module
		Choice = UART_recieveByte();
		switch(Choice)
		{
			case FORWARD_CASE:
				Move_Forward();
				break;

			case BACKWARD_CASE:
				Move_Backward();
				break;

			case RIGHT_CASE:
				PWM_Timer0_Generate(AVERAGE_SPEED); // LEFT Motors
				PWM_Timer2_Generate(LOWEST_SPEED); //  RIGHT Motors
				Move_Right();
				break;

			case LEFT_CASE:
				PWM_Timer0_Generate(LOWEST_SPEED);	  // LEFT Motors
				PWM_Timer2_Generate(AVERAGE_SPEED);  //  RIGHT Motors
				Move_Left();
				break;

			case STOP_CASE:
				Stop();
				break;

			// Change The Mode to Automatic Mode
			case AUTO_MODE:
				AutoMode = 1;
				break;

			// Change The Mode to Line Follower Mode
			case LINE_FOLLOWER_MODE:
				Line_Follower_Mode = 1;
				break;

			case MINIMUM_SPEED_PWM:
				PWM_Timer0_Generate(MINIMUM_SPEED);
				PWM_Timer2_Generate(MINIMUM_SPEED);
				_delay_ms(10);
				break;

			case MAXIMUM_SPEED_PWM:
				PWM_Timer0_Generate(MAXIMUM_SPEED);
				PWM_Timer2_Generate(MAXIMUM_SPEED);
				_delay_ms(10);
				break;

			case AVERAGE_SPEED_PWM:
				PWM_Timer0_Generate(AVERAGE_SPEED);
				PWM_Timer2_Generate(AVERAGE_SPEED);
				_delay_ms(10);
				break;

			case DEFAULT_SPEED_PWM:
				PWM_Timer0_Generate(DEFAULT_SPEED_PWM);
				PWM_Timer2_Generate(DEFAULT_SPEED_PWM);
				_delay_ms(10);
				break;
			}

		/*******************************************************************************
		 *                             Automatic Mode                           *
		 *******************************************************************************/
		while(AutoMode)/* If the Automode flag is set*/
		{
			/* Store the readings of the four ultrasonic sensors */
			Front_UltraSonic_Reading=Ultrasonic_readDistance(front_ULTRASONIC_TIRGGER_PORT_ID,
															 front_ULTRASONIC_TIRGGER_PIN_ID);

			Back_UltraSonic_Reading  = Ultrasonic_readDistance(Back_ULTRASONIC_TIRGGER_PORT_ID,
															   Back_ULTRASONIC_TIRGGER_PIN_ID);

			Right_UltraSonic_Reading = Ultrasonic_readDistance(right_ULTRASONIC_TIRGGER_PORT_ID,
															   right_ULTRASONIC_TIRGGER_PIN_ID);

			Left_UltraSonic_Reading  = Ultrasonic_readDistance(left_ULTRASONIC_TIRGGER_PORT_ID,
															   left_ULTRASONIC_TIRGGER_PIN_ID);

			/* If the front view is empty move the car to forward*/
			if(Front_UltraSonic_Reading > DISTANCE_THRESHOLD)
			{
				Move_Forward();
			}
			else
			{
				/* If the right view is empty move the car to right*/
				if(Right_UltraSonic_Reading > DISTANCE_THRESHOLD)
				{
					Move_Right();
				}

				/* If the left view is empty move the car to left*/
				else if(Left_UltraSonic_Reading >DISTANCE_THRESHOLD)
				{
					Move_Left();
				}

				/*if there is an object in right and left move the car to backward*/
				else if((Left_UltraSonic_Reading <DISTANCE_THRESHOLD) && (Right_UltraSonic_Reading > DISTANCE_THRESHOLD))
				{
					Move_Backward();
				}

				/*if all the view is busy stop the car*/
				else
				{
					Stop();
				}
			}
			_delay_ms(100);/*delay 100ms to retake the readings*/

			Choice = UART_recieveByte();/*check if the Automode flag is changed */
			if(Choice == MANUAL_MODE)
			{
				AutoMode = 0;/* clear the automatic flag*/
				Stop();
				break;
			}

			/*******************************************************************************
			 *                             Line follower Mode                           *
			 *******************************************************************************/

			 while(Line_Follower_Mode)
			 {
				  Center_IR_Reading = IR_Sensor_Reading(center_IR_OUT_PORT_ID,center_IR_OUT_PIN_ID);
				  Right_IR_Reading  = IR_Sensor_Reading(right_IR_OUT_PORT_ID,right_IR_OUT_PIN_ID);
				  Left_IR_Reading   = IR_Sensor_Reading(left_IR_OUT_PORT_ID,left_IR_OUT_PIN_ID);

				  if(Center_IR_Reading == BLACK)
				  {
					  Move_Forward();
				  }

				  else if((Left_IR_Reading == WHITE && Center_IR_Reading == WHITE && Right_IR_Reading == BLACK)|| (Left_IR_Reading == WHITE && Center_IR_Reading == BLACK && Right_IR_Reading == BLACK))
				  {
					  Move_Right();
				  }

				  else if((Left_IR_Reading == BLACK && Center_IR_Reading == WHITE && Right_IR_Reading == WHITE) || (Left_IR_Reading == BLACK && Center_IR_Reading == BLACK && Right_IR_Reading == WHITE))
				  {
					  Move_Left();
				  }

				  //Rotate Right continuously
				  else if(Left_IR_Reading == WHITE && Center_IR_Reading == WHITE && Right_IR_Reading == WHITE)
				  {
					  Move_Right();
				  }

				  _delay_ms(100); /*delay 100ms to retake the readings*/

				  Choice = UART_recieveByte();

				  if(Choice == MANUAL_MODE)
				  {
					  Line_Follower_Mode = 0;/* clear the line follower mode flag*/
					  Stop();
					  break;
				  }
			 }
		}

/*operate all the motors with the same speed and clockwise*/
void Move_Forward(void)
{
	Up_Right_DC_Motor_Rotate(DcMotor_CW);
	Down_Right_DC_Motor_Rotate(DcMotor_CW);

	Up_Left_DC_Motor_Rotate(DcMotor_CW);
	Down_Left_DC_Motor_Rotate(DcMotor_CW);
}

/*operate all the motors with the same speed and anti-clockwise*/
void Move_Backward(void)
{
	Up_Right_DC_Motor_Rotate(DcMotor_A_CW);
	Down_Right_DC_Motor_Rotate(DcMotor_A_CW);

	Up_Left_DC_Motor_Rotate(DcMotor_A_CW);
	Down_Left_DC_Motor_Rotate(DcMotor_A_CW);
}

  /* operate the left motors with the lower speed and clockwise
  * and the right motors with the higher speed and also clockwise*/
void Move_Left(void)
{
	Up_Right_DC_Motor_Rotate(DcMotor_CW);
	Down_Right_DC_Motor_Rotate(DcMotor_CW);

	Up_Left_DC_Motor_Rotate(DcMotor_A_CW);
	Down_Left_DC_Motor_Rotate(DcMotor_A_CW);
}

/* operate the right motors with the lower speed and clockwise
 * and the left motors with the higher speed and also clockwise
 * */
void Move_Right(void)
{
	Up_Right_DC_Motor_Rotate(DcMotor_A_CW);
	Down_Right_DC_Motor_Rotate(DcMotor_A_CW);

	Up_Left_DC_Motor_Rotate(DcMotor_CW);
	Down_Left_DC_Motor_Rotate(DcMotor_CW);
}

/*stop all the motors*/
void Stop(void)
{
	Up_Right_DC_Motor_Rotate(DcMotor_OFF);
	Down_Right_DC_Motor_Rotate(DcMotor_OFF);

	Up_Left_DC_Motor_Rotate(DcMotor_OFF);
	Down_Left_DC_Motor_Rotate(DcMotor_OFF);
}
