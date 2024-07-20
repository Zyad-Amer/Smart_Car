/******************************************************************************
 *
 * Module: PWM
 *
 * File Name: pwm.c
 *
 * Description: Source file for the ATmega32 PWM driver
 *
 * Author: Zyad Montaser
 *
 *******************************************************************************/

#include <avr/io.h> /* To use PWM/Timer0 Registers */
#include "pwm.h"
#include "gpio.h" /* to use GPIO_setupPinDirection function*/

/* function responsible for generate pwm signal using timer0 with different duty cycles*/
void PWM_Timer0_Generate(uint8 duty_cycle)
{

	TCNT0 = 0; // Set Timer Initial Value to 0

	if(duty_cycle <100)
		OCR0  = (uint8)( ((uint32)duty_cycle*256)/100 ); //Set Compare value
	else if (duty_cycle ==100)
		OCR0=255; /*to avoid the overflow (256 exceed the range of uint8)*/

	GPIO_setupPinDirection(pwm0_PORT_ID,pwm0_PIN_ID,PIN_OUTPUT); // Configure PB3/OC0 as output pin --> pin where the PWM signal is generated from MC

	/* Configure timer control register
	 * 1. Fast PWM mode FOC0=0
	 * 2. Fast PWM Mode WGM01=1 & WGM00=1
	 * 3. Clear OC0 when match occurs (non inverted mode) COM00=0 & COM01=1
	 * 4. clock = F_CPU/8 CS00=0 CS01=1 CS02=0
	 */
	TCCR0 = (1<<WGM00) | (1<<WGM01) | (1<<COM01) | (1<<CS01);
}

/* function responsible for generate pwm signal using timer2 with different duty cycles*/
void PWM_Timer2_Generate(uint8 duty_cycle)
{

	TCNT2 = 0; // Set Timer Initial Value to 0

	if(duty_cycle <100)
		OCR2  = (uint8)( ((uint32)duty_cycle*256)/100 ); //Set Compare value
	else if (duty_cycle ==100)
		OCR2=255; /*to avoid the overflow (256 exceed the range of uint8)*/

	GPIO_setupPinDirection(pwm2_PORT_ID,pwm2_PIN_ID,PIN_OUTPUT); // Configure PD7/OC2 as output pin --> pin where the PWM signal is generated from MC

	/* Configure timer control register
	 * 1. Fast PWM mode FOC0=0
	 * 2. Fast PWM Mode WGM21=1 & WGM20=1
	 * 3. Clear OC2 when match occurs (non inverted mode) COM20=0 & COM21=1
	 * 4. clock = F_CPU/8 CS20=0 CS21=1 CS22=0
	 */
	TCCR2 = (1<<WGM20) | (1<<WGM21) | (1<<COM21) | (1<<CS21);
}

/*Function responsible for initializing the timer1 for pwm in fast mode */
void PWM_vInit_T1_FastMode()
{
	
		//we made it CLR on compare to get the desired correct signal for servo
		// Make OC1A pin as output
		GPIO_setupPinDirection(PORTD_ID, PIN_NO_5,OUTPUT);
		//checked  //Set timer1 count zero
		TCNT1 = 0 ;			
		//for frequency 50Hz  // Set TOP count for timer1 in ICR1 register
		ICR1 = 2499 ; 
		//Set Fast PWM, TOP in ICR1, Clr OC1A on compare match, Clk/64
		TCCR1A = (1<<PIN_NO_7) |  (1<<PIN_NO_1) ;
		TCCR1B = (1<<PIN_NO_3) | (1<<PIN_NO_4) | (1<<PIN_NO_0) | (1<<PIN_NO_1) ;

	
}

/*Function responsible for assining a value in ICR1 REG*/
void PWM_vSetValue_ICR1_T1_Manual(uint16 copy_u16Value)
{
	
	ICR1 = copy_u16Value ;
	
}

/*Function responsible for assining a value in OCR1A REG */
void PWM_vSetValue_OCR1A_T1_Manual(uint16 copy_u16Value)
{
		OCR1A = copy_u16Value ;
}

void Timer1_Fast_PWM_Init(unsigned short duty_cycle)
{
	TCNT1 = 0;		/* Set timer1 initial count to zero */
	ICR1 = 2499;	/* Set TOP count for timer1 in ICR1 register */

	OCR1A = duty_cycle; /* Set the compare value */

	/* Configure timer control register TCCR1A
     * 1. Clear OC1A on compare match (non inverting mode) COM1A1=1 COM1A0=0
     * 2. Disconnect OC1B  COM1B0=0 COM1B1=0
     * 3. FOC1A=0 FOC1B=0 because these bits are only active in case non-pwm mode
     * 4. Fast Pwm Mode with the TOP in ICR1 WGM10=0 WGM11=1 (Mode Number 14)
	 */
	TCCR1A = (1<<WGM11) | (1<<COM1A1);

	/* Configure timer control register TCCR1B
	 * 1. Fast Pwm Mode with the TOP in ICR1 WGM12=1 WGM13=1 (Mode Number 14)
	 * 2. Prescaler = F_CPU/64
     */
	TCCR1B = (1<<WGM12) | (1<<WGM13) | (1<<CS10) | (1<<CS11);
}
