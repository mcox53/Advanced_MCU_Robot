/*
 * HCSR04.c
 *
 * Created: 11/25/2018 6:12:47 PM
 *  Author: Administrator
 */ 

 #define F_CPU 16000000UL

 #include "HCSR04.h"
 #include <stdint.h>
 #include <stdio.h>
 #include <util/delay.h>
 #include <avr/io.h>
 #include <avr/interrupt.h>


void SR04_init(){
	//SR04 init
	DDRC &= ~(1 << ECHO); //set echo as output
	PORTC |= (1 << ECHO); //initialize
	PCMSK1 |= (1 << ECHO);
	PCICR |= (1 << PCIE1);
	DDRC |= (1 << TRIG); //set trig as input

	//Servo init
	DDRD |= (1 << SERVO); //set servo as input
	
	//ISR inits
	//timer1  200 microseconds timer. 
	// Ftimer = 4 Khz corresponds to 200 us period
	// Prescaler = 1
	TCCR1A |= (1 << WGM12); //CTC
	TCCR1B |= (1 << CS10); // prescaler 1
	TIMSK1 |= (1 << OCIE1A); // Timer 1 output compare match interrupt A
	OCR1A = 1999;

	//timer2 for servo (fast PWM)
	TCCR2A |= (1 << WGM21) | (1 << WGM20) | (1 << COM2B1) | (1 << COM2B0);
	TCCR2B |= (1 << WGM22) | (1 << CS22) | (1 << CS21) | (1 << CS20);
	OCR2A = 255;
	sei();
	
	SR04_SERVO_Adjust(30);
 }

 //10 us pulse
void SR04_pulse(){
	cli();
	PORTC |= (1 << TRIG); //set trig
	_delay_us(10); // for 10 us
	PORTC &= ~(1 << TRIG); //unset trig
	sei();
 }

 //check echo
 int SR04_ECHO_Check(){
 	if(PINC & (1 << ECHO)){
 		return 1;
 	}
 	else{
 		return 0;
 	}
 }

void SR04_SERVO_Adjust(double angle){
	OCR2B = 218 + (angle/180)*30; // adjust servo angle
}