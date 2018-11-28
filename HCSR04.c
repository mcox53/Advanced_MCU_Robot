/*
 * HCSR04.c
 *
 * Created: 11/25/2018 6:12:47 PM
 *  Author: Administrator
 */ 

 #include "HCSR04.h"
 #include <stdint.h>
 #include <stdio.h>
 #include <util/delay.h>
 #include <avr/io.h>


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
	//timer1 US sensor
	TCCR1B |= (1 << CS11) | (1 << CS10) | (1 << WGM12);
	//TCCR1A |= (1 << WGM12); //CTC
	//TCCR1B |= ( 1<< CS11) | (1 << CS10); // prescaler 64
	TIMSK1 |= (1 << OCIE1A); // interrupts globally enabled
	OCR1A = 5;

	//timer2 for servo (fast PWM)
	TCCR2A |= (1 << WGM21) | (1 << WGM20) | (1 << COM2B1) | (1 << COM2B0);
	TCCR2B |= (1 << WGM22) | (1 << CS22) | (1 << CS21) | (1 << CS20);
	OCR2A = 255;
	sei();
	
	SR04_SERVO_Adjust(30);
 }

 //10 us pulse
void SR04_pulse(){
	PORTC |= (1 << TRIG); //set trig
	_delay_us(10); // for 10 us
	PORTC &= ~(1 << TRIG); //unset trig
 }

 //check echo
 Boolean SR04_ECHO_Check(){
 	if(PINC & (1 << ECHO)){
 		return true;
 	}
 	else{
 		return false;
 	}
 }

void SR04_SERVO_Adjust(double angle){
	OCR2B = 218 + (angle/180)*30; // adjust servo angle
}
