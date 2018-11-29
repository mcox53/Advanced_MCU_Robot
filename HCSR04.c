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
	DDRC &= ~(1 << ECHO);							// Set echo as input
	DDRC |= (1 << TRIG);							// Set trig as output
	PORTC |= (1 << ECHO);							// Initialize

	//Servo init
	DDRD |= (1 << SERVO);							// Set servo as input
	
	//PCINT1
	PCMSK1 |= (1 << ECHO);
	PCICR |= (1 << PCIE1);

	// Timer Init
	// Timer 1: 200 microsecond period. 
	// Prescaler = 8
	TCCR1B |= (1 << WGM12);							// CTC
	TCCR1B |= (1 << CS11);							// Prescaler 8
	TIMSK1 |= (1 << OCIE1A);						// Timer 1 output compare match interrupt A
	OCR1A = 399;

	// Timer 2 for servo
	TCCR2A |= (1 << WGM21) | (1 << WGM20) | (1 << COM2B1) | (1 << COM2B0);
	TCCR2B |= (1 << WGM22) | (1 << CS22) | (1 << CS21) | (1 << CS20);
	OCR2A = 255;
	
	SR04_SERVO_Adjust(30);
 }

 // 10 us pulse
void SR04_pulse(){
	cli();
	PORTC |= (1 << TRIG);							// Set trig
	_delay_us(10);									// For 10 us
	PORTC &= ~(1 << TRIG);							// Unset trig
	sei();
 }


void SR04_SERVO_Adjust(double angle){
	// Calculate angle
	OCR2B = 218 + (angle / 180)*30;					
}