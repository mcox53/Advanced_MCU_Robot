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