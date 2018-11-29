/*
 * UltraSonic.c
 *
 * Created: 11/25/2018 6:12:15 PM
 * Author : Administrator
 */ 
 #define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "HCSR04.h"

// Initial Servo Angle
int servo_angle = 30; 

volatile uint16_t counter = 0;
volatile uint8_t time_counter = 0;
volatile uint16_t servo_counter = 0;

uint8_t distance;
uint8_t collision_flag = 0;

void init_all(){
	SR04_init();
	sei();
	
	// PINB4 has an LED on the board which can be used for debugging signals
	DDRB |= (1 << DDB4);
}

ISR(PCINT1_vect){
	cli();
	if(PINC & (1 << ECHO)){
		time_counter = 0;
	}
	else{
		distance = time_counter;
		if(distance < 7){
			collision_flag = 1;
		}
		else{
			collision_flag = 0;
		}
	}

	if(collision_flag == 0){
		PORTB &= ~(1 << PINB4);
	}else{
		PORTB |= (1 << PINB4);
	}
	sei();
	
}

ISR(TIMER1_COMPA_vect){
	counter++;
	time_counter++; 
	servo_counter++;
}

int main(void)
{
    init_all();
    while (1) 
    {	

		if(counter >= 300){
			SR04_pulse();
			counter = 0;
		}

		if(servo_counter >= 100){
			if(servo_angle >= 150){
				servo_angle = 30;
			}
			servo_angle ++;
			SR04_SERVO_Adjust(servo_angle);
			servo_counter = 0;
		}
     }
}

