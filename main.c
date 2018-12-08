/*
 * atmega328p BT33 Bluetooth Modules.c
 *
 * Created: 11/15/2018 4:02:05 PM
 * Author : Alex Valdes
 */ 
 //6 & 5 duty cycle
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "UART.h"
#include "ADC.h"
#include "motor.h"
#include "HCSR04.h"
#include <util/delay.h>

uint8_t UD; // up down mag
uint16_t ADC_LR;
uint16_t ADC_UD;
double L_speed = 0;
double R_speed = 0;
double power = 0;
double L_speed_mag = 0;
double R_speed_mag = 0;
double UD_MAG = 512;
double LR_MAG = 512;
volatile uint8_t rx[4]; // rx buffer;
int i = 0;
char conv_buff[10]; //buffer for itoa converstions

// Initial Servo Angle
int servo_angle = 30;

// Counters for Timer1
volatile uint16_t counter = 0;
volatile uint8_t time_counter = 0;
volatile uint16_t servo_counter = 0;

// Distance Calculations
uint8_t distance;
uint8_t collision_flag = 0;

ISR(USART_RX_vect) {
	
	//while (!(UCSR0A & (1<<RXC0)));
	rx[i++] = UDR0;
	if(i > 3){
		i = 0;		
		ADC_UD = rx[0];
		ADC_UD = (ADC_UD << 8) | rx[1];
		ADC_LR = rx[2];
		ADC_LR = (ADC_LR << 8) | rx[3];
// 		itoa(ADC_UD,conv_buff,10);
// 		USART_Transmit_String(conv_buff);
// 		USART_Transmit(':');
// 		itoa(ADC_LR,conv_buff,10);
// 		USART_Transmit_String(conv_buff);
// 		USART_Transmit('\r');
// 		USART_Transmit('\n');

	}
}


ISR(TIMER0_COMPA_vect){
	OCR0A = L_speed_mag; // speed of right wheels
}

ISR(TIMER0_COMPB_vect){
	OCR0B = R_speed_mag; // speed of left wheels
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
int main(void){
	uart_init(); // Initialize UART
	ADC_init();
	SR04_init();
	//pwm_timer_init();
	sei();
	
	// PINB4 has an LED on the board which can be used for debugging signals
	DDRB |= (1 << DDB4);

	//OCR0A = 0;
	//OCR0B = 0;

	DDRD |= (1 << DDD6) | (1 << DDD7);
	DDRD |= 0xFF;
	DDRB |= 0xFF;

	OCR0A = 150;
	OCR0B = 150;

	TCCR0A |= (1 << COM0A1);
	TCCR0A |= (1 << COM0B1);
	TCCR0A |= (1 << WGM01) | (1 << WGM00);
	TCCR0B |= (1 << CS01) | (1 << CS00);
	TIMSK0 |= (1 << OCIE0A) | (1 << OCIE0B);

	// Reset bluetooth buffers
	ADC_UD = 0;
	ADC_LR = 0;
	//moveForward();
	//moveBackwards();

while(1){	
			// ADC_UD Stationary: 593/594    UP: 1016/1017  DOWN: 178/180 
			// ADC_LR Stationary: 593/594    LEFT: 177/178  RIGHT: 1022/1023 
			if(ADC_UD > 514){//604){ // Joystick is up
				UD_MAG = (ADC_UD - 512) / 2;
			}else if(ADC_UD < 494){ //584){ // Joystick is down ? right?
				UD_MAG = (494 - ADC_UD) / 2;
// 				if(UD_MAG == 255){
// 					UD_MAG = 255;
// 				}
			}else{ // Dead Zone
				UD_MAG = 0;
			}


			if(UD_MAG == 0){
				L_speed_mag = 0;
				R_speed_mag = 0;
			}else if(ADC_LR > 514){			// Right turn
				LR_MAG = (ADC_LR - 512) / 2;
				R_speed_mag = UD_MAG;
				L_speed_mag = 255 - LR_MAG;
			}else if(ADC_LR < 494){		// Left turn
				LR_MAG = (494 - ADC_LR) / 2;
// 				if(LR_MAG == 255){
// 					LR_MAG = 255;
// 				}
				L_speed_mag = UD_MAG;
				R_speed_mag = 255 - LR_MAG;
			}else{
				R_speed_mag = UD_MAG;
				L_speed_mag = UD_MAG;
			}




  			//L_speed_mag = UD_MAG; 
 			//R_speed_mag = UD_MAG; 

// 			L_speed_mag = round(power * L_speed);
// 			R_speed_mag = round(power * R_speed);

		//duty = interpret_duty(ADC_UD);

		UD = interpretUD_D(ADC_UD);

		//setSpeedA(duty);
		//setSpeedB(duty);

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
			
		if(collision_flag == 1){
			L_speed_mag = 200;
			R_speed_mag = 200;
			moveBackwards();
			_delay_ms(500);
		}

		if (UD == 0){
			moveForward();
		}else{
			moveBackwards();
		}
 	}
		
		
}



/*

uint8_t speed = 125;

int main(void){
	
}
*/