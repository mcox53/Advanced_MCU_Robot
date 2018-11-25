/*
 * atmega328p BT33 Bluetooth Modules.c
 *
 * Created: 11/15/2018 4:02:05 PM
 * Author : Alex Valdes
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "UART.h"
#include "ADC.h"
#include "motor.h"
#include <util/delay.h>


uint16_t ADC_LR;
uint16_t ADC_UD;
uint8_t L_speed;
uint8_t R_speed;
uint8_t ADC_R;
uint8_t duty = 50;
volatile uint8_t rx[4]; // rx buffer;
int i = 0;
char conv_buff[10]; //buffer for itoa converstions

ISR(USART_RX_vect) {
	
	while (!(UCSR0A & (1<<RXC0)));
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
	OCR0A = duty;
}

ISR(TIMER0_COMPB_vect){
	OCR0B = duty;
}


int main(void){
	uart_init(); // Initialize UART
	ADC_init();
	pwm_timer_init();
	sei();
	
	//OCR0A = 0;
	//OCR0B = 0;

	OCR0A = 180;
	OCR0B = 180;
	moveForward();

	while(1){
		
// 		if(ADC_UD > 512){
// 			ADC_R = (ADC_UD - 512) / 2;
// 		}else if(ADC_UD < 512){
// 			ADC_R = (512 - ADC_UD) / 2;
// 		}
// 	
// 		duty = ADC_R;
// 		
// 		//duty = interpret_duty(ADC_UD);
// 		uint8_t dir = interpretUD_D(ADC_UD);
// 		
// 		//setSpeedA(duty);
// 		//setSpeedB(duty);
// 		
// 		if (dir == 0){
// 			moveForward();
// 		}else{
// 			moveBackwards();
// 		}
		
		
	}
}



/*

uint8_t speed = 125;

int main(void){
	DDRD |= (1 << DDD6) | (1 << DDD7);
	DDRD |= 0xFF;
	DDRB |= 0xFF;
	
	OCR0A = 255;
	OCR0B = 255;
	
	TCCR0A |= (1 << COM0A1);
	TCCR0A |= (1 << COM0B1);
	TCCR0A |= (1 << WGM01) | (1 << WGM00);
	TCCR0B |= (1 << CS01) | (1 << CS00);
	TIMSK0 |= (1 << OCIE0A) | (1 << OCIE0B);
	moveForward();
	
	while(1){

		PORTD &= ~(1 << N1);
		PORTB |= (1 << N2);
		
		// Right motor
		PORTB |= (1 << N3);
		PORTB &= ~(1 << N4);
	}
}
*/