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
uint16_t ADC_LR;
uint16_t ADC_UD;
volatile uint8_t rx[4]; // rx buffer;
int i = 0;
ISR(USART_RX_vect) {
	while(UCSR0A & (1<<RXC0)){
		rx[i++] = UDR0;
	}
	if(i > 3){
		i = 0;
		//USART_Transmit_String(rx); //for testing through ftdi
		ADC_LR = ADC_LR | rx[3];
		ADC_LR = (ADC_LR<<8) | rx[2];
		ADC_UD = ADC_UD | rx[1];
		ADC_UD = (ADC_UD<<8) | rx[0];
	}
}



int main(void){
	uart_init(); // Initialize UART
	ADC_init();
	pwm_timer_init();
	sei();
	while(1){
		
		setSpeedA(125);
		setSpeedB(125);
		
			PORTD |= (1 << N1);
			PORTB &= ~(1 << N2);
			PORTB |= (1 << N3);
			PORTB &= ~(1 << N4);
		
		//moveForward();
		
		/*
		uint8_t duty = interpret_duty(ADC_UD);
		uint8_t dir = interpretUD_D(ADC_UD);
		
		setSpeedA(duty);
		setSpeedB(duty);
		
		if (dir = 0){
			moveForward();
		}else{
			moveBackwards();
		}
		*/
	}
}
