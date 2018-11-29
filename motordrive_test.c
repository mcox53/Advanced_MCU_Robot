*
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
#include <util/delay.h>


uint16_t ADC_LR;
uint16_t ADC_UD;
double L_speed = 0;
double R_speed = 0;
double power = 0;
double L_speed_mag = 0;
double R_speed_mag = 0;
double ADC_R;
double ADC_L;
volatile uint8_t rx[4]; // rx buffer;
int i = 0;
char conv_buff[10]; //buffer for itoa converstions

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
	OCR0A = L_speed_mag;
}

ISR(TIMER0_COMPB_vect){
	OCR0B = R_speed_mag;
}


int main(void){
	uart_init(); // Initialize UART
	ADC_init();
	pwm_timer_init();
	sei();
	
	//OCR0A = 0;
	//OCR0B = 0;

	OCR0A = 150;
	OCR0B = 150;
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
	moveForward();
	PORTD &= ~(1 << N1);
	PORTB |= (1 << N2);
	
	// Right motor
	PORTB |= (1 << N3);
	PORTB &= ~(1 << N4);

while(1){	
			if(ADC_UD > 512){
				ADC_R = (ADC_UD - 512) / 2;
			}else if(ADC_UD < 512){
				ADC_R = (512 - ADC_UD) / 2;
			}

			power = (ADC_R / 255);

			if(ADC_LR > 532){
				ADC_L = (ADC_LR - 512) / 2;
				L_speed = 255;
				R_speed = 255 - ADC_L;
			}else if(ADC_LR < 492){
				ADC_L = (512 - ADC_LR) / 2;
				R_speed = 255;
				L_speed = 255 - ADC_L;
			}

			if(power < 0.05){
				power = .5;
			}

			L_speed_mag = round(power * L_speed);
			R_speed_mag = round(power * R_speed);

		//duty = interpret_duty(ADC_UD);
		uint8_t UD = interpretUD_D(ADC_UD);

		//setSpeedA(duty);
		//setSpeedB(duty);

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