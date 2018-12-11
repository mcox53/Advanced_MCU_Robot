/*
 * main.c
 *
 * Created: 11/15/2018 4:02:05 PM
 * Author :
 */ 
 
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "UART.h"
#include "ADC.h"
#include "motor.h"
#include "HCSR04.h"
#include <util/delay.h>

// Variables for speed
uint8_t UD;
uint16_t ADC_LR;
uint16_t ADC_UD;

uint8_t L_speed_mag;
uint8_t R_speed_mag;
uint16_t UD_MAG;
uint16_t LR_MAG;

// UART variables
volatile uint8_t rx[4]; // rx buffer;
int i = 0;

// Initial Servo Angle
int servo_angle = 30;

// Counters for Timer1
volatile uint16_t counter = 0;
volatile uint8_t time_counter = 0;
volatile uint16_t servo_counter = 0;

// Distance Calculations
uint8_t distance;
uint8_t collision_flag = 0;


// When a RX interrupt occurs, read the values and increment the buffer to read all 4 values
// Message format:
// Byte 1: Joystick Up/Down movement high byte
// Byte 2: Joystick Up/Down movement low byte
// Byte 3: Joystick Left/Right movement high byte
// Byte 4: Joystick Left/Right movement low byte

ISR(USART_RX_vect) {
	
	//while (!(UCSR0A & (1<<RXC0)));
	rx[i++] = UDR0;
	if(i > 3){
		i = 0;		
		ADC_UD = rx[0];
		ADC_UD = (ADC_UD << 8) | rx[1];
		ADC_LR = rx[2];
		ADC_LR = (ADC_LR << 8) | rx[3];
	}
}


ISR(TIMER0_COMPA_vect){
	OCR0A = L_speed_mag; // speed of left wheels
}

ISR(TIMER0_COMPB_vect){
	OCR0B = R_speed_mag; // speed of right wheels
}

ISR(PCINT1_vect){
	cli();
	
	// Reset the time counter when the rising edge of the echo is received
	// Otherwise the interrupt is triggered by a falling edge of the ECHO signal
	// In this instance the number of timer periods that have elapsed is recorded
	// This number is compared to our threshold which is approximately 6 inches away
	 
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


	// Illuminate the on board LED if the object is within 6 inches (For testing)
	if(collision_flag == 0){
		PORTB &= ~(1 << PINB4);
	}else{
		PORTB |= (1 << PINB4);
	}
	sei();
	
}


ISR(TIMER1_COMPA_vect){
	// Increment various software counters
	counter++;
	time_counter++;
	servo_counter++;
}
int main(void){

	// Initialize UART, ADC, SR04 Pins and the timers for the motor
	uart_init();
	ADC_init();
	SR04_init();
	pwm_timer_init();
	sei();
	
	// PINB4 has an LED on the board which can be used for debugging signals
	DDRB |= (1 << DDB4);

	// initialize ADV values to 0;
	ADC_UD = 0;
	ADC_LR = 0;

while(1){	
		
		// Read speed and direction from ADC joysticks, move appropriate direction
		interpret_speed();
		interpret_dir();
		UD = interpretUD_D(ADC_UD);

		if (UD == 0){
			moveForward();
		}else{
			moveBackwards();
		}


		// Send out a pulse on the ultrasonic sensor ever 60 ms
		if(counter >= 300){
			SR04_pulse();
			counter = 0;
		}

		// Move the serve every 20ms and reset if the angle is too high
		if(servo_counter >= 100){
			if(servo_angle >= 150){
				servo_angle = 30;
			}
			servo_angle ++;
			SR04_SERVO_Adjust(servo_angle);
			servo_counter = 0;
		}
		

		// If something gets too close to the ultrasonic sensor, move backwards for 500 ms
		if(collision_flag == 1){
			L_speed_mag = 200;
			R_speed_mag = 200;
			moveBackwards();
			_delay_ms(500);
		}

 	}
		
}
