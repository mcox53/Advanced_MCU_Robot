/*
 *
 *	motor.c
 *
 *	Created: 11/20/18
 *	Author: Matthew Cox
 *	Description: Function Definitions for Motor Control
 *
 *
 */
 
 #include "motor.h"
  
 void pwm_timer_init(void){
	 
	DDRD |= (1 << ENA) | (1 << N1) | (1 << ENB);
	DDRB |= (1 << N2) | (1 << N3) | (1 << N4);
	
	// Clear OC0A on Compare Match
	TCCR0A |= (1 << COM0A1); 
	// Clear OC0B on Compare Match
	TCCR0A |= (1 << COM0B1);
	// Fast PWM mode w/ TOP = 0xFF
	TCCR0A = (1 << WGM01) | (1 << WGM00);
	// Set Prescaler to 8
	TCCR0A |= (1 << CS21);
	// Enable Compare Match Interrupts for OC0A and OC0B
	TIMSK0 |= (1 << OCIE0A) | (1 << OCIE0B);
 }
 
 // If N1 is HIGH and N2 is LOW -> Motor A Backwards
 // If N1 is LOW and N2 is HIGH -> Motor A Forwards
 // If N3 is HIGH and N4 is LOW -> Motor B Backwards
 // If N3 is LOW and N4 is HIGH -> Motor B Forwards
 
void moveBackwards(void){
	PORTD &= ~(1 << N1);
	PORTB |= (1 << N2);
	PORTB &= ~(1 << N3);
	PORTB |= (1 << N4);
}

void moveForward(void){
	PORTD |= (1 << N1);
	PORTB &= ~(1 << N2);
	PORTB |= (1 << N3);
	PORTB &= ~(1 << N4);
}

// To move left either disable the left motor completely or cut speed separately
// This function just properly sets the right motor direction
void moveLeft(void){
	PORTB &= ~(1 << N3);
	PORTB |= (1 << N4);
}

// To move right either disable the right motor completely or cut speed separately
// This function just properly sets the left motor direction
void moveRight(void){
	PORTD &= ~(1 << N1);
	PORTB |= (1 << N2);
}

// It is assumed that the provided value is 0-255
void setSpeedA(uint8_t speed){
	TIMER0_OUT_REGA = speed;
}

// It is assumed that the provided value is 0-255
void setSpeedB(uint8_t speed){
	TIMER0_OUT_REGB = speed;
}

// According to the L298N datasheet if the directional inputs are the same
// the motor will stop whether the PWM signal is high or not
// When PWM signal is HIGH && N1 = N2 -> Fast motor stop
// When PWM signal is LOW && N1 = N2 -> Free running motor stop
// Speed should still be set to zero to stop
void stopMotors(void){
	PORTD |= (1 << N1);
	PORTB |= (1 << N2);
	PORTB |= (1 << N3);
	PORTB |= (1 << N4);
	setSpeedA(0);
	setSpeedB(0);
}

uint8_t interpret_duty(uint16_t ADC_val){
	
	uint8_t ADC_R;
	
	if(ADC_R > 512){
		ADC_R = (ADC_val - 512) / 2;
	}else if(ADC_R < 512){
		ADC_R = (512 - ADC_val) / 2;
	}
	return ADC_R;
}

uint8_t interpretLR_D(uint16_t ADC_LR){
	// 1 if Left , 0 if Right
	uint8_t dir_flag;
	
	if(ADC_LR > 512){
		dir_flag = 0;
	}else{
		dir_flag = 1;
	}
	return dir_flag;
}

uint8_t interpretUD_D(uint16_t ADC_UD){
	// 1 if backwards, 0 if forwards
	uint8_t dir_flag;
	
	if(ADC_UD > 512){
		dir_flag = 0;
	}else{
		dir_flag = 1;
	}
	return dir_flag;
}
