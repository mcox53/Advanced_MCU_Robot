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
 
 
 // Motor B -> Right
 // Motor A -> Left
  
void pwm_timer_init(void){
	
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
}
 
 // If N1 is HIGH and N2 is LOW -> Motor A Backwards
 // If N1 is LOW and N2 is HIGH -> Motor A Forwards
 // If N3 is HIGH and N4 is LOW -> Motor B Backwards
 // If N3 is LOW and N4 is HIGH -> Motor B Forwards
 
void moveBackwards(void){
	PORTD &= ~(1 << N1);
	PORTB |= (1 << N2);
	PORTB |= (1 << N3);
	PORTB &= ~(1 << N4);
}

void moveForward(void){
	PORTD |= (1 << N1);
	PORTB &= ~(1 << N2);
	PORTB &= ~(1 << N3);
	PORTB |= (1 << N4);
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
	PORTD |= (1 << N1);
	PORTB &= ~(1 << N2);
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
// 	setSpeedA(0);
// 	setSpeedB(0);
}

void interpret_speed(void){

	// ADC_UD Stationary: 593/594    UP: 1016/1017  DOWN: 178/180
	// ADC_LR Stationary: 593/594    LEFT: 177/178  RIGHT: 1022/1023

	// Interpret speed based on up/down joystick position
	// Stationary positions for joystick were not 512 so deadzone is taken into account

	if(ADC_UD > 514){
		UD_MAG = (ADC_UD - 512) / 2;
	}else if(ADC_UD < 494){
		UD_MAG = (494 - ADC_UD) / 2;
	}else{
		UD_MAG = 0;
	}	


}


void interpret_dir(void){

	// Move left and right based on the joystick values and the up/down magnitude
	// Simple tank drive system.

	if(UD_MAG == 0){
		L_speed_mag = 0;
		R_speed_mag = 0;
	}else if(ADC_LR > 514){			
		LR_MAG = (ADC_LR - 512) / 2;
		R_speed_mag = UD_MAG;
		L_speed_mag = 255 - LR_MAG;
	}else if(ADC_LR < 494){		
		LR_MAG = (494 - ADC_LR) / 2;
		L_speed_mag = UD_MAG;
		R_speed_mag = 255 - LR_MAG;
	}else{
		R_speed_mag = UD_MAG;
		L_speed_mag = UD_MAG;
	}

}

uint8_t interpretUD_D(uint16_t ADC_UD){
	// 1 if backwards, 0 if forwards
	
	uint8_t dir_flag;
	
	if(ADC_UD > 594){
		dir_flag = 0;
	}else{
		dir_flag = 1;
	}
	return dir_flag;
}
