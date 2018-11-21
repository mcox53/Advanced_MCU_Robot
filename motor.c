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
 
 #include "motor.c"
 
 void pwm_timer_init(void){
	
	// Clear OC0A on Compare Match
	TCCR2A |= (1 << COM2A1); 
	// Clear OC0B on Compare Match
	TCCR2A |= (1 << COM2B1);
	// Fast PWM mode w/ TOP = 0xFF
	TCCR2A = (1 << WGM01) | (1 << WGM00);
	// Set Prescaler to XXX
	
	// Enable Compare Match Interrupts for OC0A and OC0B
	TIMSK2 |= (1 << OCIE2A) | (1 << OCIE2B);
 }
 
 // If N1 is HIGH and N2 is LOW -> Motor A Backwards
 // If N1 is LOW and N2 is HIGH -> Motor A Forwards
 // If N3 is HIGH and N4 is LOW -> Motor B Backwards
 // If N3 is LOW and N4 is HIGH -> Motor B Forwards
 
void moveForward(void){
	A_PORT &= ~(1 << N1);
	A_PORT |= (1 << N2);
	B_PORT &= ~(1 << N3);
	B_PORT |= (1 << N4);
}

void moveBackwards(void){
	A_PORT |= (1 << N1);
	A_PORT &= ~(1 << N2);
	B_PORT |= (1 << N3);
	B_PORT &= ~(1 << N4);
}

// To move left either disable the left motor completely or cut speed separately
// This function just properly sets the right motor direction
void moveLeft(void){
	B_PORT &= ~(1 << N3);
	B_PORT |= (1 << N4);
}

// To move right either disable the right motor completely or cut speed separately
// This function just properly sets the left motor direction
void moveRight(void){
	A_PORT &= ~(1 << N1);
	A_PORT |= (1 << N2);
}

// It is assumed that the provided value is 0-255
void setSpeedA(uint8_t speed){
	TIMER2_OUT_REGA = speed;
}

// It is assumed that the provided value is 0-255
void setSpeedB(uint8_t speed){
	TIMER2_OUT_REGB = speed;
}

// According to the L298N datasheet if the directional inputs are the same
// the motor will stop whether the PWM signal is high or not
// When PWM signal is HIGH && N1 = N2 -> Fast motor stop
// When PWM signal is LOW && N1 = N2 -> Free running motor stop
// Speed should still be set to zero to stop
void stopMotors(void){
	A_PORT |= (1 << N1);
	A_PORT |= (1 << N2);
	B_PORT |= (1 << N3);
	B_PORT |= (1 << N4);
	setSpeedA(0);
	setSpeedB(0);
}