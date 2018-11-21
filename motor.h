/*
 *
 *	motor.h
 *
 *	Created: 11/20/18
 *	Author: Matthew Cox
 *	Description: Function Definitions for Motor Control
 *
 *
 */
 
 #ifndef __motor_ctrl__
 #define __motor_ctrl__
 
 #include <stdio.h>
 #include <avr/io.h>
 #include <avr/interrupt.h>
 
 #define TIMER2_OUT_REGA	OCR2A
 #define TIMER2_OUT_REGB	OCR2B
 
 /* Arduino Pins are as follows
	ENA		Pin 11
	N1		Pin 10
	N2		Pin 9
	N3		Pin 7
	N4		Pin 6
	ENB		Pin 3
 */
 
 #define ENA	PINB3
 #define N1		PINB2
 #define N2		PINB1
 #define A_PORT	PORTB
 #define N3		PIND7
 #define N4		PIND6
 #define ENB	PIND3
 #define B_PORT	PORTD
 
 // Setup and initialize timers
 void pwm_timer_init(void);
 
 // Set the motors to move forward
 void moveForward(void);
 
 // Set the motors to move backwards
 void moveBackwards(void);
 
 // Set the motors to move left
 void moveLeft(void);
 
 // Set the motors to move right
 void moveRight(void);
 
 // Set the motors to stop
 void stopMotors(void);
 
 // Set the duty cycle for Motor A
 void setSpeedA(uint8_t speed);
 
 // Set the duty cycle for Motor B
 void setSpeedB(uint8_t speed);
 
 