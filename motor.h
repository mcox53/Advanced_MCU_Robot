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
 
 #define TIMER0_OUT_REGA	OCR0A
 #define TIMER0_OUT_REGB	OCR0B
 
 /* Arduino Pins are as follows
	ENA		Pin 5
	N1		Pin 7
	N2		Pin 8
	N3		Pin 9
	N4		Pin 11
	ENB		Pin 6
 */
 
 #define ENA	PIND5
 #define N1		PIND7
 #define N2		PINB0
 #define N3		PINB1
 #define N4		PINB3
 #define ENB	PIND6
 
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
 
 // 
 uint8_t interpret_duty(uint16_t ADC_val);
 
 uint8_t interpretLR_D(uint16_t ADC_LR);
 
 uint8_t interpretUD_D(uint16_t ADC_UD);
 
 #endif