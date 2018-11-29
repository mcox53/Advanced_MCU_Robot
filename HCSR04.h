/*
 * HCSR04.h
 *
 * Created: 11/25/2018 6:13:02 PM
 *  Author: Administrator
 */ 

  #define TRIG PORTC5
  #define ECHO PORTC4
  #define SERVO PORTD3


 // Initialize
 void SR04_init();

 // Pulse Sensor
 void SR04_pulse();
 
 // Move Servo
 void SR04_SERVO_Adjust(double angle);