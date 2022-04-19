/*
 * LED_TOGGLE.h
 *
 * Created: 3/10/2022 1:25:49 PM
 *  Author: Matthew
 */ 


#ifndef LED_TOGGLE_H_
#define LED_TOGGLE_H_

#include <avr/io.h>
#define DEBUG_DDR DDRF
#define DEBUG_LED4  PORTF0
#define DEBUG_LED5  PORTF1

#define initialize_LED(ddr, pin)  ddr |= (1 << pin)    // Make pin 13 be an output.

#define LED_ON(port, pin)      port |= (1 << pin)   // Turn the LED on.

#define LED_TOGGLE(port, pin)  port ^= (1 << pin)   // Toggles the led state

#define LED_OFF(port, pin)     port &= ~(1 << pin)  // Turn the LED off.



#endif /* LED_TOGGLE_H_ */