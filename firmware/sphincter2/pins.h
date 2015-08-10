/*
 * pins.h
 *
 * Created: 01.08.2015 03:39:29
 *  Author: phil_underscore
 */ 


#ifndef PINS_H_
#define PINS_H_

/*
 *	X_PORT		=	corresponding PORT-Register, e.g. PORTB
 *	X_DDR		=	corresponding data direction register, e.g. DDRB (outputs need this!)
 *	X_PIN		=	corresponding pin, e.g. PB0
 *	X_PINREG	=	corresponding pin register, e.g. PINB (inputs need this!)
 */


#define LED_RD_PORT PORTC			//red LED (on PCB) port
#define LED_RD_DDR DDRC				
#define LED_RD_PIN PC1				

#define LED_YL_PORT PORTB			//yellow LED port
#define LED_YL_DDR DDRB				
#define LED_YL_PIN PB1				

#define LED_BL_PORT PORTB			//blue LED port
#define LED_BL_DDR DDRB				
#define LED_BL_PIN PB2				

#define ENDSTOP_PORT PORTC			//reed contact (was:PC5)
#define ENDSTOP_PIN PC5
#define ENDSTOP_PINREG PINC			

#define DOOR_SWITCH_PORT PORTD		//safety switch: door closed? (was:PD4)
#define DOOR_SWITCH_PIN PD5
#define DOOR_SWITCH_PINREG PIND

#define DOOR_BUTTON_PORT PORTD		//pushbutton on door (was:PD5)
#define DOOR_BUTTON_PIN PD4
#define DOOR_BUTTON_PINREG PIND

#define IN0A_PORT PORTD				//input from channel 0 (no INT possible)
#define IN0A_PIN PD0
#define IN0A_PINREG PIND
#define IN0B_PORT PORTD
#define IN0B_PIN PD1
#define IN0B_PINREG PIND

#define IN1A_PORT PORTD				//input from channel 1 (INT capable)
#define IN1A_PIN PD2
#define IN1A_PINREG PIND
#define IN1B_PORT PORTD
#define IN1B_PIN PD3
#define IN1B_PINREG PIND

#define STATUS0_PORT PORTB			//status outputs
#define STATUS0_DDR DDRB
#define STATUS0_PIN PB6
#define STATUS1_PORT PORTB
#define STATUS1_DDR DDRB	
#define STATUS1_PIN PB7

#define EN_PORT PORTB				//stepper driver enable signal
#define EN_DDR DDRB					
#define EN_PIN PB0	

#define STEP_PORT PORTD				//stepper driver step signal
#define STEP_DDR DDRD
#define STEP_PIN PD7				

#define DIR_PORT PORTD				//stepper driver direction signal
#define DIR_DDR DDRD
#define DIR_PIN PD6 

#define ADC_CHAN 0					//ADC channel for battery voltage measurement



#endif /* PINS_H_ */
