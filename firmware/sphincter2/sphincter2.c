/*
 * sphincter2.c	- OpenLab Augsburg Türzugangskontrollsystem
 *
 * Created: 01.08.2015 02:47:08
 *  Author: phil_underscore
 *
 * Designed for ATmega8. Might be used with newer ATmega48/88/168.33
 * You need to change interrupt vectors and some registers in
 * setup(), though. Look for the corresponding atmel app note regarding
 * code migration from ATmega8 to ATmega48/88/168 for more information.
 *
 * This code needs some optimization. It works, but it is not pretty.
 *
 */ 

#define F_CPU 8000000
#define NEUTRAL 1030	//how many steps do we need from reference (locked) position to neutral?
#define OPEN 1750		//how many steps do we need from reference position to open?
#define LOCKED 0		//how many steps do we need from reference position to locked?

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "pins.h"


/* these variables need to work inside a ISR, thats why they are global and labeled "volatile" */
	volatile uint8_t ms_tick = 0;
	volatile uint8_t ms_10_tick = 0;
	volatile uint8_t ms_100_tick = 0;
	volatile uint8_t LED_tick = 0;
	volatile uint8_t delayed_lock_tick = 0;

/* used to carry command flags from timer ISR to main loop */
	volatile struct
	{
		unsigned check_supply:1;
		unsigned check_inputs:1;
		unsigned allow_step:1;
	} ISR_flag;
	
	volatile struct
	{
		unsigned unlock:1;
		unsigned lock:1;
		unsigned delayed_lock:1;
		unsigned reference:1;
		unsigned block_lock:1;
		unsigned block_unlock:1;
	} command;
	
	volatile struct
	{
		unsigned door_closed:1;
		unsigned locked:1;
		unsigned unlocked:1;
		unsigned delayed_lock:1;
		unsigned open:1;
		unsigned voltage_low:1;
		unsigned error:1;
		unsigned reached_endstop:1;
		unsigned move_in_progress:1;
	} status;

/* input status is needed by a lot of functions and they must clear the "input_status" bit once finished.
 * Making these variables global makes the code easier to read and more comfortable to work with 
 * 
 * Now more or less obsolete, could use some rework! (10/Aug/15)
 */
	int8_t input_status_previous[7];	//used to detect falling edges on input channels
	int8_t input_status_current[7];		//used to detect falling edges on input channels
	uint8_t input_status[7];		//this holds information about falling input edges
	
	int16_t position = 0;		//this is the stepper motors position (in steps). we use this to keep track of our movements


/* setup routine, prepare the MCU for operation */
	void setup(void)	
{
	/* setup data direction registers */
	
		/* set LED pins to output */
		LED_RD_DDR |= (1<<LED_RD_PIN);
		LED_YL_DDR |= (1<<LED_YL_PIN);
		LED_BL_DDR |= (1<<LED_BL_PIN);
		
		/* set status signal pins to output */
		STATUS0_DDR |= (1<<STATUS0_PIN);
		STATUS1_DDR |= (1<<STATUS1_PIN);
		
		/* set stepper driver signals to output */
		DIR_DDR |= (1<<DIR_PIN);
		STEP_DDR |= (1<<STEP_PIN);
		
		/* special setup for stepper EN-signal: active low, init with high level */
		EN_PORT |= (1<<EN_PIN);		/* enable pullup - pin is still a input */
		EN_DDR |= (1<<EN_PIN);		/* set pin to output - port register still set, output is now at high level */
		
		
	/* setup pull-up resistors for inputs */
	
		/* inputs from optical interface (IN0A-IN1B) */
		IN0A_PORT |= (1<<IN0A_PIN);
		IN0B_PORT |= (1<<IN0B_PIN);
		IN1A_PORT |= (1<<IN1A_PIN);
		IN1B_PORT |= (1<<IN1B_PIN);
		
		/* inputs from door switches */
		DOOR_BUTTON_PORT |= (1<<DOOR_BUTTON_PIN);
		ENDSTOP_PORT |= (1<<ENDSTOP_PIN);
		DOOR_SWITCH_PORT |= (1<<DOOR_SWITCH_PIN);
		
	/* init arrays and variables to 0 */
		
		for(uint8_t i=0;i<7;i++)
		{
			input_status[i]=0;
			input_status_current[i]=0;
			input_status_previous[i]=0;
		}
		
		ISR_flag.check_inputs = 0;
		ISR_flag.check_supply = 0;
		ISR_flag.allow_step = 0;
		
		command.unlock = 0;
		command.lock = 0;
		command.delayed_lock = 0;
		command.reference = 0;
		command.block_lock = 0;
		command.block_unlock = 0;

		status.door_closed = 0;
		status.locked = 0;
		status.delayed_lock = 0;
		status.unlocked = 0;
		status.open = 0;
		status.voltage_low = 0;
		status.error = 0;
		status.reached_endstop = 0;
		status.move_in_progress = 0;
		
	
	
	/* init ADC */
		
		/* set channel by writing channel number to ADMUX */
		ADMUX = ADC_CHAN;
		
		/* 
		 * Enable internal 2.56V reference, left adjust result. 
		 * We only need 8bit resolution here, so we can just read the high byte.
		 * That allows us to use interrupts without troublesome 16bit read operations. 
		 */
		ADMUX |= (1<<REFS1) | (1<<REFS0) | (1<<ADLAR);
		
		/* set prescaler for 125kHz ADC clock, enable ADC */
		ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADEN);
		
		
	/* init timer */
		
		/* timer 2 (systick) */
		/* set to CTC, prescaler 64 */
		TCCR2 |= (1<<WGM21) | (1<<CS22);	
		
		/* clear on compare match, set to 125 -> 1ms period */
		OCR2 = 125;
		
		/* enable compare match interrupts from timer2 */
		TIMSK |= (1<<OCIE2);
	
	/* enable global interrupts */
	
		sei();
}

/* switch a LED on or off. R, Y, B selects the appropriate LED. */
	void switch_LED (uint8_t onoff, char LED)
{
	if(onoff)
	{
		switch(LED)
		{
			case 'R': LED_RD_PORT |= (1<<LED_RD_PIN); break;
			case 'B': LED_BL_PORT |= (1<<LED_BL_PIN); break;
			case 'Y': LED_YL_PORT |= (1<<LED_YL_PIN); break;
			default: break;
		}
	}else{
		switch(LED)
		{
			case 'R':LED_RD_PORT &= ~(1<<LED_RD_PIN); break;
			case 'B':LED_BL_PORT &= ~(1<<LED_BL_PIN); break;
			case 'Y':LED_YL_PORT &= ~(1<<LED_YL_PIN); break;
			default: break;
		}
	}
}

/* flash a LED. R, Y, B selects the appropriate LED. time is in 10ms intervals */
	void flash_LED(uint8_t ontime, uint8_t offtime, char LED) 
{
	if(LED_tick <= ontime)
	{
		switch_LED(1,LED);
		}else{
		switch_LED(0,LED);
	}
	
	if(LED_tick >= offtime)
	{
		LED_tick = 0;
	}
}

/* measure supply voltage */
	uint8_t read_ADC(void)		
{
	uint8_t result = 0;
	
	/* start a conversion */
	ADCSRA |= (1<<ADSC);
	
	/* wait for the conversion to finish */
	while (ADCSRA & (1<<ADSC) ) {}
	
	/* read ADC data register high byte */
	/* Remember, we only use 8 bit and discard the two least significant digits */
	result = ADCH;
	
	/* 
	 * You can insert some code here to average the result over a number of conversions
	 * if you have problems with accuracy and EMI. Try filtering the ADC input first, though!
	 */
	
	return(result);
}

/* check 12V supply voltage and warn user if it is too low */
	void check_supply_voltage(void)
{
	/*
	 *	Voltage threshold: Device is powered by a lead battery.
	 *	Warn user if voltage drops below 11.5V
	 *	Increase warning intensity if voltage drops below 11V
	 *
	 *	The input voltage divider is formed by a 100k and a 22k resistor. 
	 *	
	 *	| V_in	| V_div	| ADC	|
	 *	| 13.8V	| 2.49V | 249	|	
	 *	| 11.5V | 2.07V | 207	|
	 *	| 11.0V | 1.98V | 198	|
	 * 
	 */
	
	uint8_t ADC_result = read_ADC();
	int8_t corr = 7;	//correction for non-ideal resistors in the divider
	
	if(ADC_result > (207+corr))
	{
		switch_LED(0,'R');						//Everything is fine, no need to warn
	}
	else if(ADC_result > (198+corr))
	{
		flash_LED(2,100,'R');	//Supply voltage is low, check battery charger
	}else
	{
		flash_LED(2,30,'R');	//Supply voltage is dangerously low, user must act to prevent battery damage
	}
	
}

/* poll all inputs and determine if any falling edges are present 
 * Polling the inputs once every 1ms is a compromise between debouncing 
 * physical switches and responding to fast(ish) signals on the digital inputs
 * This means that a digital input signal on In0 or In1 needs to last at LEAST 1.5ms 
 * to be detected correctly.
 */
	void poll_inputs(void)
{
	
	/* save input status from last poll */
		for(uint8_t i=0;i<7;i++)
		{
			input_status_previous[i] = input_status_current[i];
		}
		
	/* poll the inputs */
		input_status_current[0] = (IN0A_PINREG & (1<<IN0A_PIN));	//lock the door, input0
		input_status_current[1] = (IN0B_PINREG & (1<<IN0B_PIN));	//unlock the door, input0
		input_status_current[2] = (IN1A_PINREG & (1<<IN1A_PIN));	//lock the door, input1
		input_status_current[3] = (IN1B_PINREG & (1<<IN1B_PIN));	//unlock the door, input1
		input_status_current[4] = (DOOR_BUTTON_PINREG & (1<<DOOR_BUTTON_PIN));	//auto-lock after 10 seconds
		input_status_current[5] = (DOOR_SWITCH_PINREG & (1<<DOOR_SWITCH_PIN));	//check if the door is closed
		input_status_current[6] = (ENDSTOP_PINREG & (1<<ENDSTOP_PIN));	//used to reference the door lock
	
	/* look for falling edges */
		for(uint8_t i=0;i<7;i++)
		{
			if((input_status_previous[i]) && (!input_status_current[i]))
			{
				input_status[i]=1;
			}
		}
		
		if(input_status[0])		//falling edge on "lock" signal lines
		{
			command.lock = 1;
			input_status[0] = 0;
		}
		if(input_status[1])		//falling edge on "unlock" signal lines
		{
			command.unlock = 1;
			input_status[1] = 0;
		}
		if(input_status[2])		//falling edge on "lock" signal lines
		{
			command.lock = 1;
			input_status[2] = 0;
		}
		if(input_status[3])		//falling edge on "unlock" signal lines
		{
			command.unlock = 1;
			input_status[3] = 0;
		}
		if(input_status[6])		//falling edge on "endstop" signal lines
		{
			status.reached_endstop = 1;
			input_status[6] = 0;
		}
}


/* Move the stepper motor _one_ step in your chosen direction
 * Warning: This will _not_ enable the motor, you have to do this yourself!
 *
 * Direction == 0: Motor turns CW, big gear turns CCW
 * direction == 1: Motor turns CCW, big gear turns CW
 *
 * Returns 1 after successful move or 0 if 1ms delay condition has not been met
 */
	uint8_t move_stepper(uint8_t direction)
{
	uint8_t time = 0;	//used to measure the "on"time for the step signal. In this case about 8µs
	
	if(ISR_flag.allow_step)
	{
		if(direction<=0)	//set direction
		{
			DIR_PORT |= (1<<DIR_PIN);
		}else{
			DIR_PORT &= ~(1<<DIR_PIN);
		}
		
		time = TCNT2;			//get current timer value
		while(time==TCNT2){}	//wait for the timer value to change
		time = TCNT2;			//get "fresh" timer value. one timer clock cycle lasts about 8 µs
		while(time==TCNT2)		//pull the step-pin high for one timer clock cycle (8µs)
		{
			STEP_PORT |= (1<<STEP_PIN);
		}
		STEP_PORT &= ~(1<<STEP_PIN);
		
		ISR_flag.allow_step = 0;	//wait at least 1ms until the next step and do something else in between
		return 1;	//success, motor turned one step!
	}
	return 0;	//Failure, motor did not turn. Wait a bit longer until movement is allowed again.
}

/* this will enable the stepper driver. Do not forget to turn it off! */
	void enable_stepper(uint8_t enable)
{
	if(enable)
	{
		EN_PORT &= ~(1<<EN_PIN);
	}else{
		EN_PORT |= (1<<EN_PIN);
	}
}

/* find the reference point in the locked-position. we need to to this at least every time the unit is powered on */
	void find_reference(uint8_t state, uint16_t max_steps_CW, uint16_t max_steps_CCW)
{
	enable_stepper(1);	//enable motors or you won't be able to do anything
	
	/* this means, that the gear is already near the reed contact and triggers the endstop switch
	 * we need to turn the gear CW to get a clear reference point later
	 */
		if(state==1)	
		{
			while(max_steps_CW)
			{
				max_steps_CW = max_steps_CW - move_stepper(1);
			}
			
		}
		
	/* we are now far enough from the endstop switch and can now begin the process
	 * There are 2 conditions to break the loop: Either all steps have been used up (error!)
	 * or we found the reference point!
	 */
		while((max_steps_CCW > 0) && (ENDSTOP_PINREG & (1<<ENDSTOP_PIN)))
		{
			max_steps_CCW = max_steps_CCW - move_stepper(0);
		}
	
	/* check if there are any steps left. If so, set endstop_reached status. If not, set error status
	 * this is necessary because we did not allow any other functions while referencing. That, in turn,
	 * is necessary to make sure the reference operations is not interrupted or otherwise messed with 
	 */
		if(max_steps_CCW){
			status.locked = 1;
			status.unlocked = 0;
			status.error = 0;
			status.open = 0;
			position = 0;
			
			ms_100_tick = 0;
			while(ms_100_tick<=5){}
			/* move to neutral position */
			while(position < NEUTRAL)
			{
				position = position + move_stepper(1);
			}			

		}else{
			status.error = 1;
			status.locked = 0;
			status.unlocked = 0;
			status.open = 0;
		}
		
		/* disable the motor once we finished */
			enable_stepper(0);
			status.reached_endstop = 0;
}

/* move key to a specific position (in steps counted from reference point) */
	uint8_t move_to_position(int16_t target)
{
	if(target>position)
	{
		position = position + move_stepper(1);
	}
	if(target<position)
	{
		position = position - move_stepper(0);
	}
	
	if(target==position)
	{
		return 1;
	}else{
		return 0;
	}
}

/* Set the status0/1 signals according to door status
 * 'U': unlocked
 * 'L': locked
 * 'E': error
 */
	void set_status (char door_status)
	{
		switch(door_status)
		{
			case 'L': STATUS0_PORT |= (1<<STATUS0_PIN); STATUS1_PORT &= ~(1<<STATUS1_PIN); break;
			case 'U': STATUS1_PORT |= (1<<STATUS1_PIN); STATUS0_PORT &= ~(1<<STATUS0_PIN); break;
			default: STATUS1_PORT &= ~(1<<STATUS1_PIN); STATUS0_PORT &= ~(1<<STATUS0_PIN); break;
		}
	}
	

/* interrupt routine for timer 2 compare match
 * This is executed every 1ms and provides the timing steps for other functions
 *
 * This could use some rework, it is a bit complicated and difficult to read (10/Aug/15)
 */
	ISR (TIMER2_COMP_vect)
{
	/* generate 10ms tick and do other things that need a 10ms tick to work */
		if(ms_tick>=10)
		{
			ms_10_tick++;
			LED_tick++;
			ms_tick = 0;
			
			ISR_flag.check_supply = 1;	//tell main loop to measure supply voltage
		}
		if(ms_10_tick>=10)
		{
			ms_100_tick++;
			delayed_lock_tick++;
			ms_10_tick = 0;
		}
		
	
	/* increment ms_tick and do other things that need a 1ms tick to work */
		ISR_flag.check_inputs = 1;	//poll the inputs once back in the main loop
		ISR_flag.allow_step = 1;	//allows a single step for the stepper driver
		ms_tick++;
}


int main(void)
{
		
	setup();
	uint16_t increment = 0;	
	
	//DDRD &= ~(1<<PD4);	no longer needed
	//PORTD |= (1<<PD4);
		
    while(1)
    {
		
		/* measure supply voltage and warn user if voltage drops too low */
			if(ISR_flag.check_supply)	//measure supply voltage every 10ms
			{	
				check_supply_voltage();
				ISR_flag.check_supply = 0;	//clear flag	
			}

		/* poll the inputs and look for any falling edges indicating button presses or other input signals */
			if(ISR_flag.check_inputs)
			{
				poll_inputs();
				ISR_flag.check_inputs = 0;	//clear flag	
				
			/* Detect a long button press by incrementing a variable every ms the button is pressed
			 * This is necessary to avoid false triggering due to EMI from the motor lines.
			 */	
				if(!(DOOR_BUTTON_PINREG & (1<<DOOR_BUTTON_PIN)))
				{
					increment++;
				}else{
					increment = 0;	// reset the count if the button is released.
				}
			}
			
		/* check for a 1000ms long button press 
		 * Again, this is necessary to avoid false triggering due to motor interference 
		 */
			if(increment >= 1000)
			{
				increment = 0;				
				command.delayed_lock = 1;	//activate the delayed lock sequence
				delayed_lock_tick = 0;		//reset the timer for the delayed lock sequence
			}
		
		
		/* check, if status is unknown or a reference command has been issued
		 * If that is the case, ignore any other commands and find the reference point first!
		 *
		 * Note, that command.reference is not implemented yet. There was no need for this until now (10/Aug/15)
		 */
			if( ((!status.locked) && (!status.unlocked)) || (command.reference) )
			{
				if((command.lock)||(command.unlock)||(command.reference))
				{	
					if(input_status_current[6])	//find out, if gear magnet is already near the reed contact
					{
						find_reference(0,500,2000);	
					}else{
						find_reference(1,500,2000);
					}	
					command.lock = 0;
					command.unlock = 0;
					command.reference = 0;	
					command.delayed_lock = 0;			
				}
			}else
			
			{ 	
			/* open the door */
				if (command.unlock && !command.block_unlock && !command.reference)
				{
					command.block_lock = 1;	//prevent any other lock commands while this is active
					enable_stepper(1);

					if(!status.open)		//Door is not open yet, move to "open" position
					{
						if(move_to_position(OPEN)==1)	//set status.open once at "open" position 
						{
							status.open = 1;
						}
						ms_100_tick = 0;	//reset the 100ms tick until at "open" position 
						
					}else if(ms_100_tick<=35)
					{
						//wait 3.5s at "open" position before turning back to "neutral". This lets the user open the door by pushing it
					}

					if((ms_100_tick>35)&&(status.open))	//Door is "open" and we waited for 3.5s
					{
						
						if(move_to_position(NEUTRAL)==1)	//move back to "neutral" position and reset all status and command bits.
						{
							command.unlock = 0;
							command.block_lock = 0;
							status.open = 0;
							status.locked = 0;
							status.unlocked = 1;
							status.error = 0;
							status.reached_endstop = 0;
							command.delayed_lock = 0;

							enable_stepper(0);
						}
					}
				} else

			/* lock the door */	
				if (command.lock && !command.block_lock && !command.reference)
				{
					/* locking is the same as a reference turn, so we use every lock command to re-reference our position
					 * to account for stupid users turning the gear by hand
					 */
						enable_stepper(1);	
						command.block_unlock = 1;		//block any other actions while locking the door
						
						if(!status.reached_endstop && (position > (-2000)))		//turn CCW until either endstop is reached or 2000 steps have passed
						{
							move_to_position(-2000);
							ms_100_tick = 0;	
							
						} 
						else if (position <= (-2000))		//2000 steps have passed without reaching the endstop -> something went wrong
						{
							command.lock = 0;
							command.block_unlock = 0;
							status.locked = 0;
							status.unlocked = 0;
							status.error = 1;
							command.delayed_lock = 0;
							
							enable_stepper(0);
						}
						else if(ms_100_tick<=1){		//endstop reached, reset the position to 0 while waiting a bit 
							position = 0;
						} 
						
						if((ms_100_tick>3)&&(status.reached_endstop))	//return to "neutral" position after 300ms
						{
							if(move_to_position(NEUTRAL)==1)	//reset status and command bits after coming back to the "neutral" position
							{
								command.lock = 0;
								command.block_unlock = 0;
								status.reached_endstop = 0;
								status.locked = 1;
								status.unlocked = 0;
								status.error = 0;
								enable_stepper(0);
								command.delayed_lock = 0;
							}
	
						}							
				} 
				else
				
			/* someone locked the door by hand and triggered the endstop. Set status to "locked" */
				if (!command.lock && !command.unlock && !command.reference)
				{
					if(status.reached_endstop)
					{
						status.open = 0;
						status.unlocked = 0;
						status.locked = 1;
					}
				}
					
			}	
			
			
			
			
		/* delayed lock has been triggered. Start the countdown */
			if (command.delayed_lock && status.unlocked && !status.open && !command.block_unlock && !command.block_lock && !command.lock && !command.unlock)
			{
				if(delayed_lock_tick < 150)		//wait for 15s and set "status.delayed_lock". This triggers the flashing yellow LED 
				{
					status.delayed_lock = 1;
					switch_LED(0,'B');			
				}else
				{
					status.delayed_lock = 0;
					command.delayed_lock = 0;
					command.lock = 1;			//15s have passed, give the "lock" command to lock the door
				}
			}
			
		/* Switch LEDs according to the status bits.
		 *
		 * This needs to be streamlined, it is way to complicated
		 */	
			if(status.delayed_lock)		//Override any other LED configurations while "status.delayed_lock" is active
			{
				flash_LED(25,(180-delayed_lock_tick),'Y');
				switch_LED(0,'B');
			}else
			{

				if(status.error || ( (!status.unlocked)&&(!status.locked)))
				{
					flash_LED(10,20,'Y');
					flash_LED(10,20,'B');
					set_status('E');
				}
				if(status.open)
				{
					flash_LED(25,40,'B');
					switch_LED(0,'Y');
					set_status('U');
				}
				if(status.unlocked && !status.open)
				{
					switch_LED(0,'Y');
					switch_LED(1,'B');
					set_status('U');
				}
				if(status.locked && !status.open)
				{
					switch_LED(0,'B');
					switch_LED(1,'Y');
					set_status('L');
				}
			
			}	
    }
}