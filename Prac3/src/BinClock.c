/*
 * BinClock.c
 * Jarrod Olivier
 * Modified for EEE3095S/3096S by Keegan Crankshaw
 * August 2019
 * 
 * WLFJAD001 ARNJAM004
 * Date 13/08/2019
*/

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h>  // For printf functions
#include <stdlib.h> // For system functions
#include <signal.h> // For catching interrupt and aborts

#include "BinClock.h"
#include "CurrentTime.h"

#include <math.h> // For pow function
//Global variables
int hours, mins, secs;
long lastInterruptTime = -201; //Used for button debounce
int RTC;					   //Holds the RTC instance
char *result;
int HH, MM, SS;

void initGPIO(void)
{
	/* 
	 * Sets GPIO using wiringPi pins. see pinout.xyz for specific wiringPi pins
	 * You can also use "gpio readall" in the command line to get the pins
	 * Note: wiringPi does not use GPIO or board pin numbers (unless specifically set to that mode)
	 */
	printf("Setting up\n");
	wiringPiSetup(); //This is the default mode. If you want to change pinouts, be aware

	RTC = wiringPiI2CSetup(RTCAddr); //Set up the RTC

	//Set up the LEDS
	for (int i; i < 10; i++)
	{
		pinMode(LEDS[i], OUTPUT);
	}

	//Set Up the Seconds LED for PWM
	pinMode(SECS, PWM_OUTPUT);

	printf("LEDS done\n");

	//Set up the Buttons
	for (int j; j < sizeof(BTNS) / sizeof(BTNS[0]); j++)
	{
		pinMode(BTNS[j], INPUT);
		pullUpDnControl(BTNS[j], PUD_UP);
	}

	//Attach interrupts to Buttons
	wiringPiISR(BTNS[0], INT_EDGE_RISING, &minInc); //Add interrupt to minute incrementing button
	wiringPiISR(BTNS[1], INT_EDGE_RISING, &hourInc); //Add interrupt to hour incrementing button

	printf("BTNS done\n");
	printf("Setup done\n");
}
/*
 *	A function to call the various LED lighting functions each cycle
 */
void light()
{
	lightHours(hours);
	lightMins(mins);
	secPWM(secs);
}

/*
 * The main function
 * This function is called, and calls all relevant functions we've written
 */
int main(void)
{
	initGPIO();
	signal(SIGINT, ctrlc); //catch keyboard interupts
	signal(SIGABRT, catch_abort); //catch crashes to cleanup
	//Set random time (3:04PM)
	//You can comment this file out later - kept in for debugging
	//wiringPiI2CWriteReg8(RTC, HOUR, 0x10+TIMEZONE);
	//wiringPiI2CWriteReg8(RTC, MIN, 0x30);
	//wiringPiI2CWriteReg8(RTC, SEC, 0b10000000); // Needs to enable st bit

	toggleTime(); // Updates the RTC stored time with the system time
	// Repeat this until we shut down
	for (;;)
	{
		//Fetch the time from the RTC
		secs = wiringPiI2CReadReg8(RTC, SEC) - 0b10000000;
		mins = wiringPiI2CReadReg8(RTC, MIN);
		hours = wiringPiI2CReadReg8(RTC, HOUR);

		//Function calls to toggle LEDs
		light(); 

		// Print out the time we have stored on our RTC
		printf("The current time is: %x:%x:%x\n", hours, mins, secs);
		
		//using a delay to make our program "less CPU hungry"
		delay(1000); // 1000 milliseconds = 1 second
	}
	return 0;
}

/*
 * Change the hour format to 12 hours
 */
int hFormat(int hours)
{
	/*formats to 12h*/
	if (hours >= 24)
	{
		hours = 0;
	}
	else if (hours > 12)
	{
		hours -= 12;
	}
	return (int)hours;
}

/*
 * Turns on corresponding LED's for hours
 */
void lightHours(int units)
{
	// Compensate for the way that the unit is stored in the RTC
	units = hexCompensation(units);
	int factor = 0;
	// Loop through each of the 4 hour LEDs to check if it needs to be lit
	for (int led = 0; led < 4; led++)
	{
		// Find the base(2) value of the current LED position
		factor = pow(2, led + 1);

		// Check if units is greater than the calculated factor
		if (units % factor)
		{
			// Light the LED and remove the modded value from units to correctly calculate the next LED
			digitalWrite(LEDS[led], 1);
			units -= units % factor;
		}
		else
		{
			// Make sure the LED is unlit elsewise
			digitalWrite(LEDS[led], 0);
		}
	}
}

/*
 * Turn on the Minute LEDs
 */
void lightMins(int units)
{
	// Compensate for the way that the unit is stored in the RTC
	units = hexCompensation(units);
	int factor = 0;
	// Start from a LED base of 4 to compensate for the hour LEDs
	int base = 4;
	// Loop through each of the 6 minute LEDs to check if it needs to be lit
	for (int led = 0; led < 6; led++)
	{
		// Find the base(2) value of the current LED position
		factor = pow(2, led + 1);

		// Check if units is greater than the calculated factor
		if (units % factor)
		{
			// Light the LED and remove the modded value from units to correctly calculate the next LED
			digitalWrite(LEDS[led + base], 1);
			units -= units % factor;
		}
		else
		{
			// Make sure the LED is unlit elsewise
			digitalWrite(LEDS[led + base], 0);
		}
	}
}

/*
 * PWM on the Seconds LED
 * The LED should have 60 brightness levels
 * The LED should be "off" at 0 seconds, and fully bright at 59 seconds
 */
void secPWM(int units)
{
	// Write the value of the seconds to the PWM LED (multiplied by 17.0666 to achieve near 1024 steps)
	pwmWrite(SECS, round(hexCompensation(units) * 17.0666);
}

/*
 * hexCompensation
 * This function may not be necessary if you use bit-shifting rather than decimal checking for writing out time values
 */
int hexCompensation(int units)
{
	/*Convert HEX or BCD value to DEC where 0x45 == 0d45 
	  This was created as the lighXXX functions which determine what GPIO pin to set HIGH/LOW
	  perform operations which work in base10 and not base16 (incorrect logic) 
	*/
	int unitsU = units % 0x10;

	if (units >= 0x50)
	{
		units = 50 + unitsU;
	}
	else if (units >= 0x40)
	{
		units = 40 + unitsU;
	}
	else if (units >= 0x30)
	{
		units = 30 + unitsU;
	}
	else if (units >= 0x20)
	{
		units = 20 + unitsU;
	}
	else if (units >= 0x10)
	{
		units = 10 + unitsU;
	}
	return units;
}

/*
 * decCompensation
 * This function "undoes" hexCompensation in order to write the correct base 16 value through I2C
 */
int decCompensation(int units)
{
	int unitsU = units % 10;

	if (units >= 50)
	{
		units = 0x50 + unitsU;
	}
	else if (units >= 40)
	{
		units = 0x40 + unitsU;
	}
	else if (units >= 30)
	{
		units = 0x30 + unitsU;
	}
	else if (units >= 20)
	{
		units = 0x20 + unitsU;
	}
	else if (units >= 10)
	{
		units = 0x10 + unitsU;
	}
	return units;
}

/*
 * hourInc
 * Fetch the hour value off the RTC, increase it by 1, and write back
 * Be sure to cater for there only being 23 hours in a day
 * Software Debouncing should be used
 */
void hourInc(void)
{
	long interruptTime = millis();

	// This `if` along with the capturing of lastInterruptTime at the end of the method is what prevents bounced triggers (does the actual software debouncing)
	if (interruptTime - lastInterruptTime > 200)
	{
		//Fetch RTC Time
		hours = hexCompensation(wiringPiI2CReadReg8(RTC, HOUR));
		//Increase hours by 1, ensuring not to overflow
		hours++;
		hours=hFormat(hours);

		//Write hours back to the RTC
		wiringPiI2CWriteReg8(RTC, HOUR, decCompensation(hours));
		printf("Interrupt 1 triggered, %x\n", decCompensation(hours));
	}
	lastInterruptTime = interruptTime;
}

/* 
 * minInc
 * Fetch the minute value off the RTC, increase it by 1, and write back
 * Be sure to cater for there only being 60 minutes in an hour
 * Software Debouncing should be used
 */
void minInc(void)
{
	long interruptTime = millis();

	// This `if` along with the capturing of lastInterruptTime at the end of the method is what prevents bounced triggers (does the actual software debouncing)
	if (interruptTime - lastInterruptTime > 200)
	{
		//Fetch RTC Time
		mins = hexCompensation(wiringPiI2CReadReg8(RTC, MIN));
		//Increase minutes by 1, ensuring not to overflow
		if(mins>=59){// Prevents overflow
			hours++;
			mins=0;
		}
		else{
			mins++;
		}	
		//Write minutes back to the RTC - has to write both hour and minute values in case of overflow
		wiringPiI2CWriteReg8(RTC, HOUR, hours);
		wiringPiI2CWriteReg8(RTC, MIN, decCompensation(mins));

		printf("Interrupt 2 triggered, %x\n", decCompensation(mins));
	}
	lastInterruptTime = interruptTime;
}

//This interrupt will fetch current time from another script and write it to the clock registers
//This functions will toggle a flag that is checked in main
void toggleTime(void)
{
	long interruptTime = millis();

	// This `if` along with the capturing of lastInterruptTime at the end of the method is what prevents bounced triggers (does the actual software debouncing)
	if (interruptTime - lastInterruptTime > 200)
	{
		HH = getHours();
		printf("hours: %d\n", HH);
		MM = getMins();
		printf("minutes: %d\n", MM);
		SS = getSecs();
		printf("seconds: %d\n", SS);
		HH = hFormat(HH);
		HH = decCompensation(HH);
		wiringPiI2CWriteReg8(RTC, HOUR, HH);

		MM = decCompensation(MM);
		wiringPiI2CWriteReg8(RTC, MIN, MM);

		SS = decCompensation(SS);
		wiringPiI2CWriteReg8(RTC, SEC, 0b10000000 + SS);
	}
	lastInterruptTime = interruptTime;
}
/*
 * 	Cleans up the LEDs (writes 0 to them and sets them to inputs)
 */
void cleanup()
{
	printf("Cleaning up LEDs\n");

	// Write 0 to the LEDs and set them to input
	for (int i = 0; i < sizeof(LEDS) / sizeof(LEDS[0]); i++)
	{
		digitalWrite(LEDS[i], 0);
		pinMode(LEDS[i], INPUT);
	}
	// PWM write 0 to the PWM LED and set it to input
	pwmWrite(SECS, 0);
	pinMode(SECS, INPUT);
	printf("Cleaning up buttons\n");
}
/*
 * 	Catch keyboard interrupts and call cleanup before exiting
 */
void ctrlc(int signal)
{
	printf("Caught interrupt, exiting gracefully\n");
	cleanup();
	exit(0);
}
/*
 * 	Catch aborts and call cleanup before exiting
 */
void catch_abort(int signal)
{
	printf("Caught abort, exiting gracefully\n");
	cleanup();
	exit(1);
}
