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
#include <stdio.h> // For printf functions
#include <stdlib.h> // For system functions
#include <signal.h> // For catching interrupt and aborts

#include "BinClock.h"
#include "CurrentTime.h"

#include <math.h> // For pow function
//Global variables
int hours, mins, secs;
long lastInterruptTime = -201; //Used for button debounce
int RTC; //Holds the RTC instance
char* result;
int HH,MM,SS;

void ctrlc(int signal){
	printf("Caught interrupt, exiting gracefully\n");
	cleanup();
	exit(0);
}

void catch_abort(int signal){
	printf("Caught abort, exiting gracefully\n");
	cleanup();
	exit(1);
}

void initGPIO(void){
	/* 
	 * Sets GPIO using wiringPi pins. see pinout.xyz for specific wiringPi pins
	 * You can also use "gpio readall" in the command line to get the pins
	 * Note: wiringPi does not use GPIO or board pin numbers (unless specifically set to that mode)
	 */
	printf("Setting up\n");
	wiringPiSetup(); //This is the default mode. If you want to change pinouts, be aware
	
	RTC = wiringPiI2CSetup(RTCAddr); //Set up the RTC
	
	//Set up the LEDS
	for(int i; i < sizeof(LEDS)/sizeof(LEDS[0]); i++){
	    pinMode(LEDS[i], OUTPUT);
	}
	
	//Set Up the Seconds LED for PWM
	//Write your logic here
	//secPWM
	
	printf("LEDS done\n");
	
	//Set up the Buttons
	for(int j; j < sizeof(BTNS)/sizeof(BTNS[0]); j++){
		pinMode(BTNS[j], INPUT);
		pullUpDnControl(BTNS[j], PUD_UP);
	}
	
	//Attach interrupts to Buttons
	//Write your logic here
	
	printf("BTNS done\n");
	printf("Setup done\n");
}
void light(){
lightHours(hours);
lightMins(mins);
}

/*
 * The main function
 * This function is called, and calls all relevant functions we've written
 */
int main(void){
	initGPIO();
	signal(SIGINT, ctrlc);         		//catch interupts
	signal(SIGABRT,catch_abort);
	//Set random time (3:04PM)
	//You can comment this file out later
	//wiringPiI2CWriteReg8(RTC, HOUR, 0x10+TIMEZONE);
	//wiringPiI2CWriteReg8(RTC, MIN, 0x30);
	//wiringPiI2CWriteReg8(RTC, SEC, 0b10000000);
	
	toggleTime();
	// Repeat this until we shut down
	for (;;){

		//wiringPiI2CReadReg8(RTC,hours);
		//toggleTime();
		secs = wiringPiI2CReadReg8(RTC,SEC)-0b10000000;
		mins = wiringPiI2CReadReg8(RTC,MIN);
		hours = wiringPiI2CReadReg8(RTC,HOUR);
		//secs = SS;
		//Fetch the time from the RTC
		
		//Write your logic here
		
		//hours = RTC;
		
		//Function calls to toggle LEDs
		//Write your logic here
		
		// Print out the time we have stored on our RTC
		printf("The current time is: %x:%x:%x\n", hours, mins, secs);
		light();
		//using a delay to make our program "less CPU hungry"
		delay(1000); //milliseconds
	}
	return 0;
}

/*
 * Change the hour format to 12 hours
 */
int hFormat(int hours){
	/*formats to 12h*/
	if (hours >= 24){
		hours = 0;
	}
	else if (hours > 12){
		hours -= 12;
	}
	return (int)hours;
}

/*
 * Turns on corresponding LED's for hours
 */
void lightHours(int units){
	// Write your logic to light up the hour LEDs here	
	units = hexCompensation(units);
	//printf("Start hours: %d\n",units);
	int factor = 0;
	for(int led=0;led<4;led++){
		factor = pow(2,led+1);
		if(units%factor){digitalWrite(LEDS[led],1);units-=units%factor;}
		else{digitalWrite(LEDS[led],0);}
	}
	//if(local_units%4){digitalWrite(LEDS[2],1);local_units-=local_units%4;}         
	//else{digitalWrite(LEDS[2],0);}
	
	//if(local_units%8){digitalWrite(LEDS[1],1);local_units-=local_units%8;}         
	//else{digitalWrite(LEDS[1],0);}

	//if(local_units%16){digitalWrite(LEDS[0],1);local_units-=local_units%16;}         
	//else{digitalWrite(LEDS[0],0);}

	//printf("Leftover hours: %d\n",local_units);
}

/*
 * Turn on the Minute LEDs
 */
void lightMins(int units){
	//Write your logic to light up the minute LEDs here
	 units = hexCompensation(units);
	 int factor = 0;
	 for (int led = 4; led < 10; led++)
	 {
		 factor = pow(2, led + 1);
		 if (units % factor)
		 {
			 digitalWrite(LEDS[led], 1);
			 units -= units % factor;
		 }
		 else
		 {
			 digitalWrite(LEDS[led], 0);
		 }
	 }
	 //  if(units%2){digitalWrite(LEDS[4],1);units--;}
	 //  else{digitalWrite(LEDS[4],0);}

	 //  if(units%4){digitalWrite(LEDS[5],1);units-=units%4;}
	 //  else{digitalWrite(LEDS[5],0);}

	 //  if(units%8){digitalWrite(LEDS[6],1);units-=units%8;}
	 //  else{digitalWrite(LEDS[6],0);}

	 //  if(units%16){digitalWrite(LEDS[7],1);unitçs-=units%16;}
	 //  else{digitalWrite(LEDS[7],0);}

	 //  if(units%32){digitalWrite(LEDS[8],1);units-=units%32;}
	 //  else{digitalWrite(LEDS[8],0);}

	 //  if(units%64){digitalWrite(LEDS[9],1);units-=units%64;}
	 //  else{digitalWrite(LEDS[9],0);}

	 //printf("Leftover minutes: %d\n",units);
}

/*
 * PWM on the Seconds LED
 * The LED should have 60 brightness levels
 * The LED should be "off" at 0 seconds, and fully bright at 59 seconds
 */
void secPWM(int units){
	// Write your logic here
	
}

/*
 * hexCompensation
 * This function may not be necessary if you use bit-shifting rather than decimal checking for writing out time values
 */
int hexCompensation(int units){
	/*Convert HEX or BCD value to DEC where 0x45 == 0d45 
	  This was created as the lighXXX functions which determine what GPIO pin to set HIGH/LOW
	  perform operations which work in base10 and not base16 (incorrect logic) 
	*/
	int unitsU = units%0x10;

	if (units >= 0x50){
		units = 50 + unitsU;
	}
	else if (units >= 0x40){
		units = 40 + unitsU;
	}
	else if (units >= 0x30){
		units = 30 + unitsU;
	}
	else if (units >= 0x20){
		units = 20 + unitsU;
	}
	else if (units >= 0x10){
		units = 10 + unitsU;
	}
	return units;
}


/*
 * decCompensation
 * This function "undoes" hexCompensation in order to write the correct base 16 value through I2C
 */
int decCompensation(int units){
	int unitsU = units%10;

	if (units >= 50){
		units = 0x50 + unitsU;
	}
	else if (units >= 40){
		units = 0x40 + unitsU;
	}
	else if (units >= 30){
		units = 0x30 + unitsU;
	}
	else if (units >= 20){
		units = 0x20 + unitsU;
	}
	else if (units >= 10){
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
void hourInc(void){
	//Debounce
	long interruptTime = millis();

	if (interruptTime - lastInterruptTime>200){
		printf("Interrupt 1 triggered, %x\n", hours);
		//Fetch RTC Time
		//Increase hours by 1, ensuring not to overflow
		//Write hours back to the RTC
	}
	lastInterruptTime = interruptTime;
}

/* 
 * minInc
 * Fetch the minute value off the RTC, increase it by 1, and write back
 * Be sure to cater for there only being 60 minutes in an hour
 * Software Debouncing should be used
 */
void minInc(void){
	long interruptTime = millis();

	if (interruptTime - lastInterruptTime>200){
		printf("Interrupt 2 triggered, %x\n", mins);
		//Fetch RTC Time
		//Increase minutes by 1, ensuring not to overflow
		//Write minutes back to the RTC
	}
	lastInterruptTime = interruptTime;
}

//This interrupt will fetch current time from another script and write it to the clock registers
//This functions will toggle a flag that is checked in main
void toggleTime(void){
	long interruptTime = millis();

	if (interruptTime - lastInterruptTime>200){
		HH = getHours();
		printf("hours: %d\n",HH);
		MM = getMins();
		printf("minutes: %d\n",MM);
		SS = getSecs();
		printf("seconds: %d\n",SS);
		HH = hFormat(HH);
		HH = decCompensation(HH);
		wiringPiI2CWriteReg8(RTC, HOUR, HH);

		MM = decCompensation(MM);
		wiringPiI2CWriteReg8(RTC, MIN, MM);

		SS = decCompensation(SS);
		wiringPiI2CWriteReg8(RTC, SEC, 0b10000000+SS);

	}
	lastInterruptTime = interruptTime;
}

void cleanup()
{
	printf("Cleaning up LEDs\n");
	for (int i = 0; i < sizeof(LEDS) / sizeof(LEDS[0]); i++)
	{
		digitalWrite(LEDS[i], 0);
		//printf("Turned off %d\n",i);
		//pinMode(LEDS[i], INPUT);
	}
	//TODO add PWM cleanup
	printf("Cleaning up buttons\n");
}

void ctrlc(int signal)
{
	printf("Caught interrupt, exiting gracefully\n");
	cleanup();
	exit(0);
}

void catch_abort(int signal)
{
	printf("Caught abort, exiting gracefully\n");
	cleanup();
	exit(1);
}