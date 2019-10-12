//Mini Project A
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <mcp3004.h>
#include <stdio.h> //For printf functions
#include <stdlib.h> // For system functions
#include <signal.h>
#include <math.h>
#include <wiringPiI2C.h>
#include "miniprojectA.h"
#include "CurrentTime.h"
int hours, mins, secs;
int RTC; //Holds the RTC instance
// int HH,MM,SS;
int main(void){
signal(SIGINT, exiting);
wiringPiSetup();
wiringPiSPISetup(SPI_CHAN_DAC,SPI_SPEED_DAC);
RTC = wiringPiI2CSetup(RTCAddr);
toggleTime();
mcp3004Setup(BASE, SPI_CHAN);
int c;
for (;;)
{
for(c=0;c<8;++c)
{
  channels[c]=analogRead(BASE+c);
}
printf("Humidity: %0.1fV\n",channels[3]*3.3/1023);
printf("Light Level: %d\n", channels[0]);
printf("Temperature: %0.0f\n",round(((channels[1]*3.3/1023)-0.5)/0.01));
secs = wiringPiI2CReadReg8(RTC, SEC) - 0b10000000;
mins = wiringPiI2CReadReg8(RTC, MIN);
hours = wiringPiI2CReadReg8(RTC, HOUR);
float Light = channels[0];
float Humidity = channels[3]*3.3/1023;
int DAC = (int)((Light/1023)*Humidity*1023/3.3);
unsigned char DAC_VAL[3] = {(DAC&0b1100000000)>>8,(DAC&0b11110000)>>4,DAC&0b1111};
//uint16_t DAC_16 = DAC;
//printf("DAC: %d\n",DAC_VAL<<2);
float DAC_VOLTAGE = DAC*3.3/1023;
printf("DAC Voltage: %f\n",DAC_VOLTAGE);
printf("ADC_DAC Voltage: %0.1f\n",(channels[2]*3.3)/1023);
printf("The current time is: %x:%x:%x\n", hours, mins, secs);
printf("\n");
unsigned char c = 0b1111;
wiringPiSPIDataRW(SPI_CHAN_DAC,DAC_VAL,2);
delay(1000);
}
return 0;
}

void exiting(int x)
{
	printf("EXITING\n");
	exit(0);
}

//START BinClock
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
//This interrupt will fetch current time from another script and write it to the clock registers
//This functions will toggle a flag that is checked in main
void toggleTime(void){
	long interruptTime = millis();

	//if (interruptTime - lastInterruptTime>200){
		HH = getHours();
		MM = getMins();
		SS = getSecs();

		HH = hFormat(HH);
		HH = decCompensation(HH);
		wiringPiI2CWriteReg8(RTC, HOUR, HH);

		MM = decCompensation(MM);
		wiringPiI2CWriteReg8(RTC, MIN, MM);

		SS = decCompensation(SS);
		wiringPiI2CWriteReg8(RTC, SEC, 0b10000000+SS);

//	}
//	lastInterruptTime = interruptTime;
}
