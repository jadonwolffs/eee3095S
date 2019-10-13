//EEE3095/96S Project A
//WLFJAD001 ARNJAM004
#include "project_a.h"
#pragma GCC push_options
#pragma GCC optimize ("O0")
int hours, mins, secs;
int RTC; //Holds the RTC instance
int reset_time=0;
int main(void)
{
	
	wiringPiSetup();
	wiringPiSPISetup(SPI_CHAN_DAC, SPI_SPEED_DAC);

	//SETUP GPIO PINS
	pinMode (26, PWM_OUTPUT);
	pinMode (1, PWM_OUTPUT);

	pinMode (22, INPUT);
	pullUpDnControl(22,PUD_UP);
    wiringPiISR(22, INT_EDGE_RISING, reset);

	pinMode (23, INPUT);
	pullUpDnControl(23,PUD_UP);
    wiringPiISR(23, INT_EDGE_RISING, cycle_freq);

	pinMode (24, INPUT);
	pullUpDnControl(24,PUD_UP);
    wiringPiISR(24, INT_EDGE_RISING, stop_start);

	pinMode (25, INPUT);
	pullUpDnControl(25,PUD_UP);
    wiringPiISR(25, INT_EDGE_RISING, dismiss_alarm);
	signal(SIGINT, shut_down);

	RTC = wiringPiI2CSetup(RTCAddr);
	toggleTime();
	mcp3004Setup(BASE, SPI_CHAN);

	pthread_attr_t tattr;
    pthread_t thread_id;
    int newprio = 99;
    sched_param param;
    pthread_attr_init (&tattr);
    pthread_attr_getschedparam (&tattr, &param);
    param.sched_priority = newprio;
    pthread_attr_setschedparam (&tattr, &param);
    pthread_create(&thread_id, &tattr, read_adc, (void *)1);
	pthread_create(&thread_id, &tattr, alarm_led, (void *)1);
	printf("_________________________________________________________________________________________________\n");
	printf("| RTC Time \t| Sys Timer \t| Humidity \t| Temp \t| Light | DAC out \t| Alarm \t|\n");
	delay(500);
	for (;;)
	{
		int temp = (round(((channels[1] * 3.3 / 1023) - 0.7) / 0.01));
		
		temp_ptr = &temp;
		float light = (float)channels[0];
		secs = hexCompensation(wiringPiI2CReadReg8(RTC, SEC) - 0b10000000);
		mins = hexCompensation(wiringPiI2CReadReg8(RTC, MIN));
		hours = hexCompensation(wiringPiI2CReadReg8(RTC, HOUR));
		
		float hum = channels[3] * 3.3 / 1023;
		dac_out_voltage = (light / 1023) * hum;
		int dac_out = round((dac_out_voltage/3.3)*1023);
		unsigned char * dac_char_array;
		dac_char_array = (unsigned char *)(0b0111<<12 | ((int)dac_out)<<2 | 0b00);//|0b00 isn't strictly necessary
		if (DEBUG)
		{
			printf("temp ");
			printf("%d\n",(temp));
			printf("%f %f %d\n",light,hum,dac_out);
			printf("%d\n",dac_char_array);
		}
		wiringPiSPIDataRW(SPI_CHAN_DAC, dac_char_array, 1);
		// RTC Time 	Sys Timer 	Humidity 	Temp 	Light 	DAC out Alarm
		// 10:17:15 	00:00:00 	0.5 V 		25 C 	595 	0.29V 	*
		if (monitoring)
		{
			printf("| %d:%d:%d \t| %d \t\t| %f \t| %dC \t| %d \t| %f(%d) | %s \t\t|\n",hours, mins, secs,(millis()-reset_time)/1000,hum,temp,(int)light,dac_out_voltage,dac_out,alarm_value);
		}
		delay(freq);
	}
	return 0;
}
void trigger_alarm(void){
	if (millis()-last_alarm > 0)
	{
		if (dac_out_voltage>=2.65 || (dac_out_voltage<=0.5&&dac_out_voltage>0.0))
		{
			if (DEBUG){
				printf("alarm %f\n",dac_out_voltage);
			}
			alarm_triggered = true;
		}
	}
	
	
}
void *alarm_led(void *threadargs){
	while (true)
	{
		if (alarm_triggered){
			alarm_value = "*";
			for (size_t i = 0; i < 1024; i++)
			{
				pwmWrite(26,i);
				delay(1);
			}
			delay(10);
			for (size_t i = 1023; i > 0; i--)
			{
				pwmWrite(26,i);
				delay(1);
			}
			delay(5);
		}
		else{
			pwmWrite(26,0);
			alarm_value = " ";
		}
		delay(100);
	}
}
void *read_adc(void *threadargs)
{
	while (true)
	{
		if (monitoring)
		{
			for (int chan = 0; chan < 8; ++chan)
			{
				channels[chan] = analogRead(BASE + chan);
			}
			trigger_alarm();
		}
		delay(freq);
	}
}
void reset(void){
	long current_time = millis();
	if(current_time-last_interrupt>150){
		reset_sys_time();
		clear_console();
		printf("_________________________________________________________________________________________________\n");
		printf("| RTC Time \t| Sys Timer \t| Humidity \t| Temp \t| Light | DAC out \t| Alarm \t|\n");
	}
	last_interrupt=current_time;
}
void dismiss_alarm(void)
{
	long current_time = millis();
	if(current_time-last_interrupt>150){
		
		if (DEBUG){
			printf("dismiss called\n");
			if (alarm_triggered){
				alarm_triggered = false;
			}
			else{
				alarm_triggered = true;
			}	
		}
		else{
			alarm_triggered = false;
		}	
	}
	last_interrupt=current_time;
}
void clear_console(void)
{
	system("clear");
}
void reset_sys_time(void)
{
	reset_time=millis();
}

void cycle_freq(void){
	long current_time = millis();
	if(current_time-last_interrupt>150){
		switch (freq)
		{
		case 1000:
			freq=2000;
			break;
		case 2000:
			freq=5000;
			break;
		case 5000:
			freq=1000;
			break;
		default:
		printf("error cycle");
			freq=1000;
			break;
		}
	}
	last_interrupt=current_time;
}

void stop_start(void)
{
	long current_time = millis();
	if(current_time-last_interrupt>150){
		switch (monitoring)
		{
		case true:
			monitoring = false;
			break;
		case false:
			monitoring = true;
			break;
		default:
		printf("error start stop");
			break;
		}
	}
	last_interrupt=current_time;
	
}

void shut_down(int x)
{
	cleanup();
	
	printf("Shutting down\n");
	exit(0);
}
void cleanup(void){
	printf("Cleaning up.");
	delay(100);
	pwmWrite(26,0);
	pinMode (26, INPUT);
	printf(".");
	pwmWrite(1,0);
	pinMode (1, INPUT);
	delay(100);
	printf(".");
	printf("\n");
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
//This interrupt will fetch current time from another script and write it to the clock registers
//This functions will toggle a flag that is checked in main
void toggleTime(void)
{
	long interruptTime = millis();

	//if (interruptTime - lastInterruptTime>200){
	HH = getHours();
	printf("Hour: %d\n",HH);
	MM = getMins();
	printf("Minutes: %d\n",MM);
	SS = getSecs();
	printf("Seconds: %d\n",SS);

	HH = hFormat(HH);
	HH = decCompensation(HH);
	wiringPiI2CWriteReg8(RTC, HOUR, HH);

	MM = decCompensation(MM);
	wiringPiI2CWriteReg8(RTC, MIN, MM);

	SS = decCompensation(SS);
	wiringPiI2CWriteReg8(RTC, SEC, 0b10000000 + SS);

	//	}
	//	lastInterruptTime = interruptTime;
}
#pragma GCC pop_options