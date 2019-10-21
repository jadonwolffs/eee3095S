#ifndef LOGGER_H
#define LOGGER_H

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h>
#include <stdlib.h>
#include <wiringPiSPI.h>
#include <mcp3004.h>
#include <signal.h>
#include <math.h>
#include <wiringPiI2C.h>
#include <stdint.h>
#include "RTC.h"
#include <pthread.h>

//MAIN FUNCTIONS
void *read_adc(void *threadargs);
void *alarm_led(void *threadargs);

//HELPER METHODS
int hFormat(int hours);
int hexCompensation(int units);
int decCompensation(int units);
void shut_down(int signal);
void toggleTime(void);
void cleanup(void);
// int run_env(void);

//BUTTONS
void dismiss_alarm(void);
void trigger_alarm(void);

void reset(void);
void clear_console(void);
void reset_sys_time(void);

void stop_start(void);

void cycle_freq(void);

//VARIABLES
char * alarm_value;
volatile bool alarm_triggered = false;
float data[8] = {0,0,0,0,0,0,0,0};
const char RTCAddr = 0x6f;
const char SEC = 0x00;
const char MIN = 0x01;
const char HOUR = 0x02;
const char TIMEZONE = 2;
extern int HH,MM,SS;
short freq = 1000;
bool monitoring = true;
long last_interrupt = -201;
long last_alarm = -180000;
float dac_out_voltage = 0.0;
extern int temp = 0;

#define DEBUG false
#define BASE 100
#define SPI_CHAN 0
#define SPI_CHAN_DAC 1
#define SPI_SPEED_DAC freq

#endif
