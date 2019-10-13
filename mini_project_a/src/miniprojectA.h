#ifndef MINIPROJECTA_H
#define MINIPROJECTA_H

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
#include "CurrentTime.h"
#include <pthread.h>
// #include <BlynkApiWiringPi.h>
// #include <BlynkSocket.h>
// #include <BlynkOptionsParser.h>

// static BlynkTransportSocket _blynkTransport;
// BlynkSocket Blynk(_blynkTransport);

// #include <BlynkWidgets.h>


#define BASE 100
#define SPI_CHAN 0
void exiting(int signal);
int channels[8] = {0,0,0,0,0,0,0,0};
void exiting(int signal);
int hFormat(int hours);
int hexCompensation(int units);
int decCompensation(int units);
void *read_adc(void *threadargs);
void toggleTime(void);
#define SPI_CHAN_DAC 1
#define SPI_SPEED_DAC 25600
#define BUFFER_SIZE 1000

unsigned int uptime;      		// 1 second intervals
unsigned int pinStatus;   		// status of BCM 17
unsigned int lastpinStatus = 0; // to toggle

const char RTCAddr = 0x6f;
const char SEC = 0x00; // see register table in datasheet
const char MIN = 0x01;
const char HOUR = 0x02;
const char TIMEZONE = 2; // +02H00 (RSA)
unsigned char DAC_VAL;
float DAC_VOLTAGE;
extern int HH,MM,SS;
#endif
