#ifndef MINIPROJECTA_H
#define MINIPROJECTA_H

#define BASE 100
#define SPI_CHAN 0
void exiting(int signal);
int channels[8] = {0,0,0,0,0,0,0,0};
void exiting(int signal);
int hFormat(int hours);
int hexCompensation(int units);
int decCompensation(int units);
void toggleTime(void);
#define SPI_CHAN_DAC 1
#define SPI_SPEED_DAC 25600
const char RTCAddr = 0x6f;
const char SEC = 0x00; // see register table in datasheet
const char MIN = 0x01;
const char HOUR = 0x02;
const char TIMEZONE = 2; // +02H00 (RSA)
unsigned char DAC_VAL;
float DAC_VOLTAGE;
extern int HH,MM,SS;
#endif
