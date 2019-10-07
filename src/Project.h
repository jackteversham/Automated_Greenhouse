#ifndef PRAC4_H
#define PRAC4_H

//Includes
#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <wiringPiSPI.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>

//Function definitions
void initGPIO(void);
void reset(void);
void changeFrequency(void);
void dismissAlarm(void);
void monitoring(void);
// static int myAnalogRead(struct wiringPiNodeStruct *node, int pin);
// int mcp3004Setup(const int pinBase, int spiChannel);
int hFormat(int hours);
void secPWM(int units);
int hexCompensation(int units);
int decCompensation(int units);
void toggleTime(void);
void cleanup(int);
void setCurrentTime(void)

//Define constants
const char RTCAddr = 0x6f;
const char SEC = 0x00;
const char MIN = 0x01;
const char HOUR = 0x02;
const char TIMEZONE = 2; // +02H00 (RSA)

//Define buttons
const int RESET = 25; //wiringPi 25 AND BCM 26
const int FREQUENCY = 26; //wiringPi 26 AND BCM 12
const int ALARM_DISMISS = 22; //wiringPi 22 AND BCM 6
const int START = 21; //wiringPi 21 AND BCM 5
const int BTNS[] = {25, 26, 22, 21};

//Define LEDS
const int RESET_LED = 2; //wiringPi 2 AND BCM 27
const int FREQUENCY_LED = 1;//wiringPi 1 AND BCM 18
const int ALARM_DISMISS_LED = 0; //wiringPi 0 AND BCM 17
const int START_LED = 7; //wiringPi 7 AND BCM 4
const int LEDS[] = {2, 1, 0, 7};

const int ALARM_LED = 23; //wiringPi 23 AND BCM 13

#endif
