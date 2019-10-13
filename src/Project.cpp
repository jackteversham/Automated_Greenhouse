#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringPiSPI.h>
#include <stdio.h>   //For printf functions
#include <stdlib.h>  //For system functions
#include <signal.h>  //For cleanup fucntion
#include <softPwm.h> //For PWM on seconds LED
#include <string>

#include "Project.h"
#include "CurrentTime.c"
#include "mcp3004.h"

using namespace std;

//Global variables
int bounce = 200; 					//Minimal interrpt interval (ms)
long lastInterruptTime = 0; //Used for button debounce
int RTC; 										//Holds the RTC instance
int hours, mins, secs;

int increment = 1000; //Default of 1 second
int choice = 0;

bool alarmActive = false;
bool alarmEnabled = true;
bool monitorConditions = true;

int prevAlarmHour = 0;
int prevAlarmMin = 0;
int prevAlarmSec = 0;

void initGPIO(void){
	/*
	 * Sets GPIO using wiringPi pins. see pinout.xyz for specific wiringPi pins
	 * You can also use "gpio readall" in the command line to get the pins
	 * Note: wiringPi does not use GPIO or board pin numbers (unless specifically set to that mode)
	 */
	printf("Setting Up\n");
	wiringPiSetup(); //This is the default mode. If you want to change pinouts, be aware

	RTC = wiringPiI2CSetup(RTCAddr); //Set up the RTC
	wiringPiSPISetup(SPI_CHAN, SPI_CLOCKSPEED); //setup the SPI
	mcp3004Setup(BASE, SPI_CHAN); //setup mcp3004 library

	//Set up the LEDS
	for(unsigned int i; i < sizeof(LEDS)/sizeof(LEDS[0]); i++){
	    pinMode(LEDS[i], OUTPUT);
	}

	//Set Up the alarm LED for PWM
	pinMode(ALARM_LED,OUTPUT);
	softPwmCreate(ALARM_LED, 0, 60); //set SECONDS pin to support software PWM. Range of 0-60 (fully on)
	printf("LEDS Done\n");

	//Set up the Buttons
	for(unsigned int j; j < sizeof(BTNS)/sizeof(BTNS[0]); j++){
		pinMode(BTNS[j], INPUT);
		pullUpDnControl(BTNS[j], PUD_UP);
	}

	//Attach interrupts to Buttons
	wiringPiISR(RESET, INT_EDGE_FALLING, reset);
	wiringPiISR(FREQUENCY, INT_EDGE_FALLING, changeFrequency);
	wiringPiISR(ALARM_DISMISS, INT_EDGE_FALLING, dismissAlarm);
	wiringPiISR(START, INT_EDGE_FALLING, monitoring);

	printf("BTNS Done\n");
	printf("Setup Done\n");

	printHeading();
}

/*
 * Method associated with RESET button interrpt
 */
void reset(void){
	//Debounce
	long interruptTime = millis();

	if (interruptTime - lastInterruptTime>bounce){
		//printf("Reset Interrupt Triggered\n");
		digitalWrite(RESET_LED, HIGH);
		delay(1000);
		digitalWrite(RESET_LED, LOW);

		wiringPiI2CWriteReg8(RTC, HOUR, 0x0);
    wiringPiI2CWriteReg8(RTC, MIN, 0x0);
    wiringPiI2CWriteReg8(RTC, SEC, 0b10000000);

		system("clear");
		printHeading();

	}
	lastInterruptTime = interruptTime;
}

/*
 * Method associated with FREQUENCY button interrpt
 */
void changeFrequency(void){
	//Debounce
	long interruptTime = millis();

	if (interruptTime - lastInterruptTime>bounce){
		//printf("Frequency Interrupt Triggered\n");
		digitalWrite(FREQUENCY_LED, HIGH);
		delay(1000);
		digitalWrite(FREQUENCY_LED, LOW);

		switch(choice) {
	    case 0:
				increment = 1000;
				break;
	    case 1:
				increment = 2000;
				break;
	    case 2:
				increment = 5000;
				break;
		}
		choice = (choice + 1) % 3;

	}
	lastInterruptTime = interruptTime;
}

/*
 * Method associated with ALARM_DISMISS button interrpt
 */
void dismissAlarm(void){
	//Debounce
	long interruptTime = millis();

	if (interruptTime - lastInterruptTime>bounce){
		//printf("Dismiss Alarm Interrupt Triggered\n");
		digitalWrite(ALARM_DISMISS_LED, HIGH);
		delay(1000);
		digitalWrite(ALARM_DISMISS_LED, LOW);

		alarmActive = false;

	}
	lastInterruptTime = interruptTime;
}

/*
 * Method associated with START button interrpt
 */
void monitoring(void){
	//Debounce
	long interruptTime = millis();

	if (interruptTime - lastInterruptTime>bounce){
		//printf("Monitoring Interrupt Triggered\n");
		digitalWrite(START_LED, HIGH);
		delay(1000);
		digitalWrite(START_LED, LOW);

		monitorConditions = !monitorConditions;
	}
	lastInterruptTime = interruptTime;
}
/*
 * The main function
 * This function is called, and calls all relevant functions we've written
 */
int main(void){

  signal(SIGINT, cleanup);
	initGPIO();
	//set the RTC registers with the current system time
	setCurrentTime();
	// Initialize thread with parameters
  // Set the main thread to have a priority of 99
  pthread_attr_t tattr;
  pthread_t thread_id;
  int priorty = 99;
  sched_param param;

  pthread_attr_init (&tattr); 		 // Initialized with default attributes
  pthread_attr_getschedparam (&tattr, &param); // Safe to get existing scheduling param
  param.sched_priority = priorty; 		 // Set the priority; others are unchanged
  pthread_attr_setschedparam (&tattr, &param); // Setting the new scheduling param
  pthread_create(&thread_id, &tattr, monitorThread, (void *)1); // with new priority specified

	wiringPiI2CWriteReg8(RTC, HOUR, 0x0);
	wiringPiI2CWriteReg8(RTC, MIN, 0x0);
	wiringPiI2CWriteReg8(RTC, SEC, 0b10000000);

	// Print out the time we have stored on our RTC
	//printf("The current time is: %d:%d:%d\n", hexCompensation(HH), hexCompensation(MM), hexCompensation(SS));

	pthread_join(thread_id, NULL);
  pthread_exit(NULL);

	delay(1000); //milliseconds

	return 0;
}

void *monitorThread(void *threadargs){

    for(;;){

			while (!monitorConditions) continue;

			//Fetch the time from the RTC
			HH = wiringPiI2CReadReg8(RTC, HOUR); //read SEC register from RTC
			MM = wiringPiI2CReadReg8(RTC, MIN); //read MIN register from RTC
			SS = wiringPiI2CReadReg8(RTC, SEC); //read the SEC register from RTC

			hours = hexCompensation(HH);
			mins = hexCompensation(MM);
			secs = hexCompensation(SS & 0b01111111);

      string systemHour = to_string(hours);
      string systemMin = to_string(mins);
			string systemSec = to_string(secs);
      string systemTime = string(systemHour + ":" + systemMin + ":" + systemSec);

			string currentHour = to_string(getHours());
			string currentMin = to_string(getMins());
			string currentSec = to_string(getSecs());
			string currentTime = string(currentHour + ":" + currentMin + ":" + currentSec);

			//Reading from ADC
			int temperatureReading = analogRead(BASE+0); //temp on channel zero
			float temperatureVolts = (temperatureReading*3.3)/1024.0;
			float temperatureInCelsius = (temperatureVolts - (550.0 / 1000.0)) / (10.0 / 1000.0);
			//printf("The temperature is: %f\n", temperatureVolts);

			int humidityReading = analogRead(BASE+1); //humidity from pot on channel 1
			float humidity = (humidityReading*3.3)/1024.0;
			//printf("The humidity is: %f\n", humidity);

			int light = analogRead(BASE+2); //light from LDR on channel 2

			float dacOutput = (light / 1024.0) * humidity;

			checkAlarm(hours, mins, secs, prevAlarmHour, prevAlarmMin, prevAlarmSec);

			if((dacOutput < 0.65 || dacOutput > 2.65)){

				if(alarmEnabled){
					alarmActive = true;
					prevAlarmHour = hours;
					prevAlarmMin = mins;
					prevAlarmSec = secs;
				}

			}

			if(alarmActive){
				secPWM(dacOutput);
				printf("    %-17s%-17s%-14.2f%-12.2f%-12d%-14.2f%-14s\n", currentTime.c_str(), systemTime.c_str(), humidity, temperatureInCelsius, light, dacOutput, "*");
			}
			else{
				secPWM(0);
				printf("    %-17s%-17s%-14.2f%-12.2f%-12d%-14.2f%-14s\n", currentTime.c_str(), systemTime.c_str(), humidity, temperatureInCelsius, light, dacOutput, " ");
			}
			//printf("--------------------------------------------------------------------------------------------------\n");

			delay(increment);

		}
    pthread_exit(NULL);
}

// void updateSystemTime(SS, MM, HH, ){
//     int second1 = rtcTime[0];
//     int minute1 = rtcTime[1];
//     int hour1 = rtcTime[2];
//     int second2 = timeOfStart[0];
//     int minute2 = timeOfStart[1];
//     int hour2 = timeOfStart[2];
//
//     if(second2 > second1) {
//       minute1--;
//       second1 += 60;
//     }
//
//    systemTime[0] = second1 - second2;
//
//    if(minute2 > minute1) {
//       hour1--;
//       minute1 += 60;
//    }
//    systemTime[1] = minute1 - minute2;
//    systemTime[2] = hour1 - hour2;
// }

/*
 * Calculate amount of time between alarms being triggered
 */
void checkAlarm(int hour1, int min1, int sec1, int hour2, int min2, int sec2){
	int diff_hour, diff_min, diff_sec;

  if(sec2 > sec1) {
      min1--;
      sec1 += 60;
   }

   diff_sec = sec1 - sec2;

   if(min2 > min1) {
      hour1--;
      min1 += 60;
   }

   diff_min = min1 - min2;
   diff_hour = hour1 - hour2;

   if ((diff_hour >= 0) && (diff_sec >= 5) && (diff_min >= 0)) { //3 minutes have passed
		 alarmEnabled = true;
   }
   else {
		 alarmEnabled = false;
   }
}

/*
 * Displays headings
 */
void printHeading(void){
	printf("--------------------------------------------------------------------------------------------------\n");
	printf("|   RTC Time   |   Sys Timer   |   Humidity (V)|   Temp (C) |  Light  |   DAC Out (V)|   Alarm   |\n");
	printf("--------------------------------------------------------------------------------------------------\n");
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
 * Change the minute format
 */
int mFormat(int mins){
  if (mins >= 60){
      HH = hexCompensation(HH); // Increment by 1
      HH++;
      HH = decCompensation(HH);
      mins = 0;
  }
  return (int)mins;
}

/*
 * PWM on the ALARM LED
 */
void secPWM(int units){
	softPwmCreate(ALARM_LED, 0, 1023);
	softPwmWrite(ALARM_LED, units);
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
 * Reset GPIO pins
 */
void cleanup(int i){
	// Set all outputs to LOW
	for(unsigned int i=0 ; i<sizeof(LEDS)/sizeof(LEDS[0]); i++){
		digitalWrite(LEDS[i], LOW);
	}
	// Set up each LED to Input (High Impedance)
	for (unsigned int i=0 ; i<sizeof(LEDS)/sizeof(LEDS[0]); i++) {
		pinMode(LEDS[i], INPUT);
	}

	pinMode(ALARM_LED, INPUT);

	printf("Cleaning Up\n");
	exit(0);
}

/*
 * Gets current system time writes to RTC
 */
void setCurrentTime(void){ //
	    HH = getHours();
			MM = getMins();
			SS = getSecs();

			HH = hFormat(HH);
			HH = decCompensation(HH);
			wiringPiI2CWriteReg8(RTC, HOUR, HH);

			MM = decCompensation(MM);
			wiringPiI2CWriteReg8(RTC, MIN, MM);

			SS = decCompensation(SS);
			secPWM(SS);
			wiringPiI2CWriteReg8(RTC, SEC,  0b10000000 + SS);
}
