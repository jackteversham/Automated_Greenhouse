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
//int HH, MM, SS;
int hours, mins, secs;
int increment = 1000; //Default of 1 second
int choice = 0;

void initGPIO(void){
	/*
	 * Sets GPIO using wiringPi pins. see pinout.xyz for specific wiringPi pins
	 * You can also use "gpio readall" in the command line to get the pins
	 * Note: wiringPi does not use GPIO or board pin numbers (unless specifically set to that mode)
	 */
	printf("Setting Up\n");
	wiringPiSetup(); //This is the default mode. If you want to change pinouts, be aware

	RTC = wiringPiI2CSetup(RTCAddr); //Set up the RTC
	wiringPiSPISetup(SPI_CHAN1, 500000); //setup the SPI
	mcp3004Setup(BASE, SPI_CHAN1); //setup mcp3004 library

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

	printf("--------------------------------------------------------------------------------------------------\n");
	printf("|   RTC Time   |   Sys Timer   |   Humidity (V)|   Temp (C) |  Light  |   DAC Out (V)|   Alarm   |\n");
	printf("--------------------------------------------------------------------------------------------------\n");

}

/*
 * Method associated with RESET button interrpt
 */
void reset(void){
	//Debounce
	long interruptTime = millis();

	if (interruptTime - lastInterruptTime>bounce){
		printf("Reset Interrupt Triggered\n");
		digitalWrite(RESET_LED, HIGH);
		delay(1000);
		digitalWrite(RESET_LED, LOW);

		wiringPiI2CWriteReg8(RTC, HOUR, 0x0);
    wiringPiI2CWriteReg8(RTC, MIN, 0x0);
    wiringPiI2CWriteReg8(RTC, SEC, 0b10000000);

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
		printf("Frequency Interrupt Triggered\n");
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
		printf("Dismiss Alarm Interrupt Triggered\n");
		digitalWrite(ALARM_DISMISS_LED, HIGH);
		delay(1000);
		digitalWrite(ALARM_DISMISS_LED, LOW);

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
		printf("Monitoring Interrupt Triggered\n");
		digitalWrite(START_LED, HIGH);
		delay(1000);
		digitalWrite(START_LED, LOW);

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
  // Set the play thread to have a 99 priority
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

	secPWM(SS);

	// Print out the time we have stored on our RTC
	//printf("The current time is: %d:%d:%d\n", hexCompensation(HH), hexCompensation(MM), hexCompensation(SS));

	//Using a delay to make our program less CPU 'hungry'


	pthread_join(thread_id, NULL);
  pthread_exit(NULL);

	delay(1000); //milliseconds

	return 0;
}

void *monitorThread(void *threadargs){
    for(;;){

			//Fetch the time from the RTC
			HH = wiringPiI2CReadReg8(RTC, HOUR); //read SEC register from RTC
			MM = wiringPiI2CReadReg8(RTC, MIN); //read MIN register from RTC
			SS = wiringPiI2CReadReg8(RTC, SEC); //read the SEC register from RTC

			hours = hexCompensation(HH);
			mins = hexCompensation(MM);
			secs = hexCompensation(SS & 0b01111111);

			hours = hFormat(hours);
      mins = mFormat(mins);

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
			float temperatureInCelsius = (temperatureVolts - (500.0 / 1000.0)) / (10.0 / 1000.0);
			//printf("The temperature is: %f\n", temperatureVolts);

			int humidityReading = analogRead(BASE+1); //humidity from pot on channel 1
			float humidity = (humidityReading*3.3)/1024.0;
			//printf("The humidity is: %f\n", humidity);

			int light = analogRead(BASE+2); //light from LDR on channel 2
			float dacOut =0;

			printf("    %-17s%-17s%-14.2f%-10.2f%-12d%-14.2f%-14s\n", currentTime.c_str(), systemTime.c_str(), humidity, temperatureInCelsius, light, dacOut, " ");
			//printf("--------------------------------------------------------------------------------------------------\n");

			delay(increment);

		}
    pthread_exit(NULL);
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
	//softPwmCreate(SECS, 0, 1023);
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

//This interrupt will fetch current time from another script and write it to the clock registers
//This functions will toggle a flag that is checked in main
void toggleTime(void){
	long interruptTime = millis();

	if (interruptTime - lastInterruptTime>bounce){
		HH = getHours();
		MM = getMins();
		SS = getSecs();

		HH = hFormat(HH);
		HH = decCompensation(HH);
		wiringPiI2CWriteReg8(RTC, HOUR, HH);

		MM = decCompensation(MM);
		wiringPiI2CWriteReg8(RTC, MIN, MM);

		SS = decCompensation(SS);
		wiringPiI2CWriteReg8(RTC, SEC, SS);

	}
	lastInterruptTime = interruptTime;
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
