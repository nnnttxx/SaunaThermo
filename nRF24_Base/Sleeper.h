#ifndef SLEEPER_H
#define SLEEPER_H

#include <arduino.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

// This sleeper class utilises the watchdog timer as the oscilators for the timers get powered off in SLEEP_MODE_PWR_DOWN
// mode, which is the most power effcient sleep mode. The sleep time will not be more than 16ms accurate.

class Sleeper {
	public:
	void SleepMillis(long millisec);

	private:
	void DoSleep();
	void SetupWatchdog(uint8_t prescalar);

	// Watchdog Prescalars
	const int static NumberOfPrescalars = 10;
	uint8_t Prescalars[NumberOfPrescalars] = {9, 8, 7, 6, 5, 4, 3, 2, 1, 0};
	long Times[NumberOfPrescalars] = {8000, 4000, 2000, 1000, 500, 250, 128, 64, 32, 16};
};

#endif


