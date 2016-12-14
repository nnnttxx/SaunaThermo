#include "Sleeper.h"

// The interupt vector for the watch dog must be present, even if empty else the CPU resets.
ISR(WDT_vect) {
    // Do nothing.
}

/// Sleeps the arduino for a number of milliseconds
void Sleeper::SleepMillis(long millis) {

	// OPTIONAL delay to wait for all things to finish, e.g. serial prints - else you may get garbled serial prints from sleeping before the sending has finished.
    delay(50);
    
	uint8_t prescalar = 0;

	// Sleep for the longest possible watchdog timeout that's less than millis and keep going until there are no millis left.
	while (millis > Times[NumberOfPrescalars-1]) 
	{
		for (int i = 0; i < NumberOfPrescalars; ++i) 
		{
			if (millis > Times[i]) {
				prescalar = Prescalars[i];
				millis -= Times[i];
				break;
			}
		}
   
    SetupWatchdog(prescalar);  
		DoSleep();
	}
}

/// Sets up the watchdog to timeout after a certain time period.
/// There are many comments / notes in this function which have been copied directly from the data sheet for
/// user convenience.
void Sleeper::SetupWatchdog(uint8_t prescalar) {

	// Prescalars can be: 0=16ms, 1=32ms, 2=64ms, 3=125ms, 4=250ms, 5=500ms, 6=1sec, 7=2sec, 8=4sec, 9=8sec
	if (prescalar > 9) {
		prescalar = 9;
	}

    // _BV is a macro that can be thought of as a funtion that takes in a number and outputs a byte with that bit set.
    
    // WDTCSR - Watchdog Timer Control Register
    // Note that WDP[0-3] is not in order (WDP[0-2] is but WDP3 is actually bit 5 not 3!) so we have to preprocess the prescaler passed in above.
    // bits 7 = WDIF, 6 = WDIE, 5 = WDP3, 4 = WDCE, 3 =  WDE, 2 =  WDP2, 1 = WDP1, 0 = WDP0
	
	// WDP3 WDP2    WDP1    WDP0        Typical Time-out at VCC = 5.0V
    // 0    0       0       0           16ms
    // 0    0       0       1           32ms
    // 0    0       1       0           64ms
    // 0    0       1       1           0.125 s
    // 0    1       0       0           0.25 s
    // 0    1       0       1           0.5 s
    // 0    1       1       0           1.0 s
    // 0    1       1       1           2.0 s
    // 1    0       0       0           4.0 s
    // 1    0       0       1           8.0 s
	
	// Take the first 3 bits (WDP[0-2])
	uint8_t wdtPrescalarBits = prescalar & 7;

    // Now we need to set WDP3, to do this we don't set bit 3 but bit 5, so if our presclar had bit 8 set i.e. it 
    // was 8 or 9 being passed in then we must set WDP3 accordingly, else we could have just used prescar as it was passed in. 
	if ( prescalar & 8 ) {
		wdtPrescalarBits |= _BV(WDP3);
	}
    
    // MCUSR – MCU Status Register
    // The MCU Status Register provides information on which reset source caused an MCU reset.
	// MCUSR Bit 3 – WDRF: Watchdog System Reset Flag
    // This bit is set if a Watchdog System Reset occurs. The bit is reset by a Power-on Reset, or by writing a logic zero to the flag.
	MCUSR &= ~_BV(WDRF);
    
    // WDTCSR Bit 4 – WDCE: Watchdog Change Enable
    // This bit is used in timed sequences for changing WDE and prescaler bits. To clear the WDE bit, and/or change the prescaler bits, WDCE must be set.
    // Once written to one, hardware will clear WDCE after four clock cycles.
	
    // WDTCSR Bit 3 – WDE: Watchdog System Reset Enable
    // WDE is overridden by WDRF in MCUSR. This means that WDE is always set when WDRF is set. To clear
    // WDE, WDRF must be cleared first. This feature ensures multiple resets during conditions causing failure, and a
    // safe start-up after the failure

	// Allow changes
	WDTCSR = _BV(WDCE) | _BV(WDE);

    // WDTCSR Bit 6 – WDIE: Watchdog Interrupt Enable
    // When this bit is written to one and the I-bit in the Status Register is set, the Watchdog Interrupt is enabled. If WDE is cleared in combination with this setting, the Watchdog Timer is in Interrupt Mode, and the corresponding interrupt is executed if time-out in the Watchdog Timer occurs. If WDE is set, the Watchdog Timer is in Interrupt and System Reset Mode. The first time-out in the Watchdog Timer will set WDIF. Executing the corresponding interrupt vector will clear WDIE and WDIF automatically by hardware (the Watchdog goes to System Reset Mode). This is useful for keeping the Watchdog Timer security while using the interrupt. To stay in Interrupt and System Reset Mode, WDIE must be set after each interrupt. This should however not be done within the interrupt service routine itself, as this might compromise the safety-function of the Watchdog System Reset mode. If the interrupt is not executed before the next time-out, a System Reset will be applied.
    // Note: 1. WDTON Fuse set to "0" means programmed and "1" means unprogrammed.

    // Watchdog Timer Configuration
    //WDTON WDE WDIE    Mode                                Action on Time-out
    //1     0   0       Stopped                             None
    //1     0   1       Interrupt Mode                      Interrupt
    //1     1   0       System Reset Mode                   Reset
    //1     1   1       Interrupt and System Reset Mode     Interrupt, then go to System Reset Mode
    //0     x   x       System Reset Mode                   Reset
    
    // Perform the change.
	WDTCSR = _BV(WDCE) | wdtPrescalarBits | _BV(WDIE);
}

/// Powers down system.
void Sleeper::DoSleep() {

	// Set the sleep mode.
	set_sleep_mode(SLEEP_MODE_PWR_DOWN); 
	sleep_enable();

	// Put the device into sleep mode.
	sleep_mode();

	// System continues execution here after watchdog timeout.
	sleep_disable();
}
