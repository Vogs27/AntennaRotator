/*!
* @file watchdog.h
*
* Watchdog timer for SAMD21/SAMD51, based on Adafruit_SleepyDog.
*
* On SAMD the watchdog cannot run an interrupt routine before the reset (as
* the old AVR ISR did): the chip is simply reset. The firmware therefore
* checks the reset cause at boot and, if the watchdog fired, starts in the
* error state with ERR_WDT latched and the motors disabled. The easycomm
* handler keeps running, so GS/GE/RB/RESET still work.
*
* Licensed under the GPLv3
*
*/

#ifndef WATCHDOG_H_
#define WATCHDOG_H_

#include <Arduino.h>
#include <Adafruit_SleepyDog.h>

class wdt_timer {
public:

    /*!
        @brief  Enable the watchdog
        @param  timeout_ms  Requested timeout, rounded down to a power of two
                            of WDT cycles by SleepyDog (max ~16 s on SAMD)
        @return Actual timeout in ms
    */
    int watchdog_init(int timeout_ms) {
        return Watchdog.enable(timeout_ms);
    }

    void watchdog_reset() {
        Watchdog.reset();
    }

    /*!
        @brief  True if the last reset was caused by the watchdog
    */
    bool reset_by_watchdog() {
#if defined(__SAMD51__)
        return (Watchdog.resetCause() & RSTC_RCAUSE_WDT) != 0;
#else
        return (Watchdog.resetCause() & PM_RCAUSE_WDT) != 0;
#endif
    }
};

#endif /* WATCHDOG_H_ */
