/*!
* @file rs485.h
*
* Non-blocking driver for a half-duplex RS485 transceiver.
*
* print() queues the data in the UART TX buffer and returns immediately.
* update(), called from loop(), releases the bus (direction pin LOW) once the
* last stop bit has left the UART. The release time is estimated from the
* baudrate, and Serial.flush() is then called to guarantee it: on the SAMD
* core flush() waits for the TXC flag, so if the estimate is early it only
* blocks for the remaining bits.
*
* Licensed under the GPLv3
*
*/

#ifndef RS485_H_
#define RS485_H_

#include <Arduino.h>

class rs485 {
public:

    /*!
        @param pin_dir   Direction pin, HIGH = transmit
        @param guard_us  Extra time to keep the driver enabled after the
                         estimated end of transmission, in microseconds
    */
    rs485(uint8_t pin_dir, uint16_t guard_us) {
        _pin_dir = pin_dir;
        _guard_us = guard_us;
    }

    void begin(uint32_t baudrate) {
        _baudrate = baudrate;
        pinMode(_pin_dir, OUTPUT);
        digitalWrite(_pin_dir, LOW);
        Serial.begin(baudrate);
    }

    /*!
        @brief Queue a string for transmission, does not wait for it to be sent
    */
    void print(const String &str) {
        digitalWrite(_pin_dir, HIGH);
        size_t n = Serial.print(str);
        // 10 bit times per byte (8N1)
        uint32_t duration = (uint32_t)((uint64_t)n * 10UL * 1000000UL / _baudrate)
                            + _guard_us;
        uint32_t now = micros();
        if (_busy && (int32_t)(_deadline - now) > 0) {
            _deadline += duration;
        } else {
            _deadline = now + duration;
        }
        _busy = true;
    }

    /*!
        @brief Release the bus when the transmission is complete. Call often.
    */
    void update() {
        if (_busy && (int32_t)(micros() - _deadline) >= 0) {
            Serial.flush(); // returns at once if the estimate was right
            digitalWrite(_pin_dir, LOW);
            _busy = false;
        }
    }

    bool busy() { return _busy; }

    int read() { return Serial.read(); }

    int available() { return Serial.available(); }

    /*!
        @brief Blocking: wait for the end of transmission and release the bus
    */
    void flush() {
        Serial.flush();
        digitalWrite(_pin_dir, LOW);
        _busy = false;
    }

    void end() {
        flush();
        Serial.end();
    }

private:
    uint8_t _pin_dir;
    uint16_t _guard_us;
    uint32_t _baudrate = 19200;
    bool _busy = false;
    uint32_t _deadline = 0;
};

#endif /* RS485_H_ */
