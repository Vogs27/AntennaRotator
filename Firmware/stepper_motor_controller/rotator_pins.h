/*!
* @file rotator_pins.h
*
* Pin mapping for the RotatorMobo board rev 1.0a (ATSAMD21G18A), using the
* Arduino Zero variant pin numbers. Port and schematic net in the comments.
*
* Licensed under the GPLv3
*
*/

#ifndef ROTATOR_PINS_H_
#define ROTATOR_PINS_H_

#define M1IN1 10 ///< PA18 STEP_AZ
#define M1IN2 9  ///< PA07 DIR_AZ
#define M1SF  7  ///< PA21 M1SF, CN7 AUX2 pin 4, free GPIO
#define M1FB  A1 ///< PB08 M1FB, CN7 AUX2 pin 5, free GPIO / analog

#define M2IN1 11 ///< PA16 STEP_EL
#define M2IN2 3  ///< PA09 DIR_EL
#define M2SF  6  ///< PA20 M2SF, CN7 AUX2 pin 6, free GPIO
#define M2FB  A2 ///< PB09 M2FB, CN7 AUX2 pin 3, free GPIO / analog

#define MOTOR_EN 8 ///< PA06 STEPPER_EN -> ENN of both TMC2209, active low, R19 10k pull-up

#define DIAG_AZ_PIN 12 ///< PA19 DIAG of the azimuth TMC2209 (input)
#define DIAG_EL_PIN 13 ///< PA17 DIAG of the elevation TMC2209 (input)

#define SW1 5 ///< PA15 ESTOP_AZ, CN10
#define SW2 4 ///< PA08 ESTOP_EL, CN11

#define RS485_DIR 2 ///< PA14 RTS_L -> TXS0104 -> DE and #RE of SN65HVD485E, HIGH = transmit

/*
 * Serial  (PB22 TX_L / PB23 RX_L, SERCOM5): RS485 via TXS0104, or CN5 serial
 *         adapter when JP1 disables the level shifter.
 * Serial1 (PA10 TX through R15 1k / PA11 RX, SERCOM0): TMC_UART, single wire.
 * Wire    (PA22 SDA / PA23 SCL, SERCOM3, R13/R14 4k7 pull-ups): CN9, and the
 *         PCA9540 multiplexer for CN3/CN4/CN6, which is DNP (legacy SatNOGS).
 * SPI     (PB10 MOSI / PB11 SCK / PA12 MISO, SERCOM4): CN8.
 */

/*
 * AUX GPIO, used by the easycomm AN, OP and IP2x commands. The index is the
 * Arduino analog number for A0..A5 and the Arduino pin number for 6 and 7.
 *   0 A0   PA02  CN9 pin 1       analog/digital
 *   1 A1   PB08  CN7 pin 5 M1FB  analog/digital
 *   2 A2   PB09  CN7 pin 3 M2FB  analog/digital
 *   3 A3   PA04  CN9 pin 3       analog/digital
 *   4 A4   PA05  CN9 pin 5       analog/digital
 *   5 A5   PB02  CN9 pin 7       analog/digital
 *   6 D6   PA20  CN7 pin 6 M2SF  digital
 *   7 D7   PA21  CN7 pin 4 M1SF  digital
 */
#define AUX_GPIO_COUNT  8
#define AUX_ANALOG_COUNT 6
static const uint8_t aux_gpio[AUX_GPIO_COUNT] = { A0, A1, A2, A3, A4, A5, M2SF, M1SF };

#endif /* ROTATOR_PINS_H_ */
