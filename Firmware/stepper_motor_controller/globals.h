/*!
* @file globals.h
*
* Global state shared between the control loop (.ino) and the easycomm
* protocol handler (easycomm.h).
*
* Licensed under the GPLv3
*
*/

#ifndef LIBRARIES_GLOBALS_H_
#define LIBRARIES_GLOBALS_H_

#include <Arduino.h>

#define FW_VERSION "SatNOGS-stepper-3.0" ///< Reply to the VE command

/** Axis indices */
#define AXIS_AZ 0
#define AXIS_EL 1

/**
 * Status register (GS). Bits can be combined.
 * 1/2/4/8 follow the Easycomm III proposed mapping, 16 is an extension.
 */
#define STATUS_IDLE     1  ///< Not moving
#define STATUS_MOVING   2  ///< Free motion: velocity (Vx/Mx) commands or homing
#define STATUS_POINTING 4  ///< Slewing to a commanded AZ/EL position
#define STATUS_ERROR    8  ///< At least one latched error, see GE
#define STATUS_WARNING  16 ///< Extension: driver over-temperature pre-warning

/**
 * Error register (GE). Bits can be combined and are latched until RESET.
 * 0..4 follow the Easycomm III proposed mapping, 8..32 are extensions.
 */
#define ERR_NONE      0
#define ERR_SENSOR    1  ///< Sensor error (reserved, no position sensor fitted)
#define ERR_JAM       2  ///< Jam (reserved, stall detection not implemented)
#define ERR_HOMING    4  ///< End-stop not found or homing did not settle
#define ERR_MOTOR     8  ///< Driver fault: short, UART lost, driver reset, charge pump UV
#define ERR_OVER_TEMP 16 ///< Driver over-temperature (shutdown or pre-warning, see OTPW_IS_FAULT)
#define ERR_WDT       32 ///< The MCU was reset by the watchdog

/** Per-axis telemetry, written by the control loop, read by easycomm */
struct _axis_state {
    float position;      ///< Current position in deg (open loop, from step count)
    float speed;         ///< Current speed in deg/s
    float setpoint;      ///< Position set point in deg
    bool velocity_mode;  ///< true while executing a Vx/Mx command
    uint32_t drv_status; ///< Last valid TMC2209 DRV_STATUS register
};

struct _rotator {
    uint8_t status;           ///< STATUS_* bitmask
    uint8_t error;            ///< ERR_* bitmask, latched
    bool homed;               ///< Position reference is valid
    bool switch_az, switch_el;///< End-stop states
    int16_t driver_temp;      ///< Highest TMC2209 temperature threshold exceeded (0, 120, 143, 150, 157 degC)
    float park_az, park_el;   ///< Park position in deg
    float max_speed;          ///< Slew speed limit for AZ/EL commands in deg/s
    _axis_state axis[2];
};

_rotator rotator = {};

/* Motion API, implemented in stepper_motor_controller.ino */
void rotator_set_position(uint8_t ax, float deg);
void rotator_set_velocity(uint8_t ax, float deg_per_s);
void rotator_stop(uint8_t ax);
void rotator_park();
void rotator_reset();
bool rotator_set_max_speed(float deg_per_s);
float rotator_max_speed_limit();
bool rotator_position_valid(uint8_t ax, float deg);

#endif /* LIBRARIES_GLOBALS_H_ */
