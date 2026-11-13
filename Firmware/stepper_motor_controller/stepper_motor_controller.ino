/*!
   @file stepper_motor_controller.ino

   SatNOGS rotator controller firmware, stepper motor version, for a SAMD21
   board with two TMC2209 drivers on a shared UART (Serial1).

   Dependencies: AccelStepper, TMCStepper, Adafruit_SleepyDog.

   Licensed under the GPLv3.
*/

/* ---------------------------------------------------------------- mechanics */
#define RATIO              54     ///< Gear ratio of rotator gear box
#define MICROSTEP          16     ///< Driver microstepping
#define MIN_PULSE_WIDTH    1      ///< In microsecond for AccelStepper
#define MAX_SPEED          6400   ///< In steps/s, hard speed limit (microsteps)
#define MAX_ACCELERATION   3200   ///< In steps/s^2 (microsteps)
#define SPR                3200   ///< Steps per motor revolution: 200 * MICROSTEP
#define MIN_M1_ANGLE       0      ///< Minimum angle of azimuth
#define MAX_M1_ANGLE       360    ///< Maximum angle of azimuth
#define MIN_M2_ANGLE       0      ///< Minimum angle of elevation
#define MAX_M2_ANGLE       180    ///< Maximum angle of elevation
#define DEFAULT_HOME_STATE HIGH   ///< Change to LOW according to Home sensor

/* ------------------------------------------------------------------- homing */
#define HOMING_SPEED       MAX_SPEED ///< steps/s while seeking the end-stops
#define HOME_DELAY         12000  ///< ms allowed to decelerate and return to the switch point

/* ------------------------------------------------------------ TMC2209 drivers */
#define AZ_DRIVER_ADDRESS  0b00   ///< TMC2209 address according to MS1 and MS2
#define EL_DRIVER_ADDRESS  0b01   ///< TMC2209 address according to MS1 and MS2
#define R_SENSE            0.11f  ///< Match to your driver
#define MOTOR_CURRENT_MA   1700   ///< RMS run current
#define HOLD_MULTIPLIER    0.5    ///< Hold current = run current * this
#define TMC_BAUD           115200 ///< TMC2209 auto-detects the baudrate

/* ------------------------------------------------------------------- safety */
#define WDT_TIMEOUT_MS        2000  ///< Watchdog timeout
#define WDT_LATCH_ERROR       1     ///< 1: after a watchdog reset stay in error until RESET. 0: home and continue
#define DIAG_PERIOD_MS        250   ///< One driver is polled per period (each driver every 2 periods)
#define DIAG_MAX_STEP_RATE    1000  ///< steps/s. Faster than this, polling is deferred (see README)
#define DIAG_MAX_COMM_FAILS   3     ///< Consecutive failed UART reads before ERR_MOTOR
#define OTPW_IS_FAULT         1     ///< 1: the 120 degC pre-warning stops the rotator. 0: only sets STATUS_WARNING
#define DIAG_DEBOUNCE_US      1000  ///< A DIAG pin must stay high this long to be considered

#include <AccelStepper.h>
#include <TMCStepper.h>
#include <Wire.h>
#include "globals.h"
#include "rotator_pins.h"
#include "rs485.h"
#include "easycomm.h"
#include "endstop.h"
#include "watchdog.h"

#define STEPS_PER_DEG ((float)RATIO * SPR / 360.0f)

/* TMC2209 register bits (datasheet DRV_STATUS 0x6F / GSTAT 0x01) */
#define DRV_OTPW   (1UL << 0)
#define DRV_OT     (1UL << 1)
#define DRV_S2GA   (1UL << 2)
#define DRV_S2GB   (1UL << 3)
#define DRV_S2VSA  (1UL << 4)
#define DRV_S2VSB  (1UL << 5)
#define DRV_T120   (1UL << 8)
#define DRV_T143   (1UL << 9)
#define DRV_T150   (1UL << 10)
#define DRV_T157   (1UL << 11)
#define DRV_SHORTS (DRV_S2GA | DRV_S2GB | DRV_S2VSA | DRV_S2VSB)
#define GSTAT_RESET   (1 << 0)
#define GSTAT_DRV_ERR (1 << 1)
#define GSTAT_UV_CP   (1 << 2)

enum _motion_state { ST_HOMING_SEEK, ST_HOMING_SETTLE, ST_READY, ST_FAULT };

easycomm comm;
TMC2209Stepper driver_az(&Serial1, R_SENSE, AZ_DRIVER_ADDRESS);
TMC2209Stepper driver_el(&Serial1, R_SENSE, EL_DRIVER_ADDRESS);
AccelStepper stepper_az(AccelStepper::DRIVER, M1IN1, M1IN2);
AccelStepper stepper_el(AccelStepper::DRIVER, M2IN1, M2IN2);
endstop switch_az(SW1, DEFAULT_HOME_STATE), switch_el(SW2, DEFAULT_HOME_STATE);
wdt_timer wdt;

AccelStepper *steppers[2] = { &stepper_az, &stepper_el };
TMC2209Stepper *drivers[2] = { &driver_az, &driver_el };
endstop *switches[2] = { &switch_az, &switch_el };
const float min_angle[2] = { MIN_M1_ANGLE, MIN_M2_ANGLE };
const float max_angle[2] = { MAX_M1_ANGLE, MAX_M2_ANGLE };
const uint8_t diag_pins[2] = { DIAG_AZ_PIN, DIAG_EL_PIN };

_motion_state state = ST_FAULT;
bool home_found[2];
uint32_t t_state = 0;
uint8_t comm_fails[2] = { 0, 0 };
bool otpw_active[2] = { false, false };

int32_t deg2step(float deg) { return lroundf(deg * STEPS_PER_DEG); }
float step2deg(int32_t step) { return step / STEPS_PER_DEG; }

bool driver_configure(TMC2209Stepper &d);
void driver_check(uint8_t ax);
void diagnostics_poll();
void diag_pins_check();
void enable_outputs();
void disable_outputs();
void enter_fault(uint8_t err);
void start_rotator();
void motion_update();
void update_status();

/* ======================================================================= */

void setup() {
  switch_az.init();
  switch_el.init();
  // TMC2209 DIAG outputs. Pull-down so an unpowered driver reads low.
  pinMode(DIAG_AZ_PIN, INPUT_PULLDOWN);
  pinMode(DIAG_EL_PIN, INPUT_PULLDOWN);

  comm.easycomm_init();
  Serial1.begin(TMC_BAUD);

  stepper_az.setEnablePin(MOTOR_EN);
  stepper_az.setPinsInverted(false, false, true);
  for (uint8_t ax = 0; ax < 2; ax++) {
    steppers[ax]->setAcceleration(MAX_ACCELERATION);
    steppers[ax]->setMinPulseWidth(MIN_PULSE_WIDTH);
  }
  disable_outputs();

  rotator.max_speed = MAX_SPEED / STEPS_PER_DEG;

  bool by_wdt = wdt.reset_by_watchdog();
  wdt.watchdog_init(WDT_TIMEOUT_MS);

  if (by_wdt && WDT_LATCH_ERROR) {
    enter_fault(ERR_WDT);
  } else {
    if (by_wdt)
      rotator.error |= ERR_WDT; // reported, but not blocking
    start_rotator();
  }
}

void loop() {
  wdt.watchdog_reset();

  rotator.switch_az = switch_az.get_state();
  rotator.switch_el = switch_el.get_state();

  comm.easycomm_proc(); 
  diag_pins_check();      // TMC2209 DIAG pins, every loop
  diagnostics_poll();     // TMC2209 health and temperature over UART, only in low speed scenarios
  motion_update();        // homing state machine and stepping

  for (uint8_t ax = 0; ax < 2; ax++) {
    rotator.axis[ax].position = step2deg(steppers[ax]->currentPosition());
    rotator.axis[ax].speed = steppers[ax]->speed() / STEPS_PER_DEG;
  }
  update_status();
}

/* ============================================================== drivers */

/*!
    @brief Configure one TMC2209 and check that it answers on the UART
    @return true if the driver answers
*/
bool driver_configure(TMC2209Stepper &d) {
  d.begin();
  d.toff(5);                                   // enable driver in software
  d.I_scale_analog(false);                     // VREF is not connected on this board: use the internal reference
  d.rms_current(MOTOR_CURRENT_MA, HOLD_MULTIPLIER);
  d.pwm_autoscale(true);                       // StealthChop2 automatic current regulation
  d.pwm_autograd(true);
  d.microsteps(MICROSTEP);
  d.intpol(true);                              // interpolate to 256 microsteps
  // Chopper mode is chosen by the SPREAD solder jumpers (JP7 AZ, JP8 EL).
  // A high SPREAD pin inverts GCONF.en_spreadCycle; with the register at 0
  // the pin alone decides: GND = StealthChop, 3V3 = SpreadCycle.
  d.en_spreadCycle(false);
  d.TPWMTHRS(0);                               // no velocity switch: one chopper mode at all speeds
  d.TCOOLTHRS(0);                              // StallGuard output on DIAG disabled
  d.SGTHRS(0);
  d.GSTAT(GSTAT_RESET | GSTAT_DRV_ERR | GSTAT_UV_CP); // write 1 to clear
  // Read back a register we just wrote: catches a missing or unpowered driver
  return d.test_connection() == 0 && d.microsteps() == MICROSTEP;
}

/*!
    @brief Read DRV_STATUS and GSTAT of one driver and latch errors.
           Blocking for a few ms (TMCStepper waits for the reply).
*/
void driver_check(uint8_t ax) {
  TMC2209Stepper &d = *drivers[ax];
  uint32_t s = d.DRV_STATUS();
  bool fail = d.CRCerror;
  uint8_t g = d.GSTAT();
  fail |= d.CRCerror;

  if (fail) {
    if (++comm_fails[ax] >= DIAG_MAX_COMM_FAILS)
      enter_fault(ERR_MOTOR);
    return;
  }
  comm_fails[ax] = 0;
  rotator.axis[ax].drv_status = s;

  // GSTAT.reset: the driver lost its configuration (e.g. VM brown-out),
  // microstepping and current are back to defaults -> position is invalid.
  if (g & (GSTAT_RESET | GSTAT_DRV_ERR | GSTAT_UV_CP))
    enter_fault(ERR_MOTOR);
  if (s & DRV_SHORTS)
    enter_fault(ERR_MOTOR);

  otpw_active[ax] = (s & DRV_OTPW) != 0;
  if (s & DRV_OT)
    enter_fault(ERR_OVER_TEMP);
  else if ((s & DRV_OTPW) && OTPW_IS_FAULT)
    enter_fault(ERR_OVER_TEMP);

  // Highest threshold exceeded by either driver
  int16_t t = 0;
  for (uint8_t i = 0; i < 2; i++) {
    uint32_t r = rotator.axis[i].drv_status;
    if (r & DRV_T157) t = max(t, (int16_t)157);
    else if (r & DRV_T150) t = max(t, (int16_t)150);
    else if (r & DRV_T143) t = max(t, (int16_t)143);
    else if (r & DRV_T120) t = max(t, (int16_t)120);
  }
  rotator.driver_temp = t;
}

/*!
    @brief Check the DIAG pins. The TMC2209 raises DIAG on a driver error
           (over-temperature shutdown, short), so this catches faults even
           during fast slews, when the UART polling is deferred. The UART
           check that follows classifies the error.
*/
void diag_pins_check() {
  static uint32_t t_high[2] = { 0, 0 };
  static bool high[2] = { false, false };
  for (uint8_t ax = 0; ax < 2; ax++) {
    if (digitalRead(diag_pins[ax]) == HIGH) {
      if (!high[ax]) {
        high[ax] = true;
        t_high[ax] = micros();
      } else if (micros() - t_high[ax] >= DIAG_DEBOUNCE_US && state != ST_FAULT) {
        driver_check(ax);              // sets ERR_OVER_TEMP / ERR_MOTOR if it can tell
        if (state != ST_FAULT)
          enter_fault(ERR_MOTOR);      // DIAG is high but the cause is unknown: stop anyway
      }
    } else {
      high[ax] = false;
    }
  }
}

/*!
    @brief Poll the drivers periodically, one per call, never during fast
           moves because the UART read stalls the step generation.
*/
void diagnostics_poll() {
  static uint32_t t_last = 0;
  static uint8_t next_ax = 0;

  if (millis() - t_last < DIAG_PERIOD_MS)
    return;
  if (fabs(stepper_az.speed()) > DIAG_MAX_STEP_RATE ||
      fabs(stepper_el.speed()) > DIAG_MAX_STEP_RATE)
    return;
  t_last = millis();
  driver_check(next_ax);
  next_ax ^= 1;
}

/* ============================================================== motion */

void enable_outputs() {
  stepper_az.enableOutputs();   // drives MOTOR_EN, shared by both drivers
  stepper_el.enableOutputs();
}

void disable_outputs() {
  stepper_az.disableOutputs();
  stepper_el.disableOutputs();
}

/*!
    @brief Latch an error, stop and disable the motors. The position
           reference is lost, a RESET (re-homing) is required.
*/
void enter_fault(uint8_t err) {
  rotator.error |= err;
  if (state == ST_FAULT)
    return;
  state = ST_FAULT;
  rotator.homed = false;
  disable_outputs();
  for (uint8_t ax = 0; ax < 2; ax++) {
    // setCurrentPosition() also zeroes the speed: hard stop of the generator
    steppers[ax]->setCurrentPosition(steppers[ax]->currentPosition());
    rotator.axis[ax].velocity_mode = false;
    drivers[ax]->toff(0);       // also disable the power stage via UART
  }
}

/*!
    @brief Configure the drivers and start homing
*/
void start_rotator() {
  for (uint8_t ax = 0; ax < 2; ax++) {
    comm_fails[ax] = 0;
    if (!driver_configure(*drivers[ax])) {
      enter_fault(ERR_MOTOR);
      return;
    }
  }
  // Seek the end-stops: move by the full range towards the minimum
  for (uint8_t ax = 0; ax < 2; ax++) {
    home_found[ax] = false;
    rotator.axis[ax].velocity_mode = false;
    steppers[ax]->setCurrentPosition(0);
    steppers[ax]->setMaxSpeed(HOMING_SPEED);
    steppers[ax]->moveTo(deg2step(-(max_angle[ax] - min_angle[ax])));
  }
  rotator.homed = false;
  enable_outputs();
  state = ST_HOMING_SEEK;
  t_state = millis();
}

void motion_update() {
  switch (state) {
  case ST_HOMING_SEEK:
    for (uint8_t ax = 0; ax < 2; ax++) {
      if (home_found[ax])
        continue;
      if (switches[ax]->get_state()) {
        // Target = trigger point: AccelStepper decelerates past it and returns
        steppers[ax]->moveTo(steppers[ax]->currentPosition());
        home_found[ax] = true;
      } else if (steppers[ax]->distanceToGo() == 0) {
        enter_fault(ERR_HOMING); // full range travelled, no end-stop
        return;
      }
    }
    if (home_found[AXIS_AZ] && home_found[AXIS_EL]) {
      state = ST_HOMING_SETTLE;
      t_state = millis();
    }
    stepper_az.run();
    stepper_el.run();
    break;

  case ST_HOMING_SETTLE:
    stepper_az.run();
    stepper_el.run();
    if (!stepper_az.isRunning() && !stepper_el.isRunning()) {
      for (uint8_t ax = 0; ax < 2; ax++) {
        steppers[ax]->setCurrentPosition(deg2step(min_angle[ax]));
        steppers[ax]->setMaxSpeed(rotator.max_speed * STEPS_PER_DEG);
        // Go to the set point received during homing, if any
        steppers[ax]->moveTo(deg2step(rotator.axis[ax].setpoint));
      }
      rotator.homed = true;
      state = ST_READY;
    } else if (millis() - t_state > HOME_DELAY) {
      enter_fault(ERR_HOMING);
    }
    break;

  case ST_READY:
    stepper_az.run();
    stepper_el.run();
    for (uint8_t ax = 0; ax < 2; ax++) {
      // A velocity move ends at the travel limit
      if (rotator.axis[ax].velocity_mode && !steppers[ax]->isRunning()) {
        rotator.axis[ax].velocity_mode = false;
        rotator.axis[ax].setpoint = step2deg(steppers[ax]->currentPosition());
      }
    }
    break;

  case ST_FAULT:
  default:
    break;
  }
}

void update_status() {
  uint8_t s;
  if (state == ST_FAULT) {
    s = STATUS_ERROR;
  } else if (state != ST_READY) {
    s = STATUS_MOVING; // homing
  } else if (rotator.axis[AXIS_AZ].velocity_mode || rotator.axis[AXIS_EL].velocity_mode) {
    s = STATUS_MOVING;
  } else if (stepper_az.isRunning() || stepper_el.isRunning()) {
    s = STATUS_POINTING;
  } else {
    s = STATUS_IDLE;
  }
  if (otpw_active[AXIS_AZ] || otpw_active[AXIS_EL])
    s |= STATUS_WARNING;
  rotator.status = s;
}

/* ============================================ motion API used by easycomm */

bool rotator_position_valid(uint8_t ax, float deg) {
  return deg >= min_angle[ax] && deg <= max_angle[ax];
}

float rotator_max_speed_limit() {
  return MAX_SPEED / STEPS_PER_DEG;
}

void rotator_set_position(uint8_t ax, float deg) {
  deg = constrain(deg, min_angle[ax], max_angle[ax]);
  rotator.axis[ax].setpoint = deg;     // also remembered during homing
  if (state != ST_READY)
    return;
  rotator.axis[ax].velocity_mode = false;
  steppers[ax]->setMaxSpeed(rotator.max_speed * STEPS_PER_DEG);
  steppers[ax]->moveTo(deg2step(deg));
}

void rotator_set_velocity(uint8_t ax, float deg_per_s) {
  if (state != ST_READY)
    return;
  if (deg_per_s == 0) {
    rotator_stop(ax);
    return;
  }
  float steps_s = min(fabs(deg_per_s) * STEPS_PER_DEG, (float)MAX_SPEED);
  rotator.axis[ax].velocity_mode = true;
  steppers[ax]->setMaxSpeed(steps_s);
  // Run towards the travel limit, with acceleration and automatic stop there
  steppers[ax]->moveTo(deg2step(deg_per_s > 0 ? max_angle[ax] : min_angle[ax]));
}

void rotator_stop(uint8_t ax) {
  if (state != ST_READY)
    return;
  steppers[ax]->stop();                // decelerate
  rotator.axis[ax].velocity_mode = false;
  rotator.axis[ax].setpoint = step2deg(steppers[ax]->targetPosition());
}

void rotator_park() {
  rotator_set_position(AXIS_AZ, rotator.park_az);
  rotator_set_position(AXIS_EL, rotator.park_el);
}

/*!
    @brief Clear the latched errors and re-home. An over-temperature error is
           not cleared while a driver still reports it.
*/
void rotator_reset() {
  for (uint8_t ax = 0; ax < 2; ax++) {
    uint32_t s = drivers[ax]->DRV_STATUS();
    if (!drivers[ax]->CRCerror) {
      rotator.axis[ax].drv_status = s;
      otpw_active[ax] = (s & DRV_OTPW) != 0;
      if ((s & DRV_OT) || ((s & DRV_OTPW) && OTPW_IS_FAULT))
        return;  // still too hot, stay in fault
    }
  }
  rotator.error = ERR_NONE;
  state = ST_HOMING_SEEK; // leave ST_FAULT so a new fault is latched again
  start_rotator();
}

bool rotator_set_max_speed(float deg_per_s) {
  if (!(deg_per_s > 0))
    return false;
  rotator.max_speed = min(deg_per_s, rotator_max_speed_limit());
  for (uint8_t ax = 0; ax < 2; ax++) {
    if (state == ST_READY && !rotator.axis[ax].velocity_mode)
      steppers[ax]->setMaxSpeed(rotator.max_speed * STEPS_PER_DEG);
  }
  return true;
}
