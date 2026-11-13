/*!
* @file easycomm.h
*
* Easycomm II/III protocol handler, as specified in Hamlib
* (rotators/easycomm/easycomm.txt) and as used by the current Hamlib
* easycomm backend (rotctl -m 202 / -m 204).
*
* Parsing rules:
*  - A line ends with CR or LF. Inside a line, commands are separated by
*    spaces (per the spec), so "AZ10.0 EL20.0" and "SA SE" are two commands.
*    ';' and TAB are also treated as separators, because Hamlib's get_conf
*    sends a stray ';' after the newline (e.g. "GS\n;").
*  - All replies produced by one line are joined with a space and sent once,
*    terminated by '\n'. So the legacy query "AZ EL" gets "AZx.x ELy.y\n",
*    while the current Hamlib queries "AZ" and "EL" get one line each.
*  - Commands that only set something do not reply.
*  - The argument of IP, AN, OP, CR may follow the command directly or after
*    a space ("CR1" or "CR 1", the latter is what Hamlib sends).
*
* Licensed under the GPLv3
*
*/

#ifndef LIBRARIES_EASYCOMM_H_
#define LIBRARIES_EASYCOMM_H_

#include <Arduino.h>
#include <WString.h>
#include <math.h>
#include "rs485.h"
#include "rotator_pins.h"
#include "globals.h"

#define RS485_GUARD_US     100   ///< Keep the RS485 driver on this long after the last stop bit
#define BUFFER_SIZE        128   ///< Longest accepted line
#define MAX_TOKENS         16    ///< Max commands in one line
#define BAUDRATE           19200 ///< Easycomm serial baudrate
#define DEFAULT_MOVE_SPEED 5.0   ///< deg/s for the Easycomm II ML/MR/MU/MD commands

rs485 rs485(RS485_DIR, RS485_GUARD_US);

class easycomm {
public:

    void easycomm_init() {
        rs485.begin(BAUDRATE);
    }

    /*!
        @brief Read the bytes available on the bus and execute complete lines.
               Never blocks.
    */
    void easycomm_proc() {
        rs485.update();
        while (rs485.available() > 0) {
            char c = (char)rs485.read();
            if (c == '\n' || c == '\r') {
                if (_len > 0 && !_overflow) {
                    _buf[_len] = '\0';
                    process_line(_buf);
                }
                _len = 0;
                _overflow = false;
            } else if (_len < BUFFER_SIZE - 1) {
                _buf[_len++] = c;
            } else {
                _overflow = true; // drop the whole line
            }
        }
    }

private:
    enum { AUX_UNSET = 0, AUX_ANALOG, AUX_INPUT, AUX_OUTPUT };
    uint8_t _aux_mode[AUX_GPIO_COUNT] = { 0 };
    char _buf[BUFFER_SIZE];
    uint16_t _len = 0;
    bool _overflow = false;
    String _reply;

    void add_reply(const String &s) {
        if (_reply.length() > 0)
            _reply += ' ';
        _reply += s;
    }

    void process_line(char *line) {
        char *tok[MAX_TOKENS];
        uint8_t n = 0;
        char *save;
        char *t = strtok_r(line, " ;\t", &save);
        while (t != NULL && n < MAX_TOKENS) {
            tok[n++] = t;
            t = strtok_r(NULL, " ;\t", &save);
        }

        _reply = "";
        for (uint8_t i = 0; i < n; i++) {
            // Some commands take their argument as the next token
            const char *next = (i + 1 < n) ? tok[i + 1] : NULL;
            if (process_command(tok[i], next))
                i++; // next token consumed
        }

        if (_reply.length() > 0) {
            _reply += '\n';
            rs485.print(_reply);
        }
    }

    /*!
        @return true if the next token was consumed as argument
    */
    bool process_command(char *cmd, const char *next) {
        // Long commands first
        if (strcmp(cmd, "RESET") == 0) {
            rotator_reset();          // clear latched errors and re-home
            return false;
        }
        if (strcmp(cmd, "PARK") == 0) {
            rotator_park();
            return false;
        }
        if (strcmp(cmd, "RST") == 0) {
            // Custom: hang to test the watchdog
            rs485.flush();
            while (1)
                ;
        }
        if (strlen(cmd) < 2)
            return false;

        char c0 = cmd[0], c1 = cmd[1];
        const char *arg = cmd + 2;
        float val;

        // AZ / EL: query without argument, set with argument
        if ((c0 == 'A' && c1 == 'Z') || (c0 == 'E' && c1 == 'L')) {
            uint8_t ax = (c0 == 'A') ? AXIS_AZ : AXIS_EL;
            if (*arg == '\0') {
                add_reply(String(c0 == 'A' ? "AZ" : "EL")
                          + String(rotator.axis[ax].position, 1));
            } else if (parse_float(arg, val)) {
                rotator_set_position(ax, val);
            }
            return false;
        }

        // Stop
        if (c0 == 'S' && c1 == 'A' && *arg == '\0') { rotator_stop(AXIS_AZ); return false; }
        if (c0 == 'S' && c1 == 'E' && *arg == '\0') { rotator_stop(AXIS_EL); return false; }

        // Easycomm III velocity, mdeg/s. VL/VR = azimuth decrease/increase,
        // VU/VD = elevation increase/decrease (same as Hamlib ROT_MOVE_*).
        // A speed of 0 stops the axis.
        if (c0 == 'V' && (c1 == 'L' || c1 == 'R' || c1 == 'U' || c1 == 'D')) {
            if (parse_float(arg, val) && val >= 0) {
                float dps = val / 1000.0;
                if (c1 == 'L') rotator_set_velocity(AXIS_AZ, -dps);
                if (c1 == 'R') rotator_set_velocity(AXIS_AZ,  dps);
                if (c1 == 'U') rotator_set_velocity(AXIS_EL,  dps);
                if (c1 == 'D') rotator_set_velocity(AXIS_EL, -dps);
            }
            return false;
        }

        // Easycomm II move, at DEFAULT_MOVE_SPEED until SA/SE
        if (c0 == 'M' && *arg == '\0') {
            if (c1 == 'L') rotator_set_velocity(AXIS_AZ, -DEFAULT_MOVE_SPEED);
            if (c1 == 'R') rotator_set_velocity(AXIS_AZ,  DEFAULT_MOVE_SPEED);
            if (c1 == 'U') rotator_set_velocity(AXIS_EL,  DEFAULT_MOVE_SPEED);
            if (c1 == 'D') rotator_set_velocity(AXIS_EL, -DEFAULT_MOVE_SPEED);
            return false;
        }

        if (c0 == 'V' && c1 == 'E') {
            add_reply(String("VE") + FW_VERSION);
            return false;
        }
        if (c0 == 'G' && c1 == 'S') {
            add_reply(String("GS") + String(rotator.status));
            return false;
        }
        if (c0 == 'G' && c1 == 'E') {
            add_reply(String("GE") + String(rotator.error));
            return false;
        }

        // Commands with an index argument, possibly in the next token
        bool consumed = false;
        if (c0 == 'O' && c1 == 'P') {
            cmd_op(arg);
            return false;
        }

        if ((c0 == 'I' && c1 == 'P') || (c0 == 'A' && c1 == 'N') ||
            (c0 == 'C' && c1 == 'R')) {
            if (*arg == '\0' && next != NULL && is_index_token(next)) {
                arg = next;
                consumed = true;
            }
            if (*arg == '\0') {
                // Hamlib's get_conf sends "IP"/"AN" without an index
                add_reply(String("AL") + c0 + c1 + "_needs_index");
                return consumed;
            }
            if (c0 == 'I') cmd_ip(arg);
            else if (c0 == 'A') cmd_an(arg);
            else cmd_cr(arg);
            return consumed;
        }

        if (c0 == 'C' && c1 == 'W') {
            cmd_cw(arg);
            return false;
        }

        if (c0 == 'R' && c1 == 'B' && *arg == '\0') {
            // Custom: reboot
            rs485.flush();
            NVIC_SystemReset();
            while (1)
                ;
        }

        // UP DN DM UM DR UR AO LO ST: radio/time commands, accepted and ignored
        return false;
    }

    /* IPn: read input */
    void cmd_ip(const char *arg) {
        long idx;
        if (!parse_int(arg, idx))
            return;
        String v;
        switch (idx) {
        case 0:  v = String(rotator.driver_temp); break;
        case 1:  v = String(rotator.switch_az ? 1 : 0); break;
        case 2:  v = String(rotator.switch_el ? 1 : 0); break;
        case 3:  v = String(rotator.axis[AXIS_AZ].position, 2); break;
        case 4:  v = String(rotator.axis[AXIS_EL].position, 2); break;
        case 5:  v = String((rotator.axis[AXIS_AZ].drv_status >> 16) & 0x1F); break;
        case 6:  v = String((rotator.axis[AXIS_EL].drv_status >> 16) & 0x1F); break;
        case 7:  v = String(rotator.axis[AXIS_AZ].speed, 2); break;
        case 8:  v = String(rotator.axis[AXIS_EL].speed, 2); break;
        case 9:  v = String(rotator.axis[AXIS_AZ].drv_status & 0xFFF); break;
        case 10: v = String(rotator.axis[AXIS_EL].drv_status & 0xFFF); break;
        case 11: v = String((rotator.axis[AXIS_AZ].drv_status >> 30) & 1); break; // 1 = StealthChop
        case 12: v = String((rotator.axis[AXIS_EL].drv_status >> 30) & 1); break;
        default:
            if (idx >= 20 && idx < 20 + AUX_GPIO_COUNT) {
                // Digital level of an AUX pin. Outputs read back their real level.
                uint8_t n = idx - 20;
                if (_aux_mode[n] != AUX_OUTPUT && _aux_mode[n] != AUX_INPUT) {
                    pinMode(aux_gpio[n], INPUT);
                    _aux_mode[n] = AUX_INPUT;
                }
                v = String(digitalRead(aux_gpio[n]) == HIGH ? 1 : 0);
                break;
            }
            return;
        }
        add_reply(String("IP") + String(idx) + "," + v);
    }

    /* ANn: raw ADC reading of A0..A5 (A0/A3/A4/A5 on CN9, A1/A2 on CN7) */
    void cmd_an(const char *arg) {
        long idx;
        if (!parse_int(arg, idx) || idx < 0 || idx >= AUX_ANALOG_COUNT)
            return;
        if (_aux_mode[idx] == AUX_OUTPUT) {
            // analogRead() would switch the pin to the ADC and drop the output
            add_reply(String("ALAN") + String(idx) + "_is_output");
            return;
        }
        _aux_mode[idx] = AUX_ANALOG;
        add_reply(String("AN") + String(idx) + "," + String(analogRead(aux_gpio[idx])));
    }

    /* OPn,v: drive AUX GPIO n (0..7) low (v = 0) or high (v != 0) */
    void cmd_op(const char *arg) {
        const char *comma = strchr(arg, ',');
        if (comma == NULL || comma == arg)
            return;
        String s_idx = String(arg).substring(0, comma - arg);
        long idx, v;
        if (!parse_int(s_idx.c_str(), idx) || !parse_int(comma + 1, v))
            return;
        if (idx < 0 || idx >= AUX_GPIO_COUNT)
            return;
        // Set the level before enabling the driver, to avoid a glitch
        digitalWrite(aux_gpio[idx], v ? HIGH : LOW);
        if (_aux_mode[idx] != AUX_OUTPUT) {
            pinMode(aux_gpio[idx], OUTPUT);
            digitalWrite(aux_gpio[idx], v ? HIGH : LOW);
            _aux_mode[idx] = AUX_OUTPUT;
        }
    }

    /* CRr: read config register */
    void cmd_cr(const char *arg) {
        String v;
        if (strcmp(arg, "0") == 0)      v = String((long)lroundf(rotator.max_speed * 1000)); // mdeg/s
        else if (strcmp(arg, "a") == 0) v = "-"; // overshoot: not supported
        else if (strcmp(arg, "b") == 0) v = "-"; // jamming detection: not supported
        else if (strcmp(arg, "c") == 0) v = "1"; // endpoints: end-stops are used for homing
        else if (strcmp(arg, "d") == 0) v = "-"; // unstick: not supported
        else if (strcmp(arg, "7") == 0) v = String(rotator.park_az, 2);
        else if (strcmp(arg, "8") == 0) v = String(rotator.park_el, 2);
        else if (strcmp(arg, "9") == 0)
            v = String((rotator.axis[AXIS_AZ].velocity_mode ||
                        rotator.axis[AXIS_EL].velocity_mode) ? 1 : 0);
        else
            return;
        add_reply(String("CR") + arg + "," + v);
    }

    /* CWr,v: write config register */
    void cmd_cw(const char *arg) {
        const char *comma = strchr(arg, ',');
        if (comma == NULL || comma == arg)
            return;
        String reg = String(arg).substring(0, comma - arg);
        float val;
        if (!parse_float(comma + 1, val))
            return;
        if (reg == "0") {
            rotator_set_max_speed(val / 1000.0); // mdeg/s
        } else if (reg == "7") {
            if (rotator_position_valid(AXIS_AZ, val)) rotator.park_az = val;
        } else if (reg == "8") {
            if (rotator_position_valid(AXIS_EL, val)) rotator.park_el = val;
        }
    }

    static bool parse_float(const char *s, float &out) {
        if (s == NULL || *s == '\0')
            return false;
        char *end;
        double v = strtod(s, &end);
        if (end == s || *end != '\0' || !isfinite(v))
            return false;
        out = (float)v;
        return true;
    }

    static bool parse_int(const char *s, long &out) {
        if (s == NULL || *s == '\0')
            return false;
        char *end;
        long v = strtol(s, &end, 10);
        if (end == s || *end != '\0')
            return false;
        out = v;
        return true;
    }

    /* "1", "10" or a single register letter like "a" */
    static bool is_index_token(const char *s) {
        size_t l = strlen(s);
        if (l == 0 || l > 2)
            return false;
        if (l == 1 && isalpha(s[0]) && islower(s[0]))
            return true;
        for (size_t i = 0; i < l; i++)
            if (!isdigit(s[i]))
                return false;
        return true;
    }
};

#endif /* LIBRARIES_EASYCOMM_H_ */
