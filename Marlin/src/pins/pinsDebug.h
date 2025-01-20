/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../inc/MarlinConfig.h"

#define MAX_NAME_LENGTH  39    // one place to specify the format of all the sources of names
                               // "-" left justify, "39" minimum width of name, pad with blanks

/**
 *  Diese Routine vermindert die RAM durch erstellen eines Arrays im FLASH-Speicher, um die
 *  Pin-Namen, Pin-Nummern und durch die Erkennung, ob der Pin analog/digital ist.
 *
 *  Die ERstellung, des Arrays im FLASH, erfolgt in zwei Schritten. Der erste Schritt schreibt 
 *  die Pin-Namen, als String, in den FLASH. Der zweite Schritt erstellt dann das Array.
 *
 *  Beide Schritte nutzen die gleiche Pin-Liste. Die Liste enthält zwei Makro-Namen. 
 *  Passend  zum jeweiligen Schritt, wird der Name gewählt.
 */

// Der erste Schritt- Schreiben der Namen, als String ins Array

#define _ADD_PIN_2(PIN_NAME, ENTRY_NAME) static const char ENTRY_NAME[] PROGMEM = { PIN_NAME };
#define _ADD_PIN(PIN_NAME, COUNTER) _ADD_PIN_2(PIN_NAME, entry_NAME_##COUNTER)
#define REPORT_NAME_DIGITAL(COUNTER, NAME) _ADD_PIN(#NAME, COUNTER)
#define REPORT_NAME_ANALOG(COUNTER, NAME) _ADD_PIN(#NAME, COUNTER)

#include "pinsDebug_list.h"
#line 48

// Der zweite Schritt - Erstelle das Array

#undef _ADD_PIN_2
#undef _ADD_PIN
#undef REPORT_NAME_DIGITAL
#undef REPORT_NAME_ANALOG

#define _ADD_PIN_2(ENTRY_NAME, NAME, IS_DIGITAL) { ENTRY_NAME, NAME, IS_DIGITAL },
#define _ADD_PIN(NAME, COUNTER, IS_DIGITAL) _ADD_PIN_2(entry_NAME_##COUNTER, NAME, IS_DIGITAL)
#define REPORT_NAME_DIGITAL(COUNTER, NAME) _ADD_PIN(NAME, COUNTER, true)
#define REPORT_NAME_ANALOG(COUNTER, NAME) _ADD_PIN(analogInputToDigitalPin(NAME), COUNTER, false)


typedef struct {
  PGM_P const name;
  pin_t pin;
  bool is_digital;
} PinInfo;

const PinInfo pin_array[] PROGMEM = {

  /**
   *  [Pin-Name]  [Pin-Number]  [digital oder analog?]  1 = digital, 0 = analog
   *  Jeder Eintrag benötigt 6 bytes im FLASH:
   *     2 byte pointer to location of the name string
   *     2 bytes containing the pin number
   *         analog pin numbers were converted to digital when the array was created
   *     2 bytes containing the digital/analog bool flag
   */

  #if CONF_SERIAL_IS(0)
    #if EITHER(AVR_ATmega2560_FAMILY, ARDUINO_ARCH_SAM)
      { RXD_NAME_0, 0, true },
      { TXD_NAME_0, 1, true },
    #elif AVR_ATmega1284_FAMILY
      { RXD_NAME_0, 8, true },
      { TXD_NAME_0, 9, true },
    #elif defined(TARGET_LPC1768)           // TX P0_02  RX P0_03
      { RXD_NAME_0, 3, true },
      { TXD_NAME_0, 2, true },
    #endif
  #endif

  #if CONF_SERIAL_IS(1)
    #if EITHER(AVR_ATmega2560_FAMILY, ARDUINO_ARCH_SAM)
      { RXD_NAME_1, 19, true },
      { TXD_NAME_1, 18, true },
    #elif AVR_ATmega1284_FAMILY
      { RXD_NAME_1, 10, true },
      { TXD_NAME_1, 11, true },
    #elif defined(TARGET_LPC1768)
      #ifdef LPC_PINCFG_UART1_P2_00         // TX P2_00  RX P2_01
        { RXD_NAME_1, 0x41, true },
        { TXD_NAME_1, 0x40, true },
      #else                                 // TX P0_15  RX P0_16
        { RXD_NAME_1, 16, true },
        { TXD_NAME_1, 15, true },
      #endif
    #endif
  #endif

  #if CONF_SERIAL_IS(2)
    #if EITHER(AVR_ATmega2560_FAMILY, ARDUINO_ARCH_SAM)
      { RXD_NAME_2, 17, true },
      { TXD_NAME_2, 16, true },
    #elif defined(TARGET_LPC1768)
      #ifdef LPC_PINCFG_UART2_P2_08         // TX P2_08  RX P2_09
        { RXD_NAME_2, 0x49, true },
        { TXD_NAME_2, 0x48, true },
      #else                                 // TX P0_10  RX P0_11
        { RXD_NAME_2, 11, true },
        { TXD_NAME_2, 10, true },
      #endif
    #endif
  #endif

  #if CONF_SERIAL_IS(3)
    #if EITHER(AVR_ATmega2560_FAMILY, ARDUINO_ARCH_SAM)
      { RXD_NAME_3, 15, true },
      { TXD_NAME_3, 14, true },
    #elif defined(TARGET_LPC1768)
      #ifdef LPC_PINCFG_UART3_P0_25         // TX P0_25  RX P0_26
        { RXD_NAME_3, 0x1A, true },
        { TXD_NAME_3, 0x19, true },
      #elif defined(LPC_PINCFG_UART3_P4_28) // TX P4_28  RX P4_29
        { RXD_NAME_3, 0x9D, true },
        { TXD_NAME_3, 0x9C, true },
      #else                                 // TX P0_00  RX P0_01
        { RXD_NAME_3, 1, true },
        { TXD_NAME_3, 0, true },
      #endif
    #endif
  #endif

  #include "pinsDebug_list.h"
  #line 167

};

#include HAL_PATH(../HAL, pinsDebug.h)  // get the correct support file for this CPU

#ifndef M43_NEVER_TOUCH
  #define M43_NEVER_TOUCH(Q) false
#endif

static void print_input_or_output(const bool isout) {
  SERIAL_ECHOPGM_P(isout ? PSTR("Output = ") : PSTR("Input  = "));
}

// pretty report with PWM info
inline void report_pin_state_extended(pin_t pin, const bool ignore, const bool extended=false, PGM_P const start_string=nullptr) {
  char buffer[MAX_NAME_LENGTH + 1];   // for the sprintf statements
  bool found = false, multi_name_pin = false;

  auto alt_pin_echo = [](const pin_t &pin) {
    #if AVR_AT90USB1286_FAMILY
      // Use FastIO for pins Teensy doesn't expose
      if (pin == 46) {
        print_input_or_output(IS_OUTPUT(46));
        SERIAL_CHAR('0' + READ(46));
        return false;
      }
      else if (pin == 47) {
        print_input_or_output(IS_OUTPUT(47));
        SERIAL_CHAR('0' + READ(47));
        return false;
      }
    #endif
    return true;
  };

  LOOP_L_N(x, COUNT(pin_array))  {    // scan entire array and report all instances of this pin
    if (GET_ARRAY_PIN(x) == pin) {
      if (!found) {    // report digital and analog pin number only on the first time through
        if (start_string) SERIAL_ECHOPGM_P(start_string);
        SERIAL_ECHOPGM("PIN: ");
        PRINT_PIN(pin);
        PRINT_PORT(pin);
        if (int8_t(DIGITAL_PIN_TO_ANALOG_PIN(pin)) >= 0) PRINT_PIN_ANALOG(pin); // analog pin number
        else SERIAL_ECHO_SP(8);                                                 // add padding if not an analog pin
      }
      else {
        SERIAL_CHAR('.');
        SERIAL_ECHO_SP(MULTI_NAME_PAD + (start_string ? strlen_P(start_string) : 0));  // add padding if not the first instance found
      }
      PRINT_ARRAY_NAME(x);
      if (extended) {
        if (pin_is_protected(pin) && !ignore)
          SERIAL_ECHOPGM("protected ");
        else {
          if (alt_pin_echo(pin)) {
            if (!GET_ARRAY_IS_DIGITAL(x)) {
              sprintf_P(buffer, PSTR("Analog in = %5ld"), (long)analogRead(DIGITAL_PIN_TO_ANALOG_PIN(pin)));
              SERIAL_ECHO(buffer);
            }
            else {
              if (!GET_PINMODE(pin)) {
                //pinMode(pin, INPUT_PULLUP);  // make sure input isn't floating - stopped doing this
                                               // because this could interfere with inductive/capacitive
                                               // sensors (high impedance voltage divider) and with Pt100 amplifier
                print_input_or_output(false);
                SERIAL_ECHO(digitalRead_mod(pin));
              }
              else if (pwm_status(pin)) {
                // do nothing
              }
              else {
                print_input_or_output(true);
                SERIAL_ECHO(digitalRead_mod(pin));
              }
            }
            if (!multi_name_pin && extended) pwm_details(pin);  // report PWM capabilities only on the first pass & only if doing an extended report
          }
        }
      }
      SERIAL_EOL();
      multi_name_pin = found;
      found = true;
    }  // end of IF
  } // end of for loop

  if (!found) {
    if (start_string) SERIAL_ECHOPGM_P(start_string);
    SERIAL_ECHOPGM("PIN: ");
    PRINT_PIN(pin);
    PRINT_PORT(pin);
    if (int8_t(DIGITAL_PIN_TO_ANALOG_PIN(pin)) >= 0) PRINT_PIN_ANALOG(pin); // analog pin number
    else SERIAL_ECHO_SP(8);                                                 // add padding if not an analog pin
    SERIAL_ECHOPGM("<unused/unknown>");
    if (extended) {

      if (alt_pin_echo(pin)) {
        if (pwm_status(pin)) {
          // do nothing
        }
        else if (GET_PINMODE(pin)) {
          SERIAL_ECHO_SP(MAX_NAME_LENGTH - 16);
          print_input_or_output(true);
          SERIAL_ECHO(digitalRead_mod(pin));
        }
        else {
          if (IS_ANALOG(pin)) {
            sprintf_P(buffer, PSTR("   Analog in = %5ld"), (long)analogRead(DIGITAL_PIN_TO_ANALOG_PIN(pin)));
            SERIAL_ECHO(buffer);
            SERIAL_ECHOPGM("   ");
          }
          else
          SERIAL_ECHO_SP(MAX_NAME_LENGTH - 16);   // add padding if not an analog pin

          print_input_or_output(false);
          SERIAL_ECHO(digitalRead_mod(pin));
        }
        //if (!pwm_status(pin)) SERIAL_CHAR(' ');    // add padding if it's not a PWM pin
        if (extended) {
          SERIAL_ECHO_SP(MAX_NAME_LENGTH - 16);
          pwm_details(pin);  // report PWM capabilities only if doing an extended report
        }
      }
    }
    SERIAL_EOL();
  }
}
