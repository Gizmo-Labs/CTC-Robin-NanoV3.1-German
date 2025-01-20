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
#pragma once

//===========================================================================
//========================= Allgemeine Parameter ============================
//===========================================================================

#define CONFIGURATION_H_VERSION 02000902
#define STRING_CONFIG_H_AUTHOR "Gizmo Labs" // Who made the changes.

/**
 * Anzeige von bmp_logo.bin nach dem Einschalten des Druckers
 */
#define SHOW_BOOTSCREEN

/**
 * Ports für serielle Schnittstellen
 * 
 * :[-1, 0, 1, 2, 3, 4, 5, 6, 7]
 */
#define SERIAL_PORT -1

/**
 * Baud-Rate für serielle Schnittstellen
 * 
 * :[2400, 9600, 19200, 38400, 57600, 115200, 250000, 500000, 1000000]
 */
#define BAUDRATE 115200

/** 
 * Name des Boards, ist Bestandteil des Build-Vorgans
 * 
 */
#ifndef MOTHERBOARD
  #define MOTHERBOARD BOARD_MKS_ROBIN_NANO_V3
#endif

//===========================================================================
//============================= Extruder Parameter ==========================
//===========================================================================

/** Anzahl der Extruder
 * 
 */ 
#define EXTRUDERS 1

/** Durchmesser des Filaments
 * 
 */
#define DEFAULT_NOMINAL_FILAMENT_DIA 1.75

//===========================================================================
//=========================== Thermische Parameter ==========================
//===========================================================================

/** 
 *  An den Anschlüssen vom Robin Nano V3.x können 100kΩ Widerstände direkt 
 *  angeschlossen werden. Es sind keine zusätzlichen Widerstände notwendig.
 *  
 *  Thermoelemente können aufgrund fehlender Hardware nicht verwendet werden.
 *  Im CTC-Drucker, für den diese Firmware optimiert wurde, ist als Hot-End Sensor
 *  ein M3 Einschraubwiderstand, vom Typ ATC Semitec 104GT-2 verwendet worden. 
 *  Dieser ist bei "www.cr-3d.de" erhältlich.
 * 
 *  Das Heizbett besteht aus einer 6mmm Aluminiumplatte, mit Glasplatte darauf.
 *  Deshalb wurde Typ 30 gewählt, weil das Aluminium die Hitze lange hält.
 *  Beide Einstellungen funktionieren perfekt.
 * 
 *     1 : 100kΩ  EPCOS - Best choice for EPCOS thermistors
 *   331 : 100kΩ  Same as #1, but 3.3V scaled for MEGA
 *   332 : 100kΩ  Same as #1, but 3.3V scaled for DUE
 *     2 : 200kΩ  ATC Semitec 204GT-2
 *   202 : 200kΩ  Copymaster 3D
 *     3 : ???Ω   Mendel-parts thermistor
 *     4 : 10kΩ   Generic Thermistor !! DO NOT use for a hotend - it gives bad resolution at high temp. !!
 *     5 : 100kΩ  ATC Semitec 104GT-2/104NT-4-R025H42G - Used in ParCan, J-Head, and E3D, SliceEngineering 300°C
 *   501 : 100kΩ  Zonestar - Tronxy X3A
 *   502 : 100kΩ  Zonestar - used by hot bed in Zonestar Průša P802M
 *   512 : 100kΩ  RPW-Ultra hotend
 *     6 : 100kΩ  EPCOS - Not as accurate as table #1 (created using a fluke thermocouple)
 *     7 : 100kΩ  Honeywell 135-104LAG-J01
 *    71 : 100kΩ  Honeywell 135-104LAF-J01
 *     8 : 100kΩ  Vishay 0603 SMD NTCS0603E3104FXT
 *     9 : 100kΩ  GE Sensing AL03006-58.2K-97-G1
 *    10 : 100kΩ  RS PRO 198-961
 *    11 : 100kΩ  Keenovo AC silicone mats, most Wanhao i3 machines - beta 3950, 1%
 *    12 : 100kΩ  Vishay 0603 SMD NTCS0603E3104FXT (#8) - calibrated for Makibox hot bed
 *    13 : 100kΩ  Hisens up to 300°C - for "Simple ONE" & "All In ONE" hotend - beta 3950, 1%
 *    15 : 100kΩ  Calibrated for JGAurora A5 hotend
 *    18 : 200kΩ  ATC Semitec 204GT-2 Dagoma.Fr - MKS_Base_DKU001327
 *    22 : 100kΩ  GTM32 Pro vB - hotend - 4.7kΩ pullup to 3.3V and 220Ω to analog input
 *    23 : 100kΩ  GTM32 Pro vB - bed - 4.7kΩ pullup to 3.3v and 220Ω to analog input
 *    30 : 100kΩ  Kis3d Silicone heating mat 200W/300W with 6mm precision cast plate (EN AW 5083) NTC100K - beta 3950
 *    60 : 100kΩ  Maker's Tool Works Kapton Bed Thermistor - beta 3950
 *    61 : 100kΩ  Formbot/Vivedino 350°C Thermistor - beta 3950
 *    66 : 4.7MΩ  Dyze Design High Temperature Thermistor
 *    67 : 500kΩ  SliceEngineering 450°C Thermistor
 *    70 : 100kΩ  bq Hephestos 2
 *    75 : 100kΩ  Generic Silicon Heat Pad with NTC100K MGB18-104F39050L32
 *  2000 : 100kΩ  Ultimachine Rambo TDK NTCG104LH104KT1 NTC100K motherboard Thermistor
 */
#define TEMP_SENSOR_0 5
#define TEMP_SENSOR_1 0
#define TEMP_SENSOR_2 0
#define TEMP_SENSOR_3 0
#define TEMP_SENSOR_4 0
#define TEMP_SENSOR_5 0
#define TEMP_SENSOR_6 0
#define TEMP_SENSOR_7 0
#define TEMP_SENSOR_BED 30
#define TEMP_SENSOR_PROBE 0
#define TEMP_SENSOR_CHAMBER 0
#define TEMP_SENSOR_COOLER 0
#define TEMP_SENSOR_BOARD 0
#define TEMP_SENSOR_REDUNDANT 0

/** Dummy-Werte für 998 und 999
 * Genaue Bedeutung unklar, nicht relevant
 */
#define DUMMY_THERMISTOR_998_VALUE  25
#define DUMMY_THERMISTOR_999_VALUE 100

#define TEMP_RESIDENCY_TIME         10  // (seconds) Time to wait for hotend to "settle" in M109
#define TEMP_WINDOW                  1  // (°C) Temperature proximity for the "temperature reached" timer
#define TEMP_HYSTERESIS              3  // (°C) Temperature proximity considered "close enough" to the target

#define TEMP_BED_RESIDENCY_TIME     10  // (seconds) Time to wait for bed to "settle" in M190
#define TEMP_BED_WINDOW              1  // (°C) Temperature proximity for the "temperature reached" timer
#define TEMP_BED_HYSTERESIS          3  // (°C) Temperature proximity considered "close enough" to the target

#define TEMP_CHAMBER_RESIDENCY_TIME 10  // (seconds) Time to wait for chamber to "settle" in M191
#define TEMP_CHAMBER_WINDOW          1  // (°C) Temperature proximity for the "temperature reached" timer
#define TEMP_CHAMBER_HYSTERESIS      3  // (°C) Temperature proximity considered "close enough" to the target

/**
 * Minimal erlaubte Temperaturen
 * Bei diesen Werten wird jegliches
 * Heizen abgeschaltet, da davon 
 * ausgegangen wird, dass ein Fühler
 * defekt ist
 */
#define HEATER_0_MINTEMP   5
#define HEATER_1_MINTEMP   5
#define HEATER_2_MINTEMP   5
#define HEATER_3_MINTEMP   5
#define HEATER_4_MINTEMP   5
#define HEATER_5_MINTEMP   5
#define HEATER_6_MINTEMP   5
#define HEATER_7_MINTEMP   5
#define BED_MINTEMP        5
#define CHAMBER_MINTEMP    5

/**
 * Maximal erlaubte Temperaturen
 * Bei diesen Werten wird jegliches
 * Heizen abgeschaltet, da davon ausgegangen wird,
 * dass eine Fehlfunktion/Sensorfehler vorliegt.
 */
#define HEATER_0_MAXTEMP 275
#define HEATER_1_MAXTEMP 275
#define HEATER_2_MAXTEMP 275
#define HEATER_3_MAXTEMP 275
#define HEATER_4_MAXTEMP 275
#define HEATER_5_MAXTEMP 275
#define HEATER_6_MAXTEMP 275
#define HEATER_7_MAXTEMP 275
#define BED_MAXTEMP      150
#define CHAMBER_MAXTEMP  60

/**
 * Thermal Overshoot
 * Vereinfacht ausgedrückt ist der jeweilige Wert das DELTA zu MAXTEMP
 * Es können nur Temperaturen genutzt werden, die weit genug weg sind, 
 * vom MAXTEMP Parameter.
 */
#define HOTEND_OVERSHOOT 15   // (°C) Forbid temperatures over MAXTEMP - OVERSHOOT
#define BED_OVERSHOOT    10   // (°C) Forbid temperatures over MAXTEMP - OVERSHOOT
#define COOLER_OVERSHOOT  2   // (°C) Forbid temperatures closer than OVERSHOOT

//===========================================================================
//============================= PID Parameter ===============================
//===========================================================================

/** 
 * PID-Autotune hat beim ausprobieren nicht funktioniert. 
 * Weiterhin ist zu beachten, dass in der MKS-UI kein Menü dafür existiert.
 * Die Funktion ist auch nicht nötig, alles funktioniert auch so wunderbar.
 * Es wurde nichts an den Parameter verändert.
 */
#define PIDTEMP
#define BANG_MAX 255            // Limits current to nozzle while in bang-bang mode; 255=full current
#define PID_MAX BANG_MAX        // Limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current
#define PID_K1 0.95             // Smoothing factor within any PID loop
#define PID_FUNCTIONAL_RANGE 10 // If the temperature difference between the target temperature and the actual temperature
                                // is more than PID_FUNCTIONAL_RANGE then the PID will be shut off and the heater will be set to min/max.

#if ENABLED(PIDTEMP)  
  #if ENABLED(PID_PARAMS_PER_HOTEND) 
    #define DEFAULT_Kp_LIST {  22.20,  22.20 }
    #define DEFAULT_Ki_LIST {   1.08,   1.08 }
    #define DEFAULT_Kd_LIST { 114.00, 114.00 }
  #else
    #define DEFAULT_Kp  22.20
    #define DEFAULT_Ki   1.08
    #define DEFAULT_Kd 114.00
  #endif
#endif

//===========================================================================
//====================== Druckbett-Temperaturregelung =======================
//===========================================================================

/**
 * Maximal erlaubte Heizleistung
 * Legt den max. Duty-Cycle vom PWM fest.
 * Keine Änderung nötig.
 */
#define MAX_BED_POWER 255 // limits duty cycle to bed; 255=full current


//===========================================================================
//========================== Extruder-Parameter =============================
//===========================================================================

/**
 * Verhindert Extrusion bei zu niedrigen Temperaturen
 * Mit M-Befehl M302 kann Temperatur eingestellt werden, und die Blockierung
 * aktiviert/deaktviert werden.
 * Unterhalb dieser Temperatur erfolgt keine Extrusion, um den Extruder
 * vor Schäden zu bewahren.
 *
 */
#define PREVENT_COLD_EXTRUSION
#define EXTRUDE_MINTEMP 220

/**
 * Maximal erlaubte Länge pro Extrusionsvorgang
 */
#define PREVENT_LENGTHY_EXTRUDE
#define EXTRUDE_MAXLENGTH 200

//===========================================================================
//====================== Schutz vor thermischem Abhauen =====================
//===========================================================================

/**
 * Akiviert den Schutz vor thermischem Abhauen
 */
#define THERMAL_PROTECTION_HOTENDS // Enable thermal protection for all extruders
#define THERMAL_PROTECTION_BED     // Enable thermal protection for the heated bed

//===========================================================================
//========================= Endschalter Parameter ===========================
//===========================================================================

/** 
 * Die verwendeten Steckplätze der Endschalter
 */
#define USE_ZMIN_PLUG
#define USE_XMAX_PLUG
#define USE_YMAX_PLUG

/** 
 * Aktiviere Pullup-Widerstände für die Endschalter
 */ 
#define ENDSTOPPULLUPS

// Mechanical endstop with COM to ground and NC to Signal uses "false" here (most common setup).
#define X_MIN_ENDSTOP_INVERTING false  // Set to true to invert the logic of the endstop.
#define Y_MIN_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.
#define Z_MIN_ENDSTOP_INVERTING false  // Set to true to invert the logic of the endstop.
#define I_MIN_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.
#define J_MIN_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.
#define K_MIN_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.
#define X_MAX_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.
#define Y_MAX_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.
#define Z_MAX_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.
#define I_MAX_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.
#define J_MAX_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.
#define K_MAX_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.
#define Z_MIN_PROBE_ENDSTOP_INVERTING false  // Set to true to invert the logic of the probe.

/**
 * Es werden Watteroth Silent Step-Sticks auf allen Achsen benutzt
 */
#define X_DRIVER_TYPE  TMC2208
#define Y_DRIVER_TYPE  TMC2208
#define Z_DRIVER_TYPE  TMC2208
#define E0_DRIVER_TYPE TMC2208

/**
 * Invertieren der Schrittmotor Enable-Pins
 */
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0

/**
 * Schrittmotoren sofort abschalten, nach Bewegungsende
 * Nicht zu empfehlen, wegen möglichem Positionsverlust
 */
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z false

/**
 * Invertieren der Schrittmotor-Drehrichtung 
 */
#define INVERT_X_DIR false
#define INVERT_Y_DIR false
#define INVERT_Z_DIR false

//=============================================================================
//============================= Bewegungs-Parameter ===========================
//=============================================================================

/**
 * Standwerte für (Schritte/pro mm)
 * Können mit M92 angepasst werden
 *                                      X, Y, Z, E0
 */
#define DEFAULT_AXIS_STEPS_PER_UNIT   { 80, 80, 400, 96 }

/**
 * Standardwert für maximale Vorschubgeschwindigkeit (mm/s)
 * Können mit M203 angepasst werden
 *                                      X, Y, Z, E0
 */
#define DEFAULT_MAX_FEEDRATE          { 200, 200, 4, 100 }

/**
 * Standardwerte für maximale Beschleunigung 
 * Können mit M201 angepasst werden
 *                                      X, Y, Z, E0
 */
#define DEFAULT_MAX_ACCELERATION      { 750, 750, 200, 2000 }


/**
 * Standardwerte für Beschleunigung 
 * Können mit M204 angepasst werden
 *
 *   M204 P    Acceleration
 *   M204 R    Retract Acceleration
 *   M204 T    Travel Acceleration
 */
#define DEFAULT_ACCELERATION          1000    // X, Y, Z and E acceleration for printing moves
#define DEFAULT_RETRACT_ACCELERATION  1000    // E acceleration for retracts
#define DEFAULT_TRAVEL_ACCELERATION   1000    // X, Y, Z acceleration for travel (non printing) moves

/**
 * Standard Jerk limits (mm/s)
 * Können mit M205 X Y Z E angepasst werden
 *
 * "Jerk" specifies the minimum speed change that requires acceleration.
 * When changing speed and direction, if the difference is less than the
 * value set here, it may happen instantaneously.
 */
#define CLASSIC_JERK
#if ENABLED(CLASSIC_JERK)
  #define DEFAULT_XJERK 13.0
  #define DEFAULT_YJERK 13.0
  #define DEFAULT_ZJERK  0.3
  
  #if ENABLED(LIMITED_JERK_EDITING)
    #define MAX_JERK_EDIT_VALUES { 20, 20, 0.6, 10 } // ...or, set your own edit limits
  #endif
#endif

#define DEFAULT_EJERK    5.0  // May be used by Linear Advance

/**
 * Junction Deviation Factor
 *
 * See:
 *   https://reprap.org/forum/read.php?1,739819
 *   https://blog4kyneticcnc.com/2018/10/computing-junction-deviation-for-marlin.html
 */
#if DISABLED(CLASSIC_JERK)
  #define JUNCTION_DEVIATION_MM 0.013 // (mm) Distance from real junction edge
  #define JD_HANDLE_SMALL_SEGMENTS    // Use curvature estimation instead of just the junction angle
                                      // for small segments (< 1mm) with large junction angles (> 135°).
#endif

/**
 * S-Curve Acceleration
 *
 * This option eliminates vibration during printing by fitting a Bézier
 * curve to move acceleration, producing much smoother direction changes.
 *
 * See https://github.com/synthetos/TinyG/wiki/Jerk-Controlled-Motion-Explained
 */
#define S_CURVE_ACCELERATION

//===========================================================================
//=========================== Druckbett-Kalibrierung ========================
//===========================================================================

/**
 * Das Druckbett wird manuell eingestellt, kein Messtaster, Sensor nix..
 * Vorgang wird über UI-Menü "Ausrichten" durchgeführt.
 */
#define PROBE_MANUALLY

/** 
 * Keine Sensoren, also keine Werte
 */
#define NOZZLE_TO_PROBE_OFFSET { 0, 0, 0 }

/** 
 * Keine Sensoren, also keine Werte
 */
#define PROBING_MARGIN 0

/** 
 * Geschwindigkeit zwischen den Messpunkten
 */
#define XY_PROBE_FEEDRATE (133*60)

/** 
 * Nur relevant mit Sensorik
 */
#define Z_PROBE_FEEDRATE_FAST (4*60)

/** 
 * Nur relevant mit Sensorik
 */
#define Z_PROBE_FEEDRATE_SLOW (Z_PROBE_FEEDRATE_FAST / 2)

/** 
 * Nicht genauer untersucht, nicht notwendig, nicht verändert.
 */
#define Z_CLEARANCE_DEPLOY_PROBE   10 // Z Clearance for Deploy/Stow
#define Z_CLEARANCE_BETWEEN_PROBES  5 // Z Clearance between probe points
#define Z_CLEARANCE_MULTI_PROBE     5 // Z Clearance between multiple probes
#define Z_PROBE_LOW_POINT          -2 // Farthest distance below the trigger-point to go before stopping

// For M851 give a range for adjusting the Z probe offset
#define Z_PROBE_OFFSET_RANGE_MIN -20
#define Z_PROBE_OFFSET_RANGE_MAX 20

// Require minimum nozzle and/or bed temperature for probing
#define PREHEAT_BEFORE_PROBING
#if ENABLED(PREHEAT_BEFORE_PROBING)
  #define PROBING_NOZZLE_TEMP 120   // (°C) Only applies to E0 at this time
  #define PROBING_BED_TEMP     50
#endif

//=============================================================================
//============================= Extruder-Parameter ============================
//=============================================================================

#define DISABLE_E false             // Disable the extruder when not stepping
#define DISABLE_INACTIVE_EXTRUDER   // Keep only the active extruder enabled

/**
 * Invertieren der Extruder-Drehrichtung 
 */
#define INVERT_E0_DIR false

//=============================================================================
//============================= Homing-Parameter ============================
//=============================================================================

/**
 * Anfahrrichtung der Endschalter
 * 1 = In Richtung des MAX-Endschalters
 * 0 = In Richtung des MIN-Endschalters
 */
#define X_HOME_DIR 1
#define Y_HOME_DIR 1
#define Z_HOME_DIR -1

/**
 * Abmessungen des Druckbetts
 */
#define X_BED_SIZE 255
#define Y_BED_SIZE 155

//=============================================================================
//======================= Software-Endschalter-Parameter ======================
//=============================================================================

/**
 * - Auch als Maschinengrenzen bekannt.
 * - Siehe M-Befehl M211...
 */

/**
 * Limits für Software-Endschalter
 * Es wird nicht weiter verfahren, als die hier 
 * eingetragenen Werte. Gilt IMMER.
 */
#define X_MIN_POS 0
#define Y_MIN_POS 0
#define Z_MIN_POS 0
#define X_MAX_POS X_BED_SIZE
#define Y_MAX_POS Y_BED_SIZE
#define Z_MAX_POS 145

/**
 * Software Endschalter für die MINIMUM-Richtung
 */
#define MIN_SOFTWARE_ENDSTOPS
#if ENABLED(MIN_SOFTWARE_ENDSTOPS)
  #define MIN_SOFTWARE_ENDSTOP_X
  #define MIN_SOFTWARE_ENDSTOP_Y
  #define MIN_SOFTWARE_ENDSTOP_Z
#endif

/**
 * Software Endschalter für die MAXIMUM-Richtung
 */
#define MAX_SOFTWARE_ENDSTOPS
#if ENABLED(MAX_SOFTWARE_ENDSTOPS)
  #define MAX_SOFTWARE_ENDSTOP_X
  #define MAX_SOFTWARE_ENDSTOP_Y
  #define MAX_SOFTWARE_ENDSTOP_Z 
#endif

//=============================================================================
//============================= Filament-Erkennung ============================
//=============================================================================

/**
 * Filament Runout Sensors
 * Mechanical or opto endstops are used to check for the presence of filament.
 *
 * IMPORTANT: Runout will only trigger if Marlin is aware that a print job is running.
 * Marlin knows a print job is running when:
 *  1. Running a print job from media started with M24.
 *  2. The Print Job Timer has been started with M75.
 *  3. The heaters were turned on and PRINTJOB_TIMER_AUTOSTART is enabled.
 *
 * RAMPS-based boards use SERVO3_PIN for the first runout sensor.
 * For other boards you may need to define FIL_RUNOUT_PIN, FIL_RUNOUT2_PIN, etc.
 */
//#define FILAMENT_RUNOUT_SENSOR
#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  #define FIL_RUNOUT_ENABLED_DEFAULT true // Enable the sensor on startup. Override with M412 followed by M500.
  #define NUM_RUNOUT_SENSORS   1          // Number of sensors, up to one per extruder. Define a FIL_RUNOUT#_PIN for each.

  #define FIL_RUNOUT_STATE     LOW        // Pin state indicating that filament is NOT present.
  #define FIL_RUNOUT_PULLUP               // Use internal pullup for filament runout pins.
  //#define FIL_RUNOUT_PULLDOWN           // Use internal pulldown for filament runout pins.
  //#define WATCH_ALL_RUNOUT_SENSORS      // Execute runout script on any triggering sensor, not only for the active extruder.
                                          // This is automatically enabled for MIXING_EXTRUDERs.

  // Override individually if the runout sensors vary
  //#define FIL_RUNOUT1_STATE LOW
  //#define FIL_RUNOUT1_PULLUP
  //#define FIL_RUNOUT1_PULLDOWN

  //#define FIL_RUNOUT2_STATE LOW
  //#define FIL_RUNOUT2_PULLUP
  //#define FIL_RUNOUT2_PULLDOWN

  //#define FIL_RUNOUT3_STATE LOW
  //#define FIL_RUNOUT3_PULLUP
  //#define FIL_RUNOUT3_PULLDOWN

  //#define FIL_RUNOUT4_STATE LOW
  //#define FIL_RUNOUT4_PULLUP
  //#define FIL_RUNOUT4_PULLDOWN

  //#define FIL_RUNOUT5_STATE LOW
  //#define FIL_RUNOUT5_PULLUP
  //#define FIL_RUNOUT5_PULLDOWN

  //#define FIL_RUNOUT6_STATE LOW
  //#define FIL_RUNOUT6_PULLUP
  //#define FIL_RUNOUT6_PULLDOWN

  //#define FIL_RUNOUT7_STATE LOW
  //#define FIL_RUNOUT7_PULLUP
  //#define FIL_RUNOUT7_PULLDOWN

  //#define FIL_RUNOUT8_STATE LOW
  //#define FIL_RUNOUT8_PULLUP
  //#define FIL_RUNOUT8_PULLDOWN

  // Commands to execute on filament runout.
  // With multiple runout sensors use the %c placeholder for the current tool in commands (e.g., "M600 T%c")
  // NOTE: After 'M412 H1' the host handles filament runout and this script does not apply.
  #define FILAMENT_RUNOUT_SCRIPT "M600"

  // After a runout is detected, continue printing this length of filament
  // before executing the runout script. Useful for a sensor at the end of
  // a feed tube. Requires 4 bytes SRAM per sensor, plus 4 bytes overhead.
  //#define FILAMENT_RUNOUT_DISTANCE_MM 25

  #ifdef FILAMENT_RUNOUT_DISTANCE_MM
    // Enable this option to use an encoder disc that toggles the runout pin
    // as the filament moves. (Be sure to set FILAMENT_RUNOUT_DISTANCE_MM
    // large enough to avoid false positives.)
    //#define FILAMENT_MOTION_SENSOR
  #endif
#endif

//=============================================================================
//========================== Bett-Ausrichten-Parameter ========================
//=============================================================================

#define LEVEL_BED_CORNERS

#if ENABLED(LEVEL_BED_CORNERS)
  #define LEVEL_CORNERS_INSET_LFRB { 30, 30, 30, 30 } // (mm) Left, Front, Right, Back insets
  #define LEVEL_CORNERS_HEIGHT      0.0   // (mm) Z height of nozzle at leveling points
  #define LEVEL_CORNERS_Z_HOP       4.0   // (mm) Z height of nozzle between leveling points
  #define LEVEL_CENTER_TOO              // Move to the center after the last corner
  //#define LEVEL_CORNERS_USE_PROBE
  #if ENABLED(LEVEL_CORNERS_USE_PROBE)
    #define LEVEL_CORNERS_PROBE_TOLERANCE 0.1
    #define LEVEL_CORNERS_VERIFY_RAISED   // After adjustment triggers the probe, re-probe to verify
    //#define LEVEL_CORNERS_AUDIO_FEEDBACK
  #endif

  /**
   * Corner Leveling Order
   *
   * Set 2 or 4 points. When 2 points are given, the 3rd is the center of the opposite edge.
   *
   *  LF  Left-Front    RF  Right-Front
   *  LB  Left-Back     RB  Right-Back
   *
   * Examples:
   *
   *      Default        {LF,RB,LB,RF}         {LF,RF}           {LB,LF}
   *  LB --------- RB   LB --------- RB    LB --------- RB   LB --------- RB
   *  |  4       3  |   | 3         2 |    |     <3>     |   | 1           |
   *  |             |   |             |    |             |   |          <3>|
   *  |  1       2  |   | 1         4 |    | 1         2 |   | 2           |
   *  LF --------- RF   LF --------- RF    LF --------- RF   LF --------- RF
   */
  #define LEVEL_CORNERS_LEVELING_ORDER { LF, RF, RB, LB }
#endif

/**
 * Commands to execute at the end of G29 probing.
 * Useful to retract or move the Z probe out of the way.
 */
//#define Z_PROBE_END_SCRIPT "G1 Z10 F12000\nG1 X15 Y330\nG1 Z0.5\nG1 Z10"

// Homing speeds (mm/min)
#define HOMING_FEEDRATE_MM_M { (50*60), (50*60), (4*60) }

// Validate that endstops are triggered on homing moves
#define VALIDATE_HOMING_ENDSTOPS

//=============================================================================
//============================ Verschiedene Parameter =========================
//=============================================================================

/**
 * EEPROM
 *
 * Persistent storage to preserve configurable settings across reboots.
 *
 *   M500 - Store settings to EEPROM.
 *   M501 - Read settings from EEPROM. (i.e., Throw away unsaved changes)
 *   M502 - Revert settings to "factory" defaults. (Follow with M500 to init the EEPROM.)
 */
#define EEPROM_SETTINGS     // Persistent storage with M500 and M501
//#define DISABLE_M503        // Saves ~2700 bytes of PROGMEM. Disable for release!
#define EEPROM_CHITCHAT       // Give feedback on EEPROM commands. Disable to save PROGMEM.
#define EEPROM_BOOT_SILENT    // Keep M503 quiet and only give errors during first load
#if ENABLED(EEPROM_SETTINGS)
  #define EEPROM_AUTO_INIT  // Init EEPROM automatically on any errors.
#endif

//
// Host Keepalive
//
// When enabled Marlin will send a busy status message to the host
// every couple of seconds when it can't accept commands.
//
#define HOST_KEEPALIVE_FEATURE        // Disable this if your host doesn't like keepalive messages
#define DEFAULT_KEEPALIVE_INTERVAL 2  // Number of seconds between "busy" messages. Set with M113.
#define BUSY_WHILE_HEATING            // Some hosts require "busy" messages even during heating

//=============================================================================
//========================== Vordefinierte Materialien ========================
//=============================================================================


#define PREHEAT_1_LABEL       "ASA"
#define PREHEAT_1_TEMP_HOTEND 260
#define PREHEAT_1_TEMP_BED    100
#define PREHEAT_1_FAN_SPEED     0 // Value from 0 to 255

#define PREHEAT_2_LABEL       "ABS"
#define PREHEAT_2_TEMP_HOTEND 240
#define PREHEAT_2_TEMP_BED    110
#define PREHEAT_2_FAN_SPEED     0 // Value from 0 to 255

//=============================================================================
//============================= Park-Postition-Parameter ======================
//=============================================================================

/**
 * Nozzle Park
 *
 * Park the nozzle at the given XYZ position on idle or G27.
 *
 * The "P" parameter controls the action applied to the Z axis:
 *
 *    P0  (Default) If Z is below park Z raise the nozzle.
 *    P1  Raise the nozzle always to Z-park height.
 *    P2  Raise the nozzle by Z-park amount, limited to Z_MAX_POS.
 */
#define NOZZLE_PARK_FEATURE

#if ENABLED(NOZZLE_PARK_FEATURE)
  // Specify a park position as { X, Y, Z_raise }
  #define NOZZLE_PARK_POINT { (X_MIN_POS + 10), (Y_MAX_POS - 10), 20 }
  //#define NOZZLE_PARK_X_ONLY          // X move only is required to park
  //#define NOZZLE_PARK_Y_ONLY          // Y move only is required to park
  #define NOZZLE_PARK_Z_RAISE_MIN   2   // (mm) Always raise Z by at least this distance
  #define NOZZLE_PARK_XY_FEEDRATE 100   // (mm/s) X and Y axes feedrate (also used for delta Z axis)
  #define NOZZLE_PARK_Z_FEEDRATE    5   // (mm/s) Z axis feedrate (not used for delta printers)
#endif

//=============================================================================
//======================== Druckzeit-Erfassung-Parameter ======================
//=============================================================================

/**
 * Print Job Timer
 *
 * Automatically start and stop the print job timer on M104/M109/M140/M190/M141/M191.
 * The print job timer will only be stopped if the bed/chamber target temp is
 * below BED_MINTEMP/CHAMBER_MINTEMP.
 *
 *   M104 (hotend, no wait)  - high temp = none,        low temp = stop timer
 *   M109 (hotend, wait)     - high temp = start timer, low temp = stop timer
 *   M140 (bed, no wait)     - high temp = none,        low temp = stop timer
 *   M190 (bed, wait)        - high temp = start timer, low temp = none
 *   M141 (chamber, no wait) - high temp = none,        low temp = stop timer
 *   M191 (chamber, wait)    - high temp = start timer, low temp = none
 *
 * For M104/M109, high temp is anything over EXTRUDE_MINTEMP / 2.
 * For M140/M190, high temp is anything over BED_MINTEMP.
 * For M141/M191, high temp is anything over CHAMBER_MINTEMP.
 *
 * The timer can also be controlled with the following commands:
 *
 *   M75 - Start the print job timer
 *   M76 - Pause the print job timer
 *   M77 - Stop the print job timer
 */
#define PRINTJOB_TIMER_AUTOSTART

/**
 * SD CARD
 *
 * SD Card support is disabled by default. If your controller has an SD slot,
 * you must uncomment the following option or it won't work.
 */
#define SDSUPPORT

//=============================================================================
//=============================== Graphical TFTs ==============================
//=============================================================================

/**
 * Specific TFT Model Presets. Enable one of the following options
 * or enable TFT_GENERIC and set sub-options.
 */

//
// 480x320, 3.5", SPI Display From MKS
// Normally used in MKS Robin Nano V2
//
#define MKS_TS35_V2_0

/**
 * TFT UI - User Interface Selection. Enable one of the following options:
 *
 *   TFT_CLASSIC_UI - Emulated DOGM - 128x64 Upscaled
 *   TFT_COLOR_UI   - Marlin Default Menus, Touch Friendly, using full TFT capabilities
 *   TFT_LVGL_UI    - A Modern UI using LVGL
 *
 *   For LVGL_UI also copy the 'assets' folder from the build directory to the
 *   root of your SD card, together with the compiled firmware.
 */
#define TFT_LVGL_UI

#if ENABLED(TFT_LVGL_UI)
  #define MKS_WIFI_MODULE  // MKS WiFi module
#endif

//
// Touch Screen Settings
//
#define TOUCH_SCREEN

#if ENABLED(TOUCH_SCREEN)
  #define BUTTON_DELAY_EDIT  50 // (ms) Button repeat delay for edit screens
  #define BUTTON_DELAY_MENU 250 // (ms) Button repeat delay for menus

  #define TOUCH_SCREEN_CALIBRATION

  #if BOTH(TOUCH_SCREEN_CALIBRATION, EEPROM_SETTINGS)
    #define TOUCH_CALIBRATION_AUTO_SAVE // Auto save successful calibration values to EEPROM
  #endif

  #if ENABLED(TFT_COLOR_UI)
    //#define SINGLE_TOUCH_NAVIGATION
  #endif
#endif

//=============================================================================
//=============================== Extra Features ==============================
//=============================================================================

// Use software PWM to drive the fan, as for the heaters. This uses a very low frequency
// which is not as annoying as with the hardware PWM. On the other hand, if this frequency
// is too low, you should also increment SOFT_PWM_SCALE.
#define FAN_SOFT_PWM

// Incrementing this by 1 will double the software PWM frequency,
// affecting heaters, and the fan if FAN_SOFT_PWM is enabled.
// However, control resolution will be halved for each increment;
// at zero value, there are 128 effective control positions.
// :[0,1,2,3,4,5,6,7]
#define SOFT_PWM_SCALE 0

/**
 * RGB LED / LED Strip Control
 *
 * Enable support for an RGB LED connected to 5V digital pins, or
 * an RGB Strip connected to MOSFETs controlled by digital pins.
 *
 * Adds the M150 command to set the LED (or LED strip) color.
 * If pins are PWM capable (e.g., 4, 5, 6, 11) then a range of
 * luminance values can be set from 0 to 255.
 * For NeoPixel LED an overall brightness parameter is also available.
 *
 * *** CAUTION ***
 *  LED Strips require a MOSFET Chip between PWM lines and LEDs,
 *  as the Arduino cannot handle the current the LEDs will require.
 *  Failure to follow this precaution can destroy your Arduino!
 *  NOTE: A separate 5V power supply is required! The NeoPixel LED needs
 *  more current than the Arduino 5V linear regulator can produce.
 * *** CAUTION ***
 *
 * LED Type. Enable only one of the following two options.
 */
//#define RGB_LED
//#define RGBW_LED

#if EITHER(RGB_LED, RGBW_LED)
  //#define RGB_LED_R_PIN 34
  //#define RGB_LED_G_PIN 43
  //#define RGB_LED_B_PIN 35
  //#define RGB_LED_W_PIN -1
#endif

// Support for Adafruit NeoPixel LED driver
//#define NEOPIXEL_LED
#if ENABLED(NEOPIXEL_LED)
  #define NEOPIXEL_TYPE   NEO_GRBW // NEO_GRBW / NEO_GRB - four/three channel driver type (defined in Adafruit_NeoPixel.h)
  //#define NEOPIXEL_PIN     4     // LED driving pin
  //#define NEOPIXEL2_TYPE NEOPIXEL_TYPE
  //#define NEOPIXEL2_PIN    5
  #define NEOPIXEL_PIXELS 30       // Number of LEDs in the strip. (Longest strip when NEOPIXEL2_SEPARATE is disabled.)
  #define NEOPIXEL_IS_SEQUENTIAL   // Sequential display for temperature change - LED by LED. Disable to change all LEDs at once.
  #define NEOPIXEL_BRIGHTNESS 127  // Initial brightness (0-255)
  //#define NEOPIXEL_STARTUP_TEST  // Cycle through colors at startup

  // Support for second Adafruit NeoPixel LED driver controlled with M150 S1 ...
  //#define NEOPIXEL2_SEPARATE
  #if ENABLED(NEOPIXEL2_SEPARATE)
    #define NEOPIXEL2_PIXELS      15  // Number of LEDs in the second strip
    #define NEOPIXEL2_BRIGHTNESS 127  // Initial brightness (0-255)
    #define NEOPIXEL2_STARTUP_TEST    // Cycle through colors at startup
  #else
    //#define NEOPIXEL2_INSERIES      // Default behavior is NeoPixel 2 in parallel
  #endif

  // Use some of the NeoPixel LEDs for static (background) lighting
  //#define NEOPIXEL_BKGD_INDEX_FIRST  0              // Index of the first background LED
  //#define NEOPIXEL_BKGD_INDEX_LAST   5              // Index of the last background LED
  //#define NEOPIXEL_BKGD_COLOR { 255, 255, 255, 0 }  // R, G, B, W
  //#define NEOPIXEL_BKGD_ALWAYS_ON                   // Keep the backlight on when other NeoPixels are off
#endif

/**
 * Printer Event LEDs
 *
 * During printing, the LEDs will reflect the printer status:
 *
 *  - Gradually change from blue to violet as the heated bed gets to target temp
 *  - Gradually change from violet to red as the hotend gets to temperature
 *  - Change to white to illuminate work surface
 *  - Change to green once print has finished
 *  - Turn off after the print has finished and the user has pushed a button
 */
#if ANY(BLINKM, RGB_LED, RGBW_LED, PCA9632, PCA9533, NEOPIXEL_LED)
  #define PRINTER_EVENT_LEDS
#endif
