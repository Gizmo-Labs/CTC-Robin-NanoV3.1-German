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

//********************************************//
#define MACHINE_CONFIG_DE              "Einrichten"

#define NEXT_DE                 "Vor"
#define PREVIOUS_DE             "Vorherige"
#define DEFAULT_DE              "Standard"
#define KEY_BACK_DE             "Entf"
#define KEY_REST_DE             "Reset"
#define KEY_CONFIRM_DE          "OK"

#define KEYBOARD_KEY0_DE "0"
#define KEYBOARD_KEY1_DE "1"
#define KEYBOARD_KEY2_DE "2"
#define KEYBOARD_KEY3_DE "3"
#define KEYBOARD_KEY4_DE "4"
#define KEYBOARD_KEY5_DE "5"
#define KEYBOARD_KEY6_DE "6"
#define KEYBOARD_KEY7_DE "7"
#define KEYBOARD_KEY8_DE "8"
#define KEYBOARD_KEY9_DE "9"
#define KEYBOARD_KEY_POINT_DE "."
#define KEYBOARD_KEY_NEGATIVE_DE "-"

#define MACHINE_PARA_TITLE_DE       "Einstellungen"
#define MACHINE_TYPE_CNOFIG_DE      "Maschinen Einstellungen"
#define MOTOR_CONFIG_DE             "Motor Einstellungen"
#define MACHINE_LEVELING_CONFIG_DE  "Leveling Einstellungen"
#define ADVANCE_CONFIG_DE           "Erweiterte Einstellungen"

#define MACHINE_CONFIG_TITLE_DE     "Maschinen Einstellungen"
#define MACHINE_TYPE_DE             "Maschinentyp"
#define MACHINE_STROKE_DE           "Maschinenmaße"
#define MACHINE_HOMEDIR_DE          "Home Richtung"
#define MACHINE_ENDSTOP_TYPE_DE     "Endschaltertyp"
#define MACHINE_FILAMENT_CONFIG_DE  "Filament Einstellungen"

#define MACHINE_TYPE_CONFIG_TITLE_DE    "Maschinen Einstellungen -> Maschinentyp"
#define MACHINE_TYPE_XYZ_DE             "XYZ Maschine"
#define MACHINE_TYPE_DELTA_DE           "Delta Maschine"
#define MACHINE_TYPE_COREXY_DE          "Corexy Maschine"

#define MACHINE_STROKE_CONF_TITLE_DE    "Maschinen Einstellungen -> Maschinenmaße"
#define X_MAX_LENGTH_DE                 "X-Achse max. Verfahrweg"
#define Y_MAX_LENGTH_DE                 "Y-Achse max. Verfahrweg"
#define Z_MAX_LENGTH_DE                 "Z-Achse max. Verfahrweg"

#define X_MIN_LENGTH_DE                 "X-Achse min. Verfahrweg"
#define Y_MIN_LENGTH_DE                 "Y-Achse min. Verfahrweg"
#define Z_MIN_LENGTH_DE                 "Z-Achse min. Verfahrweg"

#define HOME_DIR_CONF_TITLE_DE          "Maschinen Einstellungen -> Home Richtung"
#define HOME_DIR_X_DE                   "X-Achse Home Richtung"
#define HOME_DIR_Y_DE                   "Y-Achse Home Richtung"
#define HOME_DIR_Z_DE                   "Z-Achse Home Richtung"
#define HOME_MIN_DE                     "MIN"
#define HOME_MAX_DE                     "MAX"

#define ENDSTOP_CONF_TITLE_DE           "Maschinen Einstellungen -> Endschaltertyp"
#define MIN_ENDSTOP_X_DE                "X-axis minimum Endstop"
#define MIN_ENDSTOP_Y_DE                "Y-axis minimum Endstop"
#define MIN_ENDSTOP_Z_DE                "Z-axis minimum Endstop"
#define MAX_ENDSTOP_X_DE                "X-axis maximum Endstop"
#define MAX_ENDSTOP_Y_DE                "Y-axis maximum Endstop"
#define MAX_ENDSTOP_Z_DE                "Z-axis maximum Endstop"
#define ENDSTOP_FIL_DE                  "Filament sensor"
#define ENDSTOP_LEVEL_DE                "Leveling sensor"
#define ENDSTOP_OPENED_DE               "Open"
#define ENDSTOP_CLOSED_DE               "Close"

#define FILAMENT_CONF_TITLE_DE          "Maschinen Einstellungen -> Filament"
#define FILAMENT_IN_LENGTH_DE           "Ladestrecke"
#define FILAMENT_IN_SPEED_DE            "Lade Geschwindigkeit"
#define FILAMENT_TEMPERATURE_DE         "Filament Temperatur"
#define FILAMENT_OUT_LENGTH_DE          "Entladestrecke"
#define FILAMENT_OUT_SPEED_DE           "Entlade Geschwindigkeit"

#define LEVELING_CONF_TITLE_DE          "Maschinen Einstellungen -> Druckbett justieren"
#define LEVELING_PARA_CONF_DE           "Druckbett justieren"
#define TRAMMING_POS_DE                 "Koordinaten Druckbett justieren"
#define LEVELING_AUTO_COMMAND_DE        "Kommando zum Auto justieren"
#define LEVELING_AUTO_ZOFFSET_DE        "Nozzle-to-probe offsets settings"

#define BLTOUCH_LEVELING_TITTLE_DE      "Maschinen Settings -> BL-Touch Probe"
#define BLTOUCH_LEVELING_DE             "BL-Touch Probe"
#define BLTOUCH_INIT_DE                 "Init"
#define BLTOUCH_ZOFFSETPOS_DE           "Zoffset+"
#define BLTOUCH_ZOFFSETNEG_DE           "Zoffset-"
#define BLTOUCH_SAVE_DE                 "Save"
#define BLTOUCH_TEST_DE                 "Test"

#define LEVELING_PARA_CONF_TITLE_DE     "leveling setting"
#define AUTO_LEVELING_ENABLE_DE         "Enable auto leveling"
#define BLTOUCH_LEVELING_ENABLE_DE      "Enable BLTouch"
#define PROBE_PORT_DE                   "Probe connector"
#define PROBE_X_OFFSET_DE               "Probe X-axis offset"
#define PROBE_Y_OFFSET_DE               "Probe Y-axis offset"
#define PROBE_Z_OFFSET_DE               "Probe Z-axis offset"
#define PROBE_XY_SPEED_DE               "Probe XY-axis speed"
#define PROBE_Z_SPEED_DE                "Probe Z-axis speed"
#define ENABLE_DE                       "YES"
#define DISABLE_DE                      "NO"
#define LOCKED_DE                       "N/A"
#define Z_MIN_DE                        "ZMin"
#define Z_MAX_DE                        "ZMax"

#define DELTA_LEVEL_CONF_TITLE_DE       "Delta Machine settings"
#define DELTA_LEVEL_CONF_DE             "Delta Machine Leveling"
#define DELTA_MACHINE_RADIUS_DE         "Machine Radius"
#define DELTA_DIAGONAL_ROD_DE           "Machine rod length"
#define DELTA_PRINT_RADIUS_DE           "Print radius"
#define DELTA_HEIGHT_DE                 "Print height"
#define SMOOTH_ROD_OFFSET_DE            "Slider offset"
#define EFFECTOR_OFFSET_DE              "Effector offset"
#define CALIBRATION_RADIUS_DE           "Leveling radius"

#define XYZ_LEVEL_CONF_TITLE_DE         "Cartesian Machine Settings"
#define PROBE_REACH_MAX_LEFT_DE         "Probe reaches leftmost position"
#define PROBE_REACH_MAX_RIGHT_DE        "Probe reaches rightmost position"
#define PROBE_REACH_MAX_FRONT_DE        "Probe reaches front position"
#define PROBE_REACH_MAX_BACK_DE         "Probe reaches final position"

#define TEMPERATURE_CONF_TITLE_DE       "Maschinen Einstellungen -> Temperaturen"
#define NOZZLE_CONF_DE                  "Duese Einstellungen"
#define HOTBED_CONF_DE                  "Heizbett Einstellungen"
#define PREHEAT_TEMPER_DE               "Voreinstellungen"

#define NOZZLE_CONF_TITLE_DE            "Maschinen Einstellungen -> Duese"
#define NOZZLECNT_DE                    "Anzahl Duesen"
#define NOZZLE_TYPE_DE                  "E0 Temperatur-Typ"
#define NOZZLE_ADJUST_TYPE_DE           "PID Thermostat"
#define NOZZLE_MIN_TEMPERATURE_DE       "Minimal Temperatur"
#define NOZZLE_MAX_TEMPERATURE_DE       "Maximum Temperatur"
#define EXTRUD_MIN_TEMPER_DE            "Minimale Extrusion Temperatur"

#define HOTBED_CONF_TITLE_DE            "Maschinen Einstellungen -> Heizbett"
#define HOTBED_ADJUST_DE                "PID thermostat"
#define HOTBED_MIN_TEMPERATURE_DE       "Minimale Temperatur"
#define HOTBED_MAX_TEMPERATURE_DE       "Maximum Temperatur"

#define MOTOR_CONF_TITLE_DE             "Maschinen Einstellungen -> Motoren"
#define MAXFEEDRATE_CONF_DE             "Maximale Geschwindigkeit"
#define ACCELERATION_CONF_DE            "Beschleunigung"
#define JERKCONF_DE                     "Jerk settings"
#define STEPSCONF_DE                    "Schrittanzahl"
#define TMC_CURRENT_DE                  "TMC Strom"
#define TMC_STEP_MODE_DE                "TMC Schritt Modus"
#define MOTORDIRCONF_DE                 "Motor Richtung"
#define HOMEFEEDRATECONF_DE             "Homing Geschwindigkeit"
#define HOMING_SENSITIVITY_CONF_DE      "Homing Empfindlichkeit"

#define MAXFEEDRATE_CONF_TITLE_DE       "Maschinen Einstellungen -> Maximale Geschwindigkeit"
#define X_MAXFEEDRATE_DE                "X-Achse Maximum"
#define Y_MAXFEEDRATE_DE                "Y-Achse Maximum"
#define Z_MAXFEEDRATE_DE                "Z-Achse Maximum"
#define E0_MAXFEEDRATE_DE               "E0 Maximum"
#define E1_MAXFEEDRATE_DE               "E1 Maxiumum"

#define ACCELERATION_CONF_TITLE_DE      "Maschinen Einstellungen ->Beschleunigung"
#define PRINT_ACCELERATION_DE           "Beim Drucken"
#define RETRACT_ACCELERATION_DE         "Beim Rueckzug"
#define TRAVEL_ACCELERATION_DE          "Beim Verfahren"
#define X_ACCELERATION_DE               "X-Achse Beschleunigung"
#define Y_ACCELERATION_DE               "Y-Achse Beschleunigung"
#define Z_ACCELERATION_DE               "Z-Achse Beschleunigung"
#define E0_ACCELERATION_DE              "E0 Beschleunigung"
#define E1_ACCELERATION_DE              "E1 Beschleunigung"

#define JERK_CONF_TITLE_DE              "Maschinen Einstellungen -> Jerk speed"
#define X_JERK_DE                       "X-Achse Jerk Geschwindigkeit"
#define Y_JERK_DE                       "Y-Achse Jerk Geschwindigkeit"
#define Z_JERK_DE                       "Z-Achse Jerk Geschwindigkeit"
#define E_JERK_DE                       "Extruder Jerk Geschwindigkeit"

#define STEPS_CONF_TITLE_DE             "Maschinen Einstellungen -> TMC Schritte"
#define X_STEPS_DE                      "X-Achse Schritte"
#define Y_STEPS_DE                      "Y-Achse Schritte"
#define Z_STEPS_DE                      "Z-Achse Schritte"
#define E0_STEPS_DE                     "E0 Schritte"
#define E1_STEPS_DE                     "E1 Schritte"

#define TMC_CURRENT_CONF_TITLE_DE       "Maschinen Einstellungen -> TMC Strom"
#define X_TMC_CURRENT_DE                "X-Achse Strom (mA)"
#define Y_TMC_CURRENT_DE                "Y-Achse Strom (mA)"
#define Z_TMC_CURRENT_DE                "Z-Achse Strom (mA)"
#define E0_TMC_CURRENT_DE               "E0 Strom (mA)"
#define E1_TMC_CURRENT_DE               "E1 Strom (mA)"

#define TMC_MODE_CONF_TITLE_DE          "Maschinen Einstellungen -> TMC Schrittmodus"
#define X_TMC_MODE_DE                   "X-Achse stealthChop-Modus ?"
#define Y_TMC_MODE_DE                   "Y-Achse stealthChop-Modus ?"
#define Z_TMC_MODE_DE                   "Z-Achse stealthChop-Modus ?"
#define E0_TMC_MODE_DE                  "E0 stealthChop-Modus ?"
#define E1_TMC_MODE_DE                  "E1 stealthChop-Modus ?"

#define MOTORDIR_CONF_TITLE_DE          "Maschinen Einstellungen -> Drehrichtung"
#define X_MOTORDIR_DE                   "X-Achse Motor Richtung umkehren"
#define Y_MOTORDIR_DE                   "Y-Achse Motor Richtung umkehren"
#define Z_MOTORDIR_DE                   "Z-Achse Motor Richtung umkehren"
#define E0_MOTORDIR_DE                  "E0 Motor Richtung umkehren"
#define E1_MOTORDIR_DE                  "E1 Motor Richtung umkehren"
#define INVERT_P_DE                     "JA"
#define INVERT_N_DE                     "NEIN"

#define HOMEFEEDRATE_CONF_TITLE_DE      "Maschinen Einstellungen -> Home Speed"
#define X_HOMESPEED_DE                  "XY-Achse Home Geschw."
#define Y_HOMESPEED_DE                  "Y-Achse Home Geschw."
#define Z_HOMESPEED_DE                  "Z-Achse Home Geschw."

#define ADVANCED_CONF_TITLE_DE          "Maschinen Einstellungen -> Weitere"
#define PWROFF_DECTION_DE               "Netzausfallerkennung"
#define PWROFF_AFTER_PRINT_DE           "Nach Druckende herunterfahren ?"
#define HAVE_UPS_DE                     "USV vorhanden ?"
#define Z2_AND_Z2ENDSTOP_CONF_DE        "Z2 Einstellungen"
#define ENABLE_PINS_CONF_DE             "Logiklevel an Pins"
#define WIFI_SETTINGS_DE                "WLAN Einstellungen"
#define HOMING_SENSITIVITY_CONF_DE      "Homing Empfindlichkeit"
#define ENCODER_SETTINGS_DE             "Drehencoder"

#define Z2_AND_Z2ENDSTOP_CONF_TITLE_DE  "Z2 Einstellungen"
#define Z2_ENABLE_DE                    "Z2 Aktivieren"
#define Z2_ENDSTOP_DE                   "Z2 Endschalter Aktivieren"
#define Z2_PORT_DE                      "Z2 Steckplatz"

#define ENABLE_PINS_CONF_TITLE_DE       "Logiklevel an Pins"
#define X_ENABLE_PINS_INVERT_DE         "Invertiere X-Enable Pin"
#define Y_ENABLE_PINS_INVERT_DE         "Invertiere Y-Enable Pin"
#define Z_ENABLE_PINS_INVERT_DE         "Invertiere Z-Enable Pin"
#define E_ENABLE_PINS_INVERT_DE         "Invertiere E-Enable Pin"

#define PAUSE_POSITION_DE    "Position Druckpause einstellen"
#define PAUSE_POSITION_X_DE  "X-Achse Position (Absolute Position,-1 ungültig)"
#define PAUSE_POSITION_Y_DE  "Y-Achse Position (Absolute Position,-1 ungültig)"
#define PAUSE_POSITION_Z_DE  "Z-Achse Position (Absolute Position,-1 ungültig)"

#define WIFI_SETTINGS_TITLE_DE    "Maschinen Einstellungen -> WLAN"
#define WIFI_SETTINGS_MODE_DE     "WLAN Modus"
#define WIFI_SETTINGS_NAME_DE     "WLAN Name: "
#define WIFI_SETTINGS_PASSWORD_DE "WLAN Passwort: "
#define WIFI_SETTINGS_CLOUD_DE    "Cloud nutzen ?"
#define WIFI_SETTINGS_CONFIG_DE   "Konfigurieren"
#define WIFI_SETTINGS_EDIT_DE     "Aendern"
#define WIFI_CONFIG_TIPS_DE       "WLAN Einstellen ?"

#define OFFSET_TITLE_DE  "Maschinen Einstellungen -> Offset"
#define OFFSET_X_DE      "X Offset"
#define OFFSET_Y_DE      "Y Offset"
#define OFFSET_Z_DE      "Z Offset"

#define HOMING_SENSITIVITY_CONF_TITLE_DE      "Maschinen Einstellungen -> Empfindlichkeit"
#define X_SENSITIVITY_DE                      "X-Achse Empfindlichkeit"
#define Y_SENSITIVITY_DE                      "Y-Achse Empfindlichkeit"
#define Z_SENSITIVITY_DE                      "Z-Achse Empfindlichkeit"
#define Z2_SENSITIVITY_DE                     "Z2-Achse Empfindlichkeit"

#define ENCODER_CONF_TITLE_DE                 "Maschinen Einstellungen -> Drehencoder"
#define ENCODER_CONF_TEXT_DE                  "Wird ein Encoder benutzt ?"

#define TOOL_TEXT_DE            "Werkzeuge"
#define PREHEAT_TEXT_DE         "Vorheizen"
#define MOVE_TEXT_DE            "Bewegen"
#define HOME_TEXT_DE            "Referenz"
#define PRINT_TEXT_DE           "Drucken"
#define EXTRUDE_TEXT_DE         "Extrudieren"
#define LEVELING_TEXT_DE        "Ausrichten"
#define AUTO_LEVELING_TEXT_DE   "AutoLevel"
#define SET_TEXT_DE             "Einstellen"
#define MORE_TEXT_DE            "Mehr"
#define MORE_GCODE_DE           "G-Code"
#define MORE_ENTER_GCODE_DE     "Tippe G-Code"

#define ADD_TEXT_DE             "Mehr"
#define DEC_TEXT_DE             "Weniger"
#define EXTRUDER_1_TEXT_DE      "Extruder"
#define EXTRUDER_2_TEXT_DE      "Extrusion2"
#define HEATBED_TEXT_DE         "Heizbett"
#define TEXT_1C_DE              "1 ℃"
#define TEXT_5C_DE              "5 ℃"
#define TEXT_10C_DE             "10 ℃"
#define CLOSE_TEXT_DE           "Ende"

#define BACK_TEXT_DE            "Ende"

#define TOOL_PREHEAT_DE         "Vorheizen"
#define TOOL_EXTRUDE_DE         "Extrudieren"
#define TOOL_MOVE_DE            "Bewegen"
#define TOOL_HOME_DE            "Referenz"
#define TOOL_LEVELING_DE        "Ausrichten"
#define TOOL_AUTO_LEVELING_DE   "AutoLevel"
#define TOOL_FILAMENT_DE        "Filament"
#define TOOL_MORE_DE            "Mehr"

#define AXIS_X_ADD_TEXT_DE      "X+"
#define AXIS_X_DEC_TEXT_DE      "X-"
#define AXIS_Y_ADD_TEXT_DE      "Y+"
#define AXIS_Y_DEC_TEXT_DE      "Y-"
#define AXIS_Z_ADD_TEXT_DE      "Z+"
#define AXIS_Z_DEC_TEXT_DE      "Z-"
#define TEXT_01MM_DE            "0.1 mm"
#define TEXT_1MM_DE             "1 mm"
#define TEXT_10MM_DE            "10 mm"

#define HOME_X_TEXT_DE          "X"
#define HOME_Y_TEXT_DE          "Y"
#define HOME_Z_TEXT_DE          "Z"
#define HOME_ALL_TEXT_DE        "Referenz"
#define HOME_STOPMOVE_DE        "Schnellstop"

#define PAGE_UP_TEXT_DE         "Seite Auf"
#define PAGE_DOWN_TEXT_DE       "Seite Ab"

#define EXTRUDER_IN_TEXT_DE           "Rein"
#define EXTRUDER_OUT_TEXT_DE          "Raus"
#define EXTRUDE_1MM_TEXT_DE           "1 mm"
#define EXTRUDE_5MM_TEXT_DE           "5 mm"
#define EXTRUDE_10MM_TEXT_DE          "10 mm"
#define EXTRUDE_LOW_SPEED_TEXT_DE     "Niedrig"
#define EXTRUDE_MEDIUM_SPEED_TEXT_DE  "Normal"
#define EXTRUDE_HIGH_SPEED_TEXT_DE    "Hoch"

#define LEVELING_POINT1_TEXT_DE       "Punkt 1"
#define LEVELING_POINT2_TEXT_DE       "Punkt 2"
#define LEVELING_POINT3_TEXT_DE       "Punkt 3"
#define LEVELING_POINT4_TEXT_DE       "Punkt 4"
#define LEVELING_POINT5_TEXT_DE       "Punkt 5"

#define FILESYS_TEXT_DE               "Dateisystem"
#define WIFI_TEXT_DE                  "WiFi"
#define FAN_TEXT_DE                   "Ventilator"
#define ABOUT_TEXT_DE                 "Info"
#define BREAK_POINT_TEXT_DE           "Weiter"
#define FILAMENT_TEXT_DE              "Filament"
#define LANGUAGE_TEXT_DE              "Sprache"
#define MOTOR_OFF_TEXT_DE             "Motor-AUS"
#define MOTOR_OFF_XY_TEXT_DE          "AUS-XY"
#define SHUTDOWN_TEXT_DE              "Runterfahren"
#define MACHINE_PARA_DE               "Einstellungen"
#define EEPROM_SETTINGS_DE            "EPROM Set"

#define U_DISK_TEXT_DE                "USB"
#define SD_CARD_TEXT_DE               "SD"
#define WIFI_NAME_TEXT_DE             "WiFi: "
#define WIFI_KEY_TEXT_DE              "Key: "
#define WIFI_IP_TEXT_DE               "IP: "
#define WIFI_AP_TEXT_DE               "State: AP"
#define WIFI_STA_TEXT_DE              "State: STA"
#define WIFI_CONNECTED_TEXT_DE        "Verbunden"
#define WIFI_DISCONNECTED_TEXT_DE     "Getrennt"
#define WIFI_EXCEPTION_TEXT_DE        "Fehler"
#define WIFI_RECONNECT_TEXT_DE        "Wiederverbinden"
#define CLOUD_TEXT_DE                 "Cloud"
#define CLOUD_BIND_DE                 "Binden"
#define CLOUD_UNBIND_DE               "Loesen"
#define CLOUD_UNBINDING_DE            "Loese..."
#define CLOUD_DISCONNECTED_DE         "Getrennt"
#define CLOUD_UNBINDED_DE             "Geloest"
#define CLOUD_BINDED_DE               "Verbunden"
#define CLOUD_DISABLE_DE              "Inaktiv"

#define FAN_ADD_TEXT_DE               "Mehr"
#define FAN_DEC_TEXT_DE               "Weniger"
#define FAN_OPEN_TEXT_DE              "100 %"
#define FAN_HALF_TEXT_DE              "50 %"
#define FAN_CLOSE_TEXT_DE             "Ende"
#define FAN_TIPS1_TEXT_DE             "Bauteil-Ventilator"
#define FAN_TIPS2_TEXT_DE             "Ende"

#define FILAMENT_IN_TEXT_DE           "Laden"
#define FILAMENT_OUT_TEXT_DE          "Entladen"
#define FILAMENT_EXT0_TEXT_DE         "Extruder"
#define FILAMENT_EXT1_TEXT_DE         "Extrusion2"
#define FILAMENT_HEAT_TEXT_DE         "Vorheizen"
#define FILAMENT_STOP_TEXT_DE         "Stop"

#define FILAMENT_CHANGE_TEXT_DE                 "<Lade> anklicken \noder <Entlade>, nach \nDruckpause."
#define FILAMENT_DIALOG_LOAD_HEAT_TIPS_DE       "Heize auf,\nbitte warten..."
#define FILAMENT_DIALOG_UNLOAD_HEAT_TIPS_DE     "Heize auf,\nbitte warten..."
#define FILAMENT_DIALOG_LOAD_CONFIRM1_TIPS_DE   "Aufheizen beendet,bitte Filament laden \nin Extruder,und quittieren\nzum Ladestart."
#define FILAMENT_DIALOG_LOAD_CONFIRM2_TIPS_DE   "Aufheizen beendet,bitte Filament laden \nin Extruder,und quittieren\nzum Ladestart."
#define FILAMENT_DIALOG_UNLOAD_CONFIRM_TIPS_DE  "Aufheizen beendet,bitte Filament laden \nin Extruder,und quittieren\nzum Entladestart."
#define FILAMENT_DIALOG_LOADING_TIPS_DE         "Es wird geladen, bitte warten!"
#define FILAMENT_DIALOG_UNLOADING_TIPS_DE       "Es wird entladen, bitte warten!"
#define FILAMENT_DIALOG_LOAD_COMPLETE_TIPS_DE   "Laden beendet,\nquittieren zum verlassen!"
#define FILAMENT_DIALOG_UNLOAD_COMPLETE_TIPS_DE "Entladen beendet,\nquittieren zum verlassen!"

#define FILAMENT_TIPS2_TEXT_DE        "T:"
#define FILAMENT_TIPS3_TEXT_DE        "Lade..."
#define FILAMENT_TIPS4_TEXT_DE        "Entlade..."
#define FILAMENT_TIPS5_TEXT_DE        "Temp ist zu gering... bitte heizen!"
#define FILAMENT_TIPS6_TEXT_DE        "Erfolgreich!"

#define PRE_HEAT_EXT_TEXT_DE            "E"
#define PRE_HEAT_BED_TEXT_DE            "Bett"

#define FILE_LOADING_DE                 "Lade......"
#define NO_FILE_AND_CHECK_DE          "  Keine Dateien vorhanden!\n            Dateisystem überprüfen!"
#define NO_FILE_DE                          "Keine Dateien!"

#define EXTRUDER_TEMP_TEXT_DE               "Temperatur"
#define EXTRUDER_E_LENGTH1_TEXT_DE          "Extruder"
#define EXTRUDER_E_LENGTH2_TEXT_DE          "Extrusion2"
#define EXTRUDER_E_LENGTH3_TEXT_DE          "Extrusion3"

#define ABOUT_TYPE_TEXT_DE                  "Typ: "
#define ABOUT_VERSION_TEXT_DE               "Firmware: "
#define ABOUT_WIFI_TEXT_DE                  "WLAN: "

#define PRINTING_TEMP_DE                    "Temp."
#define PRINTING_CHANGESPEED_DE             "Geschw."
#define PRINTING_RESUME_DE                  "Weiter"
#define PRINTING_STOP_DE                    "Stop"
#define PRINTING_MORE_DE                    "Mehr"
#define PRINTING_EXTRUDER_DE                "Extrusion"
#define PRINTING_MOVE_DE                    "Bewegen"

#define EXTRUDER_SPEED_DE                   "Extrusion"
#define MOVE_SPEED_DE                       "Bewegen"
#define EXTRUDER_SPEED_STATE_DE             "Extrudier Geschw."
#define MOVE_SPEED_STATE_DE                 "Bewegung Geschw."
#define STEP_1PERCENT_DE                    "1 %"
#define STEP_5PERCENT_DE                    "5 %"
#define STEP_10PERCENT_DE                   "10 %"

#define TITLE_READYPRINT_DE                 "CTC"
#define TITLE_PREHEAT_DE                    "Vorheizen"
#define TITLE_MOVE_DE                       "Bewegen"
#define TITLE_HOME_DE                       "Referenzfahrt"
#define TITLE_EXTRUDE_DE                    "Extrusion"
#define TITLE_LEVELING_DE                   "Justage"
#define TITLE_SET_DE                        "Einstellungen"
#define TITLE_MORE_DE                       "Mehr"
#define TITLE_CHOOSEFILE_DE                 "Dateiauswahl"
#define TITLE_PRINTING_DE                   "Drucken"
#define TITLE_OPERATION_DE                  "Druck"
#define TITLE_ADJUST_DE                     "Einstellen"
#define TITLE_WIRELESS_DE                   "Wireless"
#define TITLE_FILAMENT_DE                   "Filament"
#define TITLE_ABOUT_DE                      "Info"
#define TITLE_FAN_DE                        "Ventilator"
#define TITLE_LANGUAGE_DE                   "Sprache"
#define TITLE_PAUSE_DE                      "Pause"
#define TITLE_CHANGESPEED_DE                "Geschwindigkeit"
#define TITLE_CLOUD_TEXT_DE                 "Cloud"
#define TITLE_DIALOG_CONFIRM_DE             "OK"
#define TITLE_FILESYS_DE                    "Dateisystem"

#define AUTO_SHUTDOWN_DE                    "Auto"
#define MANUAL_SHUTDOWN_DE                  "Manuell"

#define DIALOG_CONFIRM_DE                   "OK"
#define DIALOG_CANCLE_DE                    "Abbruch"
#define DIALOG_OK_DE                        "OK"
#define DIALOG_RESET_DE                     "Reset"
#define DIALOG_RETRY_DE                     "Wiederholen"
#define DIALOG_DISABLE_DE                   "Deaktivieren"
#define DIALOG_PRINT_MODEL_DE               "Dieses Modell drucken ?"
#define DIALOG_CANCEL_PRINT_DE              "Druck beenden?"
#define DIALOG_RETRY_DE                     "Wiederholen"
#define DIALOG_STOP_DE                      "Stop"
#define DIALOG_REPRINT_FROM_BREAKPOINT_DE   "Weiter bei Haltepunkt?"
#define DIALOG_ERROR_TIPS1_DE               "Fehler: Keine Datei, bitte kontrollieren!"
#define DIALOG_ERROR_TIPS2_DE               "Fehler: Beim senden.Baudraten kontrollieren \nvon Display und Mainboard!"
#define DIALOG_ERROR_TIPS3_DE               "Fehler: Dateiname oder Pfad zu lang!"
#define DIALOG_CLOSE_MACHINE_DE             "Schliesse Maschine......"
#define DIALOG_UNBIND_PRINTER_DE            "Drucker entkoppeln?"
#define DIALOG_FILAMENT_NO_PRESS_DE         "Schalter Filamenterkennung nicht erkannt!"
#define DIALOG_PRINT_FINISH_DE              "Druck beendet!"
#define DIALOG_PRINT_TIME_DE                "Druckzeit: "
#define DIALOG_REPRINT_DE                   "Druck wiederholen?"
#define DIALOG_WIFI_ENABLE_TIPS_DE          "WLAN-Modul ist eingerichtet\nbitte warten..."
#define DIALOG_AUTO_LEVELING_TIPS_DE        "Auto-Leveling, bitte warten..."

#define HOTBED_ENABLE_DE        "Aktiviere Heizbett"
#define MOTOR_EN_HIGH_LEVEL_DE  "Hoch "
#define MOTOR_EN_LOW_LEVEL_DE   "Niedrig"

#define TEXT_WIFI_MENU_TITLE_DE "WLAN"
#define TEXT_WIFI_SAPCE_DE      "space"
#define TEXT_WIFI_LETTER_DE     "abc"
#define TEXT_WIFI_DIGITAL_DE    "123"
#define TEXT_WIFI_SYMBOL_DE     "#+="
#define TEXT_WIFI_PASSWORD_DE   "Passwort"

#define TEXT_WIFI_JOINING_DE  "Trete Netzwerk bei..."
#define TEXT_WIFI_FAILED_JOIN_DE  "Verbindung gescheitert!"
#define TEXT_WIFI_WIFI_CONECTED_DE "WLAN verbunden!"

#define TEXT_BUTTON_DISCONECTED_DE  "Verbindung trennen"
#define TEXT_WIFI_FORGET_DE         "Entferne Netzwerk"
#define TEXT_DISCONECTED_DE         "WLAN verbunden!"

// wifi-list
#define MAIN_BUILT_DE       "Build"
#define MAIN_FILAMENT_DE    "Filament"
#define MAIN_SETUP_DE       "Einstellungen"
#define MAIN_ABOUT_DE       "Info"
#define MAIN_MENU_DE        "Hauptfenster"
#define FILE_MENU_BUILD_DE  "Build"
#define FILE_MENU_MENU_DE   " < Menu"

// about
#define ABOUT_TITLE_DE                    "Info"
#define ABOUT_BUILT_MACHINES_DE           "Built Machines"
#define ABOUT_SPARK_DE                    "Spark"
#define ABOUT_VERSION_DE                  "Version 1.1.0"
#define ABOUT_SERIAL_NUMBER_DE            "Seriennummer:"
#define ABOUT_S_NUMBER_DE                 "DCPLX02KFC6P"

// set
#define SETUP_TITLE_DE                    "Einstellungen"
#define SETUP_WIFI_DE                     "Wi-Fi"
#define SETUP_MANUAL_IP_DE                "Statische IP"
#define SETUP_WIFI_NOT_CONNECTED_DE       "Nicht verbunden"
#define SETUP_WIFI_NETWORK_DE             "WLAN-Netzwerk"

// build
#define BUILD_TITLE_DE                    "Build"
#define BUILD_SD_CARD_DE                  "SD Card"
#define BUILD_USB_DRIVE_DE                "USB Drive"

// SD card
#define SD_CARD_TITLE_DE                  "SD Card"
#define SD_CARD_BACK_DE                   "< Vorherige"
// USB Drive
#define USB_DRIVE_TITLE_DE                "USB Laufwerk"
#define USB_DRIVE_BACK_DE                 "< Vorherige"
#define FILE_PAGES_DE                     "%d/%d"
#define FILE_NEXT_PAGE_DE                 "Weiter"
#define MEDIA_SELECT_TITLE_DE             "Medium aussuchen"

// BUILD PLATE
#define PLATE_TITLE_DE                    "Druckplatte"
#define PLATE_BACK_DE                     "< Vorherige"
#define PLATE_CONFIRM_DE                  "OK >"
#define PLATE_TIPS_DE                     "Quittieren Sie\ndas die Druckplatte\nleer ist."

// build model
#define MODEL_TITLE_DE                    "Drucke Modell"
#define MODEL_START_BUILD_DE              "Starte Druck"
#define MODEL_BACK_DE                     "< Vorherige"

// building
#define BUILDING_TITLE_DE                 "Drucke..."
#define BUILDING_MENU_DE                  "Druckfenster"
#define BUILDING_COMPLETED_DE             "Druck\nbeendet"

// building menu
#define BUILDING_MENU_TITLE_DE            "Druckfenster"
#define BUILDING_MENU_SETTINGS_DE         "Druck Einstellungen"
#define BUILDING_MENU_PAUSE_DE            "Pausiere Druck"
#define BUILDING_MENU_CANCEL_DE           "Druck abbrechen"
#define BUILDING_MENU_BACK_DE             "< Vorherige"

// build settings
#define SETTINGS_TITLE_DE                 "Druck Einstellungen"
#define SETTINGS_NOZZLE_TEMPER_DE         "Extruder Temperatur:"
#define SETTINGS_NOZZLE_VALUE_DE          "%d"
#define SETTINGS_BED_TEMPER_DE            "Bett Temperatur:"
#define SETTINGS_BED_VALUE_DE             "%d"
#define SETTINGS_BUILD_SPEED_DE           "Druckgeschwindigkeit:"
#define SETTINGS_SPEED_VALUE_DE           "Standard"
#define SETTINGS_BACK_DE                  "< Vorherige"

// build paused
#define PAUSED_TITLE_DE                   "Druck pausiert"
#define PAUSED_RESUME_DE                  "Druck fortsetzen"
#define PAUSED_CANCEL_DE                  "Druck abbrechen"
#define PAUSED_BACK_DE                    "< Vorherige"

// build cancel
#define CANCEL_TITLE_DE                   "Druck abbrechen"
#define CANCEL_BUILD_DE                   "Druck abbrechen"
#define CANCEL_TIPS_DE                    "Sind Sie sicher, dass\nden Druck abbrechen wollen? Modell\nwird entfernt von\nMaschine. Erneutes\nSenden notwendig\num erneut zu\ndrucken."
#define CANCEL_BACK_DE                    "< Vorherige"
#define CANCEL_BUILD_DISPLAY_DE           "Druck\nabgebrochen"
#define CANCEL_OVER_PLATE_TIPS_DE         "Quittieren Sie\ndass die Druckplatte von\nder Maschine entfernt wurde."

// filament model enter
#define FILAMENT_MODEL_ENTER_TITLE_DE     "Model-PLA"
#define FILAMENT_MODEL_ENTER_BACK_DE      "< Vorherige"
#define FILAMENT_MODEL_ENTER_BEGIN_DE     "Weiter >"
#define FILAMENT_MODEL_ENTER_TIPS_DE      "The Model Filament spool\ncompartment is located on\nthe right side of the machine."

// filament model PLA
#define FILAMENT_MODEL_PLA_TITLE_DE       "Model-PLA"
#define FILAMENT_PLA_LOAD_TITLE_DE        "Load Filament"
#define FILAMENT_PLA_UNLOAD_TITLE_DE      "Unload Filament"
#define FILAMENT_MODEL_PLA_LOAD_DE        "Load Filament"
#define FILAMENT_MODEL_PLA_UNLOAD_DE      "Unload Filament"
// filament support enter
#define FILAMENT_SUPPORT_ENTER_TITLE_DE   "Support-PVA"
#define FILAMENT_SUPPORT_ENTER_BACK_DE    "< Back"
#define FILAMENT_SUPPORT_ENTER_BEGIN_DE   "Begin >"
#define FILAMENT_SUPPORT_ENTER_TIPS_DE    "The Support Filament spool\ncompartment is located on\nthe left side of the machine."
// filament heating
#define FILAMENT_HEATING_LOAD_TITLE_DE    "Load Filament"
#define FILAMENT_HEATING_UNLOAD_TITLE_DE  "Unload Filament"
#define FILAMENT_HEATING_CANCEL_DE        "< Cancel"
#define FILAMENT_HEATING_MATERIAL_DE      "Material:"
#define FILAMENT_HEATING_PLA_DE           "Model-PLA"
#define FILAMENT_HEATING_TIPS_DE          "Print head is heating..."
// rotate left
#define ROTATE_LEFT_LOAD_TITLE_DE         "Load Filament"
#define ROTATE_LEFT_UNLOAD_TITLE_DE       "Unload Filament"
#define ROTATE_LEFT_CANCEL_DE             "< Cancel"
#define ROTATE_LEFT_MATERIAL_DE           "Material:"
#define ROTATE_LEFT_PLA_DE                "Model-PLA"
#define ROTATE_LEFT_NEXT_DE               "Next >"
#define ROTATE_LEFT_TIPS_DE               "Rotate extruder selection\ndial to the left."

// hang spool
#define HANG_SPOOL_TITLE_DE         "Load Filament"
#define HANG_SPOOL_PREVIOUS_DE      "< Previous"
#define HANG_SPOOL_MATERIAL_DE      "Material:"
#define HANG_SPOOL_PLA_DE           "Model-PLA"
#define HANG_SPOOL_NEXT_DE          "Next >"
#define HANG_SPOOL_TIPS_DE          "Hang the spool in the spool\ncompartment as shown."

// feed filament
#define FEED_FILAMENT_TITLE_DE      "Load Filament"
#define FEED_FILAMENT_PREVIOUS_DE   "< Previous"
#define FEED_FILAMENT_MATERIAL_DE   "Material:"
#define FEED_FILAMENT_PLA_DE        "Model-PLA"
#define FEED_FILAMENT_NEXT_DE       "Next >"
#define FEED_FILAMENT_TIPS_DE       "Feed filament into extruder\nup beyond the gears."

// feed filament
#define ROTATE_UP_TITLE_DE          "Load Filament"
#define ROTATE_UP_PREVIOUS_DE        "< Previous"
#define ROTATE_UP_MATERIAL_DE       "Material:"
#define ROTATE_UP_PLA_DE            "Model-PLA"
#define ROTATE_UP_NEXT_DE           "Next >"
#define ROTATE_UP_TIPS_DE           "Rotate extruder selection\ndial up."

// filamDEt begin
#define FEED_BEGIN_TITLE_DE         "Load Filament"
#define FEED_BEGIN_MATERIAL_DE      "Material:"
#define FEED_BEGIN_PLA_DE           "Model-PLA"
#define FEED_BEGIN_NEXT_DE          "Next >"
#define FEED_BEGIN_TIPS_DE          "Press Next when filament\nbegins to extrude."

// filament finish
#define FEED_FINISH_TITLE_DE        "Load Filament"
#define FEED_FINISH_MATERIAL_DE     "Material:"
#define FEED_FINISH_PLA_DE          "Model-PLA"
#define FEED_FINISH_NEXT_DE         "Finish >"
#define FEED_FINISH_TIPS_DE         "Remove filament from the\nnozzle and discard."
// fiament remove
#define REMOVE_SPOOL_TITLE_DE       "Unload Filament"
#define REMOVE_SPOOL_PREVIOUS_DE     "< Previous"
#define REMOVE_SPOOL_FINISH_DE      "Finish >"
#define REMOVE_SPOOL_MATERIAL_DE    "Material:"
#define REMOVE_SPOOL_PLA_DE         "Model-PLA"
#define REMOVE_SPOOL_TIPS_DE        "Remove the spool and pull\nfilament out of the machine."

#define FILAMENT_SUPPORT_PVA_DE     "Support-PVA"
#define LOAD_FINISH_DE              "Load\nFilament\nComplete"
#define UNLOAD_FINISH_DE            "Unload\nFilament\nComplete"

// manual ip
#define MANUAL_IP_TITLE_DE          "Statische IP"
#define MANUAL_IP_CANCEL_DE         "< Abbrechen"
#define MANUAL_IP_APPLY_DE          "Beitreten >"
#define MANUAL_IP_ADDRESS_DE        "IP Addresse"
#define MANUAL_IP_MASK_DE           "Subnet Mask"
#define MANUAL_IP_GATEWAY_DE        "Default Gateway"
#define MANUAL_IP_SERVER_DE         "Name Server"
#define MANUAL_IP_INIT_DATA_DE      "0.0.0.0"
#define MANUAL_TEXT_POINT_DE        "."
#define MANUAL_TEXT_ENTER_DE        "Enter"

#define TEXT_FORGET_TIPS_TITLE_DE "Entferne Netzwerk"
#define TEXT_FORGET_NETWORK_TIPS1_DE "Sind Sie sicher\ndas Netzwerk zu entfernen?"
#define TEXT_FORGET_NETWORK_TIPS2_DE "Maschine gehört nicht\nmehr zu diesem WLAN-Netz."

#define TEXT_IPADDRESS_DE "IP Addresse: "

#define TEXT_BUILD_FROM_CURA_CANCEL_TIPS1_DE "Sind Sie sicher\nden Druck abzubrechen?"
#define TEXT_BUILD_FROM_CURA_CANCEL_TIPS2_DE "Sind Sie sicher, dass\nden Druck abbrechen wollen? Modell\nwird gelöscht von\nMaschine. Erneute\nÜbertragung notwendig\nfür einen erneuten\nDruck."

#define DIALOG_CONFIRM_DE2              "OK"

#define HEATING_TITLE_DE  "Heizen"
#define LEVELING_TITLE_DE  "Leveln"

#define ABOUT_SPARK_ADD_DE "Spark+"

#define TEXT_RECEIVING_DATA_DE "Empfange Daten"

#define TEXT_BABY_STEP_DE "Babystep"

#define PRINTING_DE              "Drucke"
#define PRINTING_OPERATION_DE    "Druck"
#define PRINTING_PAUSE_DE        "Pause"

#define MESSAGE_PAUSING_DE        "Parke..."
#define MESSAGE_CHANGING_DE       "Warte auf Filamentwechsel"
#define MESSAGE_UNLOAD_DE         "Warte auf Filament entladen"
#define MESSAGE_WAITING_DE        "Taste klicken zum fortsetzen"
#define MESSAGE_INSERT_DE         "Filament einlegen, und Taste klicken"
#define MESSAGE_LOAD_DE           "Warte auf Filament laden"
#define MESSAGE_PURGE_DE          "Warte auf Filament Ausstoss"
#define MESSAGE_RESUME_DE         "Warte bis Druck fortgesetzt wird..."
#define MESSAGE_HEAT_DE           "Taste drücken zum Extruder aufheizen"
#define MESSAGE_HEATING_DE        "Extrufer heizt auf, bitte warten..."
#define MESSAGE_OPTION_DE         "Mehr Ausstossen oder Druck fortsetzen ?"
#define MESSAGE_PURGE_MORE_DE     "Ausstoss..."
#define MESSAGE_CONTINUE_PRINT_DE "Drucke..."
#define EEPROM_SETTINGS_TITLE_DE  "EEPROM Einstellungen"
#define EEPROM_SETTINGS_STORE_DE  "Speicher Einstellungen im EEPROM"
#define EEPROM_SETTINGS_READ_DE   "Lese Einstellungen von EEPROM"
#define EEPROM_SETTINGS_REVERT_DE "Auf Werkseinstellung setzen?"

#define EEPROM_STORE_TIPS_DE  "Einstellungen speichern im EEPROM?"
#define EEPROM_READ_TIPS_DE   "Einstellungen lesen von EEPROM?"
#define EEPROM_REVERT_TIPS_DE "Auf Werkseinstellung setzen?"

#define MORE_CUSTOM1_TEXT_DE  MAIN_MENU_ITEM_1_DESC
#define MORE_CUSTOM2_TEXT_DE  MAIN_MENU_ITEM_2_DESC
#define MORE_CUSTOM3_TEXT_DE  MAIN_MENU_ITEM_3_DESC
#define MORE_CUSTOM4_TEXT_DE  MAIN_MENU_ITEM_4_DESC
#define MORE_CUSTOM5_TEXT_DE  MAIN_MENU_ITEM_5_DESC
#define MORE_CUSTOM6_TEXT_DE  MAIN_MENU_ITEM_6_DESC
