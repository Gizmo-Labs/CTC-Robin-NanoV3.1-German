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

/**
 * Datei: pins/pins.h
 *
 * Einbinden der Pin-Defintionen
 *
 * In Abhängigkeit des Board-Namens wird die entsprechende
 * Pin-Definition ausgewählt. 
 * 
 */

#define MAX_E_STEPPERS 1

// Zum Testen... für Firmware-Build uninteressant
#ifdef __MARLIN_DEPS__
  #define NOT_TARGET(V...) 0
#else
  #define NOT_TARGET(V...) NONE(V)
#endif

// Wenn Board Nano V3.x wähle zugehörige Pin-Definition
#if MB(MKS_ROBIN_NANO_V3)
  #include "stm32f4/pins_MKS_ROBIN_NANO_V3.h"  // STM32F4                                env:mks_robin_nano_v3_usb_flash_drive_msc 
#endif

//
// Pin-Vorbelegung einbinden
//
#include "pins_postprocess.h"
