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

#define ALLOW_STM32DUINO
#include "env_validate.h"

#if HOTENDS > 2 || E_STEPPERS > 2
  #error "MKS Robin Nano V3 unterstützt nicht mehr als 2 Hotends oder Extruder."
#elif HAS_FSMC_TFT
  #error "MKS Robin Nano V3 unterstützt keine FSMC-basierten TFT Displays."
#endif

/**
 * Deklaration des Board-Namens zur Anzeige im "Firmware-Builder."
 * Dies ist nur ein Text zur Anzeige, keine weitere Funktion. 
*/
#define BOARD_INFO_NAME "MKS Robin Nano V3"

/**
 * Einbinden der Pin-Konfiguration 
*/
#include "pins_MKS_ROBIN_NANO_V3_common.h"
