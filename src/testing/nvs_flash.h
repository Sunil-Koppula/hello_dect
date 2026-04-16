#ifndef NVS_FLASH_H
#define NVS_FLASH_H

/*
 * NVS flash storage backend — drop-in replacement for EEPROM storage.
 *
 * Uses Zephyr NVS on external flash (or internal flash partition).
 * Implements the same API as storage.h so the rest of the codebase
 * doesn't need any changes.
 *
 * To use NVS instead of EEPROM:
 *   1. Set CONFIG_USE_NVS_STORAGE=y in prj.conf
 *   2. In CMakeLists.txt, nvs_flash.c is compiled instead of storage.c
 *   3. Remove CONFIG_EEPROM / CONFIG_EEPROM_AT24 / CONFIG_I2C if not needed
 *
 * To remove this file for production (EEPROM only):
 *   1. Remove CONFIG_USE_NVS_STORAGE from prj.conf
 *   2. Remove nvs_flash.c / nvs_flash.h from the project
 *   3. No other code changes needed
 */

#include "../storage.h"

/* All functions are provided by storage.h — this header exists only
 * for documentation and to make the NVS backend discoverable. */

#endif /* NVS_FLASH_H */
