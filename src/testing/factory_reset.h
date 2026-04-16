#ifndef FACTORY_RESET_H
#define FACTORY_RESET_H

/*
 * Factory reset via Button 1 (testing/dev only).
 *
 * Press and hold Button 1 for 3 seconds to clear all EEPROM/NVS
 * partitions and reboot. Short press is ignored.
 *
 * To remove for production: delete these files and remove
 * factory_reset_init() call from main.c.
 */

/* Initialize factory reset button. Call once at boot before main loop. */
int factory_reset_init(void);

#endif /* FACTORY_RESET_H */
