#ifndef PRODUCT_INFO_H
#define PRODUCT_INFO_H

#include <stdint.h>
#include "protocol.h"

#define PRODUCT_NAME "DECT NR+ PHY MESH"
#define FIRMWARE_VERSION 100

/* Runtime device type (set by product_info_init from GPIO pins). */
extern device_type_t PRODUCT_DEVICE_TYPE;

/* Runtime device ID (set by product_info_init from HWINFO). */
extern uint16_t device_id;

/* Read GPIO pins P0.21/P0.23 and HWINFO to set device type and ID.
 * Must be called before any mesh operations. */
int product_info_init(void);

#endif /* PRODUCT_INFO_H */
