#ifndef PRODUCT_INFO_H
#define PRODUCT_INFO_H

#include <stdint.h>
#include "protocol.h"

#define PRODUCT_NAME "DECT NR+ PHY MESH"
#define FIRMWARE_VERSION 100

#define MAX_ANCHORS 8
#define MAX_SENSORS 128

/* Runtime device type (set by product_info_init from GPIO pins). */
extern device_type_t PRODUCT_DEVICE_TYPE;

/* Read GPIO pins P0.21/P0.23 to set device type.
 * Must be called before any mesh operations. */
int product_info_init(void);

#endif /* PRODUCT_INFO_H */
