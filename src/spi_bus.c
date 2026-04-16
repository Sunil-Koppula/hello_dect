/*
 * Shared SPI3 bus mutex for external flash + PSRAM.
 */

#include <zephyr/kernel.h>
#include "spi_bus.h"

K_MUTEX_DEFINE(spi_bus_mutex);

void spi_bus_lock(void)
{
	k_mutex_lock(&spi_bus_mutex, K_FOREVER);
}

void spi_bus_unlock(void)
{
	k_mutex_unlock(&spi_bus_mutex);
}
