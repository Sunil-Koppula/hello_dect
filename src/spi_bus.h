#ifndef SPI_BUS_H
#define SPI_BUS_H

/*
 * Shared SPI3 bus mutex.
 *
 * Protects concurrent access to devices on the same SPI bus:
 *   - GD25WB256 external flash (NVS storage)
 *   - APS6404L PSRAM (queue overflow)
 *
 * All code that accesses either device must lock/unlock around
 * multi-step SPI operations.
 */

/* Lock the SPI bus (blocks until available). */
void spi_bus_lock(void);

/* Unlock the SPI bus. */
void spi_bus_unlock(void);

#endif /* SPI_BUS_H */
