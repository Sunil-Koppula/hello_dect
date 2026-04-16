#ifndef PSRAM_H
#define PSRAM_H

#include <stdint.h>
#include <stddef.h>

/*
 * SPI PSRAM (APS6404L) wrapper — 8MB external RAM.
 *
 * Accessed via SPI flash API (not memory-mapped).
 * Use for large data staging, overflow buffers, etc.
 *
 * Address range: 0x000000 — 0x7FFFFF (8MB)
 */

#define PSRAM_SIZE       (8 * 1024 * 1024)  /* 8MB */

/* Initialize PSRAM device. */
int psram_init(void);

/* Read data from PSRAM. */
int psram_read(uint32_t addr, void *buf, size_t len);

/* Write data to PSRAM. */
int psram_write(uint32_t addr, const void *buf, size_t len);

/* Erase a region (required before write on some flash APIs, PSRAM doesn't
 * actually need erase but the flash API may require it). */
int psram_erase(uint32_t addr, size_t len);

/* Zero-fill a region. */
int psram_clear(uint32_t addr, size_t len);

#endif /* PSRAM_H */
