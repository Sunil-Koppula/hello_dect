#ifndef PSRAM_H
#define PSRAM_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/*
 * SPI PSRAM (APS6404L) wrapper — 8MB external RAM.
 *
 * Accessed via SPI flash API (not memory-mapped).
 * Use for large data staging, overflow buffers, etc.
 *
 * Address range: 0x000000 — 0x7FFFFF (8MB)
 */

#define PSRAM_SIZE       (8 * 1024 * 1024)  /* 8MB */

/* Returns true if PSRAM was successfully initialized. */
bool is_psram_initialized(void);

/* Initialize PSRAM device. Returns 0 on success, -ENODEV if not present. */
int psram_init(void);

/* Read data from PSRAM. Returns -ENODEV if PSRAM not present. */
int psram_read(uint32_t addr, void *buf, size_t len);

/* Write data to PSRAM. Returns -ENODEV if PSRAM not present. */
int psram_write(uint32_t addr, const void *buf, size_t len);

/* Erase a region. PSRAM doesn't need erase — zero-fills instead. */
int psram_erase(uint32_t addr, size_t len);

/* Zero-fill a region. */
int psram_clear(uint32_t addr, size_t len);

#endif /* PSRAM_H */
