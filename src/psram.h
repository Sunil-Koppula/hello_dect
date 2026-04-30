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

/*
 * PSRAM partition map — keep all PSRAM consumers anchored to these.
 *
 *   P1: 0x000000 – 0x0FFFFF (1 MB) — tracker payloads + reserved future use
 *   P2: 0x100000 – 0x1FFFFF (1 MB) — reserved for small (≤256 B) data slots
 *   P3: 0x200000 – 0x7FFFFF (6 MB) — large (4 KB) data slots
 *
 * Selection logic for P2 vs P3 (size-based small/large transfers) is
 * deferred until small/large transfer types are added.
 */
#define PSRAM_P1_BASE    0x000000
#define PSRAM_P1_SIZE    (1 * 1024 * 1024)  /* 1 MB */

#define PSRAM_P2_BASE    0x100000
#define PSRAM_P2_SIZE    (1 * 1024 * 1024)  /* 1 MB */

#define PSRAM_P3_BASE    0x200000
#define PSRAM_P3_SIZE    (6 * 1024 * 1024)  /* 6 MB */

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
