/*
 * SPI PSRAM (APS6404L) driver for DECT NR+ mesh network.
 *
 * Uses raw SPI transactions (not flash API) since PSRAM is not NOR flash.
 * APS6404L commands:
 *   0x03 = Read  (cmd + 24-bit addr, then MISO data)
 *   0x02 = Write (cmd + 24-bit addr, then MOSI data)
 *   0x66 = Reset Enable
 *   0x99 = Reset
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include "psram.h"
#include "spi_bus.h"

LOG_MODULE_REGISTER(psram, CONFIG_MAIN_LOG_LEVEL);

#define PSRAM_CMD_READ   0x03
#define PSRAM_CMD_WRITE  0x02
#define PSRAM_CMD_RSTEN  0x66
#define PSRAM_CMD_RST    0x99

static const struct spi_dt_spec psram_spi = SPI_DT_SPEC_GET(
	DT_NODELABEL(psram),
	SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER,
	0);

int psram_init(void)
{
	if (!spi_is_ready_dt(&psram_spi)) {
		LOG_ERR("PSRAM SPI device not ready");
		return -ENODEV;
	}

	/* Reset the PSRAM. */
	uint8_t cmd;

	cmd = PSRAM_CMD_RSTEN;
	struct spi_buf tx_buf = { .buf = &cmd, .len = 1 };
	struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };

	spi_bus_lock();
	spi_write_dt(&psram_spi, &tx_set);

	cmd = PSRAM_CMD_RST;
	spi_write_dt(&psram_spi, &tx_set);
	spi_bus_unlock();

	k_usleep(100);  /* tRST = 50us max */

	LOG_INF("PSRAM initialized (APS6404L, %d KB)", PSRAM_SIZE / 1024);

	return 0;
}

int psram_read(uint32_t addr, void *buf, size_t len)
{
	if (addr + len > PSRAM_SIZE) {
		LOG_ERR("PSRAM read out of bounds: addr=0x%06x len=%d", addr, len);
		return -EINVAL;
	}

	/* TX: command + 24-bit address. */
	uint8_t cmd[4] = {
		PSRAM_CMD_READ,
		(addr >> 16) & 0xFF,
		(addr >> 8) & 0xFF,
		addr & 0xFF,
	};

	struct spi_buf tx_bufs[] = {
		{ .buf = cmd, .len = sizeof(cmd) },
	};
	struct spi_buf rx_bufs[] = {
		{ .buf = NULL, .len = sizeof(cmd) },  /* skip cmd bytes on RX */
		{ .buf = buf, .len = len },
	};
	struct spi_buf_set tx_set = { .buffers = tx_bufs, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = rx_bufs, .count = 2 };

	spi_bus_lock();
	int err = spi_transceive_dt(&psram_spi, &tx_set, &rx_set);
	spi_bus_unlock();

	return err;
}

int psram_write(uint32_t addr, const void *buf, size_t len)
{
	if (addr + len > PSRAM_SIZE) {
		LOG_ERR("PSRAM write out of bounds: addr=0x%06x len=%d", addr, len);
		return -EINVAL;
	}

	/* TX: command + 24-bit address + data. */
	uint8_t cmd[4] = {
		PSRAM_CMD_WRITE,
		(addr >> 16) & 0xFF,
		(addr >> 8) & 0xFF,
		addr & 0xFF,
	};

	struct spi_buf tx_bufs[] = {
		{ .buf = cmd, .len = sizeof(cmd) },
		{ .buf = (void *)buf, .len = len },
	};
	struct spi_buf_set tx_set = { .buffers = tx_bufs, .count = 2 };

	spi_bus_lock();
	int err = spi_write_dt(&psram_spi, &tx_set);
	spi_bus_unlock();

	return err;
}

int psram_erase(uint32_t addr, size_t len)
{
	/* PSRAM doesn't need erase — just zero-fill. */
	return psram_clear(addr, len);
}

int psram_clear(uint32_t addr, size_t len)
{
	if (addr + len > PSRAM_SIZE) {
		LOG_ERR("PSRAM clear out of bounds: addr=0x%06x len=%d", addr, len);
		return -EINVAL;
	}

	uint8_t zero[256] = { 0 };
	int err;

	while (len > 0) {
		size_t chunk = (len > sizeof(zero)) ? sizeof(zero) : len;

		err = psram_write(addr, zero, chunk);
		if (err) {
			return err;
		}

		addr += chunk;
		len -= chunk;
	}

	return 0;
}
