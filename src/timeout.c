/*
 * Non-blocking timeout utility for DECT NR+ mesh network.
 */

#include <zephyr/kernel.h>
#include "timeout.h"

void nbtimeout_init(struct nbtimeout *t, uint32_t duration_ms, uint8_t max_retries)
{
	t->deadline_ms = 0;
	t->duration_ms = duration_ms;
	t->retries = 0;
	t->max_retries = max_retries;
}

void nbtimeout_start(struct nbtimeout *t)
{
	t->deadline_ms = k_uptime_get() + t->duration_ms;
}

void nbtimeout_stop(struct nbtimeout *t)
{
	t->deadline_ms = 0;
}

bool nbtimeout_is_active(struct nbtimeout *t)
{
	return (t->deadline_ms != 0);
}

bool nbtimeout_expired(struct nbtimeout *t)
{
	if (t->deadline_ms == 0) {
		return false;
	}

	return (k_uptime_get() >= t->deadline_ms);
}

bool nbtimeout_retry(struct nbtimeout *t)
{
	t->retries++;

	if (t->retries > t->max_retries) {
		nbtimeout_stop(t);
		return true;
	}

	nbtimeout_start(t);
	return false;
}

void nbtimeout_reset(struct nbtimeout *t)
{
	t->retries = 0;
	t->deadline_ms = 0;
}

uint8_t nbtimeout_retries(struct nbtimeout *t)
{
	return t->retries;
}

uint8_t nbtimeout_max_retries(struct nbtimeout *t)
{
	return t->max_retries;
}
