#ifndef TIMEOUT_H
#define TIMEOUT_H

#include <stdint.h>
#include <stdbool.h>

/* Non-blocking timeout handle. */
struct nbtimeout {
	int64_t deadline_ms;  /* absolute time when timeout expires (0 = inactive) */
	uint32_t duration_ms; /* configured duration */
	uint8_t retries;      /* retry count so far */
	uint8_t max_retries;  /* max retries before giving up */
};

/* Initialize a timeout with duration and max retries. Does not start it. */
void nbtimeout_init(struct nbtimeout *t, uint32_t duration_ms, uint8_t max_retries);

/* Start (or restart) the timeout from now. */
void nbtimeout_start(struct nbtimeout *t);

/* Stop the timeout. */
void nbtimeout_stop(struct nbtimeout *t);

/* Returns true if the timeout is currently running. */
bool nbtimeout_is_active(struct nbtimeout *t);

/* Returns true if the timeout has expired. Does NOT auto-stop. */
bool nbtimeout_expired(struct nbtimeout *t);

/* Increment retry counter and restart the timeout.
 * Returns true if retries exhausted (caller should give up). */
bool nbtimeout_retry(struct nbtimeout *t);

/* Reset retries and stop the timeout. */
void nbtimeout_reset(struct nbtimeout *t);

/* Get current retry count. */
uint8_t nbtimeout_retries(struct nbtimeout *t);

/* Get max retries. */
uint8_t nbtimeout_max_retries(struct nbtimeout *t);

#endif /* TIMEOUT_H */
