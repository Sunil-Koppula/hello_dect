#ifndef FRAGMENT_H
#define FRAGMENT_H

#include <stdint.h>
#include <stddef.h>

/*
 * Fragmentation header (3 bytes, prepended to each fragment payload):
 *   [0] frag_id    — unique ID for this transfer
 *   [1] frag_index — fragment number (0-based)
 *   [2] frag_total — total number of fragments
 *
 * Max payload per fragment = subslot capacity - 3 bytes header
 */

#define FRAG_HEADER_SIZE     3
#define FRAG_MAX_SUBSLOTS    15
#define FRAG_DEFAULT_SUBSLOTS 15

/*
 * Max payload bytes per subslot count at MCS 2 (QPSK 3/4).
 * Approximate: ~14 bytes per subslot.
 */
#define FRAG_BYTES_PER_SUBSLOT 14

/* Max payload per fragment (excluding frag header). */
#define FRAG_MAX_PAYLOAD  ((FRAG_DEFAULT_SUBSLOTS * FRAG_BYTES_PER_SUBSLOT) - FRAG_HEADER_SIZE)

/* Fragment header. */
struct frag_header {
	uint8_t frag_id;
	uint8_t frag_index;
	uint8_t frag_total;
} __attribute__((packed));

/* Reassembly buffer for incoming fragments. */
struct frag_reassembly {
	uint8_t  frag_id;
	uint16_t sender_id;
	uint8_t  frag_total;
	uint8_t  received_mask;  /* bitmask of received fragments (up to 8) */
	uint16_t total_len;
	uint8_t  buf[4096];      /* reassembly buffer */
	bool     active;
};

#define FRAG_REASSEMBLY_SLOTS 4

/* Initialize fragment module. */
void fragment_init(void);

/* Fragment and queue a large buffer for TX.
 * Returns 0 on success, negative on error. */
int fragment_send(const void *data, size_t data_len, uint8_t subslots);

/* Process a received fragment. Returns pointer to complete reassembled
 * data and sets *out_len, or NULL if still incomplete. */
const uint8_t *fragment_receive(const uint8_t *data, size_t data_len,
				uint16_t sender_id, size_t *out_len);

/* Get next fragment ID (auto-incrementing). */
uint8_t fragment_next_id(void);

#endif /* FRAGMENT_H */
