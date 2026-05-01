#ifndef SERIAL_H
#define SERIAL_H

/*
 * AT command host interface.
 *
 * UART0 (re-pinned in the board overlay) is dedicated to AT traffic.
 * Logs are routed to RTT so they don't collide with AT responses.
 *
 * Wire format (3GPP-ish):
 *   Host -> device:    AT+CMD?<CR>
 *                      AT+CMD=arg1,arg2<CR>
 *   Device -> host:    <CR><LF>+CMD: value<CR><LF>     (data response)
 *                      <CR><LF>OK<CR><LF>              (terminator)
 *                      <CR><LF>ERROR<CR><LF>           (or on failure)
 *
 * Available on all device types (Gateway / Anchor / Sensor) — the same
 * command set is exposed; behavior differs only where it must (e.g.
 * AT+HOP? returns 0 on a gateway, 0xFF on an unpaired sensor).
 */

/* Initialize the UART, line buffer, and dispatch thread. Returns 0 on
 * success, or a negative errno if the UART device isn't ready. */
int at_command_init(void);

#endif /* SERIAL_H */
