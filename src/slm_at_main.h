/*
+------------------------------------------------------------------------------+
| IRISS Inc.
| -----------------------------------------------------------------------------
| Copyright 2023-2025 (c) IRISS Inc. All rights reserved.
+------------------------------------------------------------------------------+
*/
/**
 *  @file    slm_at_main.h
 *  @brief   Main entry points for the SLM AT command processor (server side).
 *           Modelled on h745-zephyr-gateway/src/slm_at_main.{c,h} but with the
 *           direction inverted: we receive AT commands and emit responses
 *           instead of sending commands to an external module.
 */
#ifndef SLM_AT_MAIN_H_
#define SLM_AT_MAIN_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialize the AT command processor (brings up the UART transport).
 * @return 0 on success, otherwise a negative error code.
 */
int slm_at_init(void);

/**
 * @brief  Drain any queued RX message, parse it, dispatch to a handler,
 *         and emit the response. Call this periodically from each device's
 *         main loop (matches h745's slm_at_run_cycle() pattern).
 */
void slm_at_run_cycle(void);

#ifdef __cplusplus
}
#endif

#endif /* SLM_AT_MAIN_H_ */
