#ifndef LED_H
#define LED_H

/*
 * Runtime status LED handling for the DK (LED1 / devicetree alias led1).
 *
 * Behaviour: a dedicated low-priority thread watches the device's own hop
 * number (product_info get_hop_number()):
 *   - hop_num == 0xFF (no route to gateway) -> LED1 blinks.
 *   - any other hop_num                     -> LED1 stays OFF.
 *
 * For testing / status indication only.
 */

/* Configure the LED1 GPIO and start the status watcher thread.
 * Call once at boot. */
int led_init(void);

#endif /* LED_H */
