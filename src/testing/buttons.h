#ifndef BUTTONS_H
#define BUTTONS_H

/*
 * Runtime DK button handlers (button1 and button2 on the nrf9151dk).
 *
 * Wiring: both buttons are active-low with pull-up (devicetree flags
 *         GPIO_ACTIVE_LOW | GPIO_PULL_UP).
 *
 * Trigger: a single press fires on release (after debounce). No
 *          long-press / hold support — for that, see factory_reset.c.
 *
 * Usage:
 *   button_register_handler(1, my_button1_handler);
 *   button_register_handler(2, my_button2_handler);
 *   buttons_init();   // start watching
 *
 * Handlers run in the buttons thread context, not ISR — safe to log,
 * call kernel APIs, allocate from the heap, etc.
 */

typedef void (*button_handler_t)(void);

/* Register a handler for buttonN (idx = 1 or 2). NULL unregisters.
 * May be called before or after buttons_init(). */
int button_register_handler(int idx, button_handler_t handler);

/* Configure GPIOs, attach interrupts, start watcher thread.
 * Call once at boot. */
int buttons_init(void);

#endif /* BUTTONS_H */
