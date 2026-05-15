#ifndef LOG_COLOR_H
#define LOG_COLOR_H

#include <zephyr/logging/log.h>

/*
 * Colored log macros. Each call expands to a normal LOG_xxx() with ANSI
 * escape codes wrapped around the format string. The trailing reset
 * (\x1b[0m) keeps the terminal from carrying the color into the next
 * line (e.g., a following uncolored timestamp or another module's log).
 *
 * Severity is preserved — LOG_INF_GRN(...) is still LOG_LEVEL_INF, so
 * CONFIG_LOG_*_LEVEL filtering, log forwarding, and any future backends
 * see it as a normal INF event.
 *
 * Use sparingly — pick a color per logical purpose, not per call.
 */

#define LOG_ANSI_RESET   "\x1b[0m"
#define LOG_ANSI_BLACK   "\x1b[30m"
#define LOG_ANSI_RED     "\x1b[31m"
#define LOG_ANSI_GREEN   "\x1b[32m"
#define LOG_ANSI_YELLOW  "\x1b[33m"
#define LOG_ANSI_BLUE    "\x1b[34m"
#define LOG_ANSI_MAGENTA "\x1b[35m"
#define LOG_ANSI_CYAN    "\x1b[36m"
#define LOG_ANSI_WHITE   "\x1b[37m"

/* INF-level coloured variants. */
#define LOG_INF_WHT(fmt, ...) LOG_INF(LOG_ANSI_WHITE   fmt LOG_ANSI_RESET, ##__VA_ARGS__)
#define LOG_INF_RED(fmt, ...) LOG_INF(LOG_ANSI_RED     fmt LOG_ANSI_RESET, ##__VA_ARGS__)
#define LOG_INF_GRN(fmt, ...) LOG_INF(LOG_ANSI_GREEN   fmt LOG_ANSI_RESET, ##__VA_ARGS__)
#define LOG_INF_YEL(fmt, ...) LOG_INF(LOG_ANSI_YELLOW  fmt LOG_ANSI_RESET, ##__VA_ARGS__)
#define LOG_INF_BLU(fmt, ...) LOG_INF(LOG_ANSI_BLUE    fmt LOG_ANSI_RESET, ##__VA_ARGS__)
#define LOG_INF_MAG(fmt, ...) LOG_INF(LOG_ANSI_MAGENTA fmt LOG_ANSI_RESET, ##__VA_ARGS__)
#define LOG_INF_CYAN(fmt, ...) LOG_INF(LOG_ANSI_CYAN   fmt LOG_ANSI_RESET, ##__VA_ARGS__)

/* WRN-level coloured variants — sometimes useful when a warning is
 * semantically also a "good thing happened" (e.g., recovery success). */
#define LOG_WRN_GRN(fmt, ...) LOG_WRN(LOG_ANSI_GREEN   fmt LOG_ANSI_RESET, ##__VA_ARGS__)
#define LOG_WRN_CYAN(fmt, ...) LOG_WRN(LOG_ANSI_CYAN   fmt LOG_ANSI_RESET, ##__VA_ARGS__)

#endif /* LOG_COLOR_H */
