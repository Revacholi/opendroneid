#pragma once

#include <stdio.h>
#include <time.h>
#include <errno.h>
#include <string.h>

/* Log levels */
typedef enum {
    LOG_DEBUG = 0,
    LOG_INFO  = 1,
    LOG_WARN  = 2,
    LOG_ERROR = 3,
} log_level_t;

extern log_level_t g_log_level;

static inline const char *log_level_str(log_level_t l) {
    switch (l) {
        case LOG_DEBUG: return "DEBUG";
        case LOG_INFO:  return "INFO ";
        case LOG_WARN:  return "WARN ";
        case LOG_ERROR: return "ERROR";
        default:        return "?????";
    }
}

#define LOG(level, fmt, ...) do { \
    if ((level) >= g_log_level) { \
        struct timespec _ts; \
        clock_gettime(CLOCK_REALTIME, &_ts); \
        fprintf(stderr, "[%ld.%03ld] [%s] " fmt "\n", \
                (long)_ts.tv_sec, (long)(_ts.tv_nsec / 1000000), \
                log_level_str(level), ##__VA_ARGS__); \
    } \
} while (0)

#define LOG_DEBUG(fmt, ...) LOG(LOG_DEBUG, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)  LOG(LOG_INFO,  fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)  LOG(LOG_WARN,  fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) LOG(LOG_ERROR, fmt, ##__VA_ARGS__)

/* Get current time in milliseconds */
static inline long long time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}
