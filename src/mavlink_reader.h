#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <netinet/in.h>

/* Unified message type from the MAVLink reader */
typedef enum {
    ODID_MSG_BASIC_ID    = 0,
    ODID_MSG_LOCATION    = 1,
    ODID_MSG_SYSTEM      = 2,
    ODID_MSG_OPERATOR_ID = 3,
    ODID_MSG_SELF_ID     = 4,
    ODID_MSG_GPS_INT     = 5,   /* GLOBAL_POSITION_INT fallback */
    ODID_MSG_HEARTBEAT   = 6,
} odid_msg_type_t;

/* GLOBAL_POSITION_INT fields we care about */
typedef struct {
    int32_t  lat;          /* deg * 1e7 */
    int32_t  lon;          /* deg * 1e7 */
    int32_t  alt;          /* mm above MSL */
    int32_t  relative_alt; /* mm above home */
    int16_t  vx;           /* cm/s */
    int16_t  vy;           /* cm/s */
    int16_t  vz;           /* cm/s */
    uint16_t hdg;          /* cdeg, 0=N */
} mavlink_gps_int_t;

/* HEARTBEAT arm status (renamed to avoid conflict with MAVLink library typedef) */
typedef struct {
    uint8_t base_mode;     /* MAV_MODE_FLAG bitfield */
    uint8_t system_status; /* MAV_STATE */
} odid_heartbeat_data_t;

/* Raw OPEN_DRONE_ID_* payload bytes (opaque, passed to state machine) */
#define ODID_MAVLINK_PAYLOAD_MAX 255

typedef struct {
    odid_msg_type_t type;
    union {
        uint8_t             raw[ODID_MAVLINK_PAYLOAD_MAX]; /* for ODID msg types */
        mavlink_gps_int_t   gps;
        odid_heartbeat_data_t hb;
    };
    uint16_t raw_len;
} odid_queue_msg_t;

/**
 * Initialise the MAVLink reader for a serial device.
 * Must be called before mavlink_reader_start().
 * Returns 0 on success, -1 on error.
 */
int mavlink_reader_init(const char *device, int baud);

/**
 * Initialise the MAVLink reader to listen on a UDP socket.
 * host is the bind address (e.g. "0.0.0.0"), port is the UDP port.
 * Must be called before mavlink_reader_start().
 * Returns 0 on success, -1 on error.
 */
int mavlink_reader_init_udp(const char *host, uint16_t port);

/**
 * Start the reader thread.  Calls mavlink_reader_init() implicitly if not done.
 */
int mavlink_reader_start(void);

/**
 * Pop one message from the queue (blocking, timeout_ms < 0 = wait forever).
 * Returns 1 if a message was retrieved, 0 on timeout, -1 on error/shutdown.
 */
int mavlink_reader_pop(odid_queue_msg_t *out, int timeout_ms);

/**
 * Signal the reader thread to stop and join it.
 */
void mavlink_reader_stop(void);

/**
 * Returns true if the reader thread is running.
 */
bool mavlink_reader_running(void);
