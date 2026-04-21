#pragma once

#include "config.h"
#include "mavlink_reader.h"
#include <stdbool.h>
#include <stdint.h>

/* Opaque encoded ODID message (25 bytes per EN 4709-002, matches ODID_MESSAGE_SIZE) */
#define ODID_MSG_SIZE  25

typedef struct {
    uint8_t data[ODID_MSG_SIZE];
} odid_encoded_t;

/* Message pack: up to 9 messages for WiFi/BT5 (ODID_PACK_MAX_MESSAGES from core-c) */
#define ODID_PACK_MAX_MSGS 9

typedef struct {
    odid_encoded_t msgs[ODID_PACK_MAX_MSGS];
    uint8_t        count;
} odid_pack_t;

/**
 * Broadcast callback type.
 * Called from the scheduler with the encoded message to broadcast.
 */
typedef void (*odid_broadcast_fn)(const odid_encoded_t *single,
                                  const odid_pack_t    *pack,
                                  bool                  location_only,
                                  void                 *user);

/**
 * Initialise the state machine with config values as defaults.
 * Registers up to 3 broadcast callbacks.
 */
void odid_state_init(const odid_config_t *cfg);
void odid_state_add_broadcast(odid_broadcast_fn fn, void *user);

/**
 * Update state from a MAVLink queue message.
 * Thread-safe (uses internal mutex).
 */
void odid_state_update(const odid_queue_msg_t *msg);

/**
 * Main scheduler loop.  Blocks forever (call from main thread).
 * Calls registered broadcast callbacks at the required rates:
 *   LOCATION:  every 1000 ms
 *   others:    every 3000 ms
 */
void odid_state_run(void);

/**
 * Request the scheduler to exit.
 */
void odid_state_stop(void);

/**
 * Returns true if the drone is currently considered armed.
 */
bool odid_state_armed(void);
