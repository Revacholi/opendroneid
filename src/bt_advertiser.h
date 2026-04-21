#pragma once

#include "odid_state.h"

/**
 * Initialise Bluetooth advertiser.
 * adapter: HCI adapter name, e.g. "hci0"
 * bt4: enable BT4 legacy advertising
 * bt5: enable BT5 Long Range (extended) advertising
 * Returns 0 on success, -1 on error.
 */
int bt_advertiser_init(const char *adapter, bool bt4, bool bt5);

/**
 * Broadcast callback registered with odid_state_add_broadcast().
 * Sends single message via BT4 and message pack via BT5.
 */
void bt_advertiser_broadcast(const odid_encoded_t *single,
                             const odid_pack_t    *pack,
                             bool                  location_only,
                             void                 *user);

/**
 * Stop advertising and close HCI socket.
 */
void bt_advertiser_stop(void);
