#pragma once

#include "odid_state.h"
#include <stdbool.h>

/**
 * Initialise WiFi beacon broadcaster.
 * iface: WiFi interface, e.g. "wlan0"
 * Returns 0 on success, -1 on error.
 */
int wifi_beacon_init(const char *iface);

/**
 * Broadcast callback registered with odid_state_add_broadcast().
 * Injects ODID message pack as vendor IE into beacon frames via nl80211.
 */
void wifi_beacon_broadcast(const odid_encoded_t *single,
                           const odid_pack_t    *pack,
                           bool                  location_only,
                           void                 *user);

/**
 * Stop and clean up.
 */
void wifi_beacon_stop(void);
