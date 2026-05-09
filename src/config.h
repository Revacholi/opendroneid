#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    /* [basic_id] */
    char    uas_id[21];          /* CTA-2063-A serial number, max 20 chars */
    uint8_t id_type;             /* ODID_IDTYPE_* */
    uint8_t ua_type;             /* ODID_UATYPE_* */

    /* [operator] */
    char    operator_id[21];     /* EU/CAA operator registration number */
    uint8_t operator_id_type;    /* ODID_OPERATORIDTYPE_* */

    /* [self_id] */
    char    self_id_desc[24];    /* Human-readable operation description */
    uint8_t self_id_type;        /* ODID_DESC_TYPE_* */

    /* [system] */
    uint8_t op_location_type;    /* ODID_OPERATOR_LOCATION_TYPE_* */
    uint8_t classification_type; /* ODID_CLASSIFICATION_TYPE_* */
    uint8_t category;            /* EU category */
    uint8_t ua_class;            /* EU class */

    /* [serial] */
    char    serial_device[64];   /* e.g. /dev/ttyACM0 */
    int     serial_baud;         /* e.g. 921600 */

    /* [mavlink] — source selection */
    bool     mavlink_use_udp;    /* false = serial (default), true = UDP */
    char     udp_host[64];       /* bind address, e.g. "0.0.0.0" */
    uint16_t udp_port;           /* UDP port, e.g. 14550 */

    /* [broadcast] */
    bool    bt4_enabled;
    bool    bt5_enabled;
    bool    wifi_beacon_enabled;
    char    wifi_iface[16];      /* e.g. wlan0 */
    char    bt_adapter[8];       /* e.g. hci0 */

    /* [location_defaults] — used until live GPS arrives */
    double  default_lat;         /* degrees, e.g. 57.7089 */
    double  default_lon;         /* degrees, e.g. 11.9746 */
    float   default_alt;         /* metres above WGS-84 */
    uint8_t default_status;      /* ODID_STATUS_GROUND=1 when on ground */
} odid_config_t;

/**
 * Load configuration from an INI file.
 * Returns 0 on success, -1 on error.
 */
int config_load(const char *path, odid_config_t *cfg);

/**
 * Populate cfg with safe defaults before calling config_load.
 */
void config_set_defaults(odid_config_t *cfg);

/**
 * Override config fields from environment variables (ODID_* prefix).
 * Call after config_load so env vars take precedence over the file.
 */
void config_apply_env(odid_config_t *cfg);

/**
 * Print current configuration to stderr for debugging.
 */
void config_dump(const odid_config_t *cfg);
