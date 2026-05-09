#include "config.h"
#include "utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ini.h>  /* inih */

void config_set_defaults(odid_config_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    strncpy(cfg->uas_id, "UNSET00000000000000", sizeof(cfg->uas_id) - 1);
    cfg->id_type             = 1;  /* ODID_IDTYPE_SERIAL_NUMBER */
    cfg->ua_type             = 1;  /* ODID_UATYPE_AEROPLANE (fixed wing) */
    strncpy(cfg->operator_id, "UNSET", sizeof(cfg->operator_id) - 1);
    cfg->operator_id_type    = 1;  /* ODID_OPERATORIDTYPE_CAA_REGISTRATION */
    strncpy(cfg->self_id_desc, "Rescue drone", sizeof(cfg->self_id_desc) - 1);
    cfg->self_id_type        = 1;  /* ODID_DESC_TYPE_PURPOSE */
    cfg->op_location_type    = 0;  /* ODID_OPERATOR_LOCATION_TYPE_TAKEOFF */
    cfg->classification_type = 1;  /* ODID_CLASSIFICATION_TYPE_EU */
    cfg->category            = 3;
    cfg->ua_class            = 2;
    strncpy(cfg->serial_device, "/dev/ttyACM0", sizeof(cfg->serial_device) - 1);
    cfg->serial_baud         = 921600;
    cfg->mavlink_use_udp     = false;
    strncpy(cfg->udp_host, "0.0.0.0", sizeof(cfg->udp_host) - 1);
    cfg->udp_port            = 14550;
    cfg->bt4_enabled         = true;
    cfg->bt5_enabled         = true;
    cfg->wifi_beacon_enabled = true;
    strncpy(cfg->wifi_iface, "wlan0", sizeof(cfg->wifi_iface) - 1);
    strncpy(cfg->bt_adapter, "hci0", sizeof(cfg->bt_adapter) - 1);
    /* Default position: Gothenburg harbour area (SSRS base) */
    cfg->default_lat    = 57.7089;
    cfg->default_lon    = 11.9746;
    cfg->default_alt    = 10.0f;
    cfg->default_status = 1;  /* ODID_STATUS_GROUND */
}

/* Strip surrounding quotes from a string value */
static void strip_quotes(char *s) {
    size_t len = strlen(s);
    if (len >= 2 && s[0] == '"' && s[len - 1] == '"') {
        memmove(s, s + 1, len - 2);
        s[len - 2] = '\0';
    }
}

static int odid_ini_handler(void *user, const char *section,
                            const char *name, const char *value) {
    odid_config_t *cfg = (odid_config_t *)user;

#define MATCH(s, n) (strcmp(section, s) == 0 && strcmp(name, n) == 0)
#define SET_STR(field) do { \
    char _tmp[256]; \
    strncpy(_tmp, value, sizeof(_tmp) - 1); _tmp[sizeof(_tmp)-1] = '\0'; \
    strip_quotes(_tmp); \
    strncpy(field, _tmp, sizeof(field) - 1); \
    field[sizeof(field) - 1] = '\0'; \
} while (0)
#define SET_INT(field) (field) = (int)strtol(value, NULL, 0)
#define SET_BOOL(field) (field) = (strcmp(value, "true") == 0 || strcmp(value, "1") == 0)

    if (MATCH("basic_id", "uas_id"))             { SET_STR(cfg->uas_id); }
    else if (MATCH("basic_id", "id_type"))        { SET_INT(cfg->id_type); }
    else if (MATCH("basic_id", "ua_type"))        { SET_INT(cfg->ua_type); }
    else if (MATCH("operator", "operator_id"))    { SET_STR(cfg->operator_id); }
    else if (MATCH("operator", "operator_id_type")){ SET_INT(cfg->operator_id_type); }
    else if (MATCH("self_id", "description"))     { SET_STR(cfg->self_id_desc); }
    else if (MATCH("self_id", "description_type")){ SET_INT(cfg->self_id_type); }
    else if (MATCH("system", "operator_location_type")){ SET_INT(cfg->op_location_type); }
    else if (MATCH("system", "classification_type")){ SET_INT(cfg->classification_type); }
    else if (MATCH("system", "category"))         { SET_INT(cfg->category); }
    else if (MATCH("system", "class"))            { SET_INT(cfg->ua_class); }
    else if (MATCH("serial", "device"))           { SET_STR(cfg->serial_device); }
    else if (MATCH("serial", "baud"))             { SET_INT(cfg->serial_baud); }
    else if (MATCH("mavlink", "source"))          { cfg->mavlink_use_udp = (strcmp(value, "udp") == 0); }
    else if (MATCH("mavlink", "host"))            { SET_STR(cfg->udp_host); }
    else if (MATCH("mavlink", "port"))            { cfg->udp_port = (uint16_t)strtol(value, NULL, 0); }
    else if (MATCH("broadcast", "bt4_enabled"))   { SET_BOOL(cfg->bt4_enabled); }
    else if (MATCH("broadcast", "bt5_enabled"))   { SET_BOOL(cfg->bt5_enabled); }
    else if (MATCH("broadcast", "wifi_beacon_enabled")){ SET_BOOL(cfg->wifi_beacon_enabled); }
    else if (MATCH("broadcast", "wifi_iface"))    { SET_STR(cfg->wifi_iface); }
    else if (MATCH("broadcast", "bt_adapter"))    { SET_STR(cfg->bt_adapter); }
    else if (MATCH("location_defaults", "lat"))   { cfg->default_lat = strtod(value, NULL); }
    else if (MATCH("location_defaults", "lon"))   { cfg->default_lon = strtod(value, NULL); }
    else if (MATCH("location_defaults", "alt"))   { cfg->default_alt = (float)strtod(value, NULL); }
    else if (MATCH("location_defaults", "status")){ SET_INT(cfg->default_status); }
    else {
        LOG_WARN("config: unknown key [%s] %s", section, name);
    }

#undef MATCH
#undef SET_STR
#undef SET_INT
#undef SET_BOOL

    return 1;  /* success */
}

int config_load(const char *path, odid_config_t *cfg) {
    int ret = ini_parse(path, odid_ini_handler, cfg);
    if (ret < 0) {
        LOG_ERROR("config: cannot open '%s': %s", path, strerror(errno));
        return -1;
    }
    if (ret > 0) {
        LOG_ERROR("config: parse error in '%s' at line %d", path, ret);
        return -1;
    }
    LOG_INFO("config: loaded '%s'", path);
    return 0;
}

void config_apply_env(odid_config_t *cfg) {
    const char *e;
#define ENV_STR(var, field) \
    if ((e = getenv(var))) { strncpy(field, e, sizeof(field) - 1); field[sizeof(field)-1] = '\0'; }
#define ENV_INT(var, field) \
    if ((e = getenv(var))) { (field) = (int)strtol(e, NULL, 0); }
#define ENV_BOOL(var, field) \
    if ((e = getenv(var))) { (field) = (strcmp(e, "true") == 0 || strcmp(e, "1") == 0); }

    ENV_STR ("ODID_UAS_ID",            cfg->uas_id)
    ENV_INT ("ODID_ID_TYPE",           cfg->id_type)
    ENV_INT ("ODID_UA_TYPE",           cfg->ua_type)
    ENV_STR ("ODID_OPERATOR_ID",       cfg->operator_id)
    ENV_INT ("ODID_OPERATOR_ID_TYPE",  cfg->operator_id_type)
    ENV_STR ("ODID_SELF_ID",           cfg->self_id_desc)
    ENV_INT ("ODID_SELF_ID_TYPE",      cfg->self_id_type)
    ENV_STR ("ODID_SERIAL_DEVICE",     cfg->serial_device)
    ENV_INT ("ODID_SERIAL_BAUD",       cfg->serial_baud)
    if ((e = getenv("ODID_MAVLINK_SOURCE")))
        cfg->mavlink_use_udp = (strcmp(e, "udp") == 0);
    ENV_STR ("ODID_UDP_HOST",          cfg->udp_host)
    if ((e = getenv("ODID_UDP_PORT"))) cfg->udp_port = (uint16_t)strtol(e, NULL, 0);
    ENV_BOOL("ODID_BT4_ENABLED",       cfg->bt4_enabled)
    ENV_BOOL("ODID_BT5_ENABLED",       cfg->bt5_enabled)
    ENV_BOOL("ODID_WIFI_BEACON_ENABLED", cfg->wifi_beacon_enabled)
    ENV_STR ("ODID_WIFI_IFACE",        cfg->wifi_iface)
    ENV_STR ("ODID_BT_ADAPTER",        cfg->bt_adapter)
    if ((e = getenv("ODID_DEFAULT_LAT"))) cfg->default_lat = strtod(e, NULL);
    if ((e = getenv("ODID_DEFAULT_LON"))) cfg->default_lon = strtod(e, NULL);
    if ((e = getenv("ODID_DEFAULT_ALT"))) cfg->default_alt = (float)strtod(e, NULL);

#undef ENV_STR
#undef ENV_INT
#undef ENV_BOOL
}

void config_dump(const odid_config_t *cfg) {
    LOG_INFO("config: uas_id='%s' id_type=%d ua_type=%d",
             cfg->uas_id, cfg->id_type, cfg->ua_type);
    LOG_INFO("config: operator_id='%s' type=%d",
             cfg->operator_id, cfg->operator_id_type);
    LOG_INFO("config: self_id='%s' type=%d",
             cfg->self_id_desc, cfg->self_id_type);
    if (cfg->mavlink_use_udp)
        LOG_INFO("config: mavlink=udp host=%s port=%u", cfg->udp_host, cfg->udp_port);
    else
        LOG_INFO("config: serial=%s baud=%d", cfg->serial_device, cfg->serial_baud);
    LOG_INFO("config: bt4=%d bt5=%d wifi=%d iface=%s adapter=%s",
             cfg->bt4_enabled, cfg->bt5_enabled,
             cfg->wifi_beacon_enabled, cfg->wifi_iface, cfg->bt_adapter);
}
