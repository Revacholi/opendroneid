/*
 * odid_state.c
 *
 * ODID state machine:
 *  - Maintains the 5 active ODID data structs (BasicID, Location, System,
 *    OperatorID, SelfID).
 *  - Merges config defaults with live MAVLink data (MAVLink wins when set).
 *  - Schedules broadcasts: Location @ 1 Hz, all others @ 0.33 Hz.
 *  - Encodes messages with opendroneid-core-c and dispatches to registered
 *    broadcast callbacks.
 */

#include "odid_state.h"
#include "utils.h"

#include <string.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>

/* MAVLink ODID structs (for decoding raw bytes from the queue) */
#include <ardupilotmega/mavlink.h>

/* opendroneid-core-c */
#include <opendroneid.h>


#define MAX_BROADCASTS 4

static struct {
    pthread_mutex_t mu;

    /* ODID data structs populated from config + MAVLink */
    ODID_BasicID_data    basic_id;
    ODID_Location_data   location;
    ODID_System_data     system;
    ODID_OperatorID_data operator_id;
    ODID_SelfID_data     self_id;

    /* Freshness flags: set when MAVLink has provided a value at least once */
    bool has_location;
    bool has_system;
    bool has_basic_id;
    bool has_operator_id;
    bool has_self_id;

    bool armed;
    bool has_heartbeat;  /* false until first HEARTBEAT received */
    bool no_arm_check;
    bool running;

    /* Broadcast callbacks */
    struct {
        odid_broadcast_fn fn;
        void *user;
    } broadcasts[MAX_BROADCASTS];
    int n_broadcasts;
} s;


static void dispatch_all(bool location_only) {
    if (s.n_broadcasts == 0) return;

    /* Encode each individual message */
    ODID_UAS_Data uas;
    memset(&uas, 0, sizeof(uas));
    uas.BasicIDValid[0]   = 1;
    uas.LocationValid     = 1;
    uas.SystemValid       = 1;
    uas.OperatorIDValid   = 1;
    uas.SelfIDValid       = 1;
    uas.BasicID[0]        = s.basic_id;
    uas.Location          = s.location;
    uas.System            = s.system;
    uas.OperatorID        = s.operator_id;
    uas.SelfID            = s.self_id;

    ODID_MessagePack_encoded pack_encoded;
    ODID_MessagePack_data pack_data;
    memset(&pack_data, 0, sizeof(pack_data));
    memset(&pack_encoded, 0, sizeof(pack_encoded));

    /* Encode individual messages into the pack */
    ODID_BasicID_encoded    enc_basic;
    ODID_Location_encoded   enc_loc;
    ODID_System_encoded     enc_sys;
    ODID_OperatorID_encoded enc_op;
    ODID_SelfID_encoded     enc_self;

    if (encodeBasicIDMessage(&enc_basic, &s.basic_id) != ODID_SUCCESS)
        LOG_WARN("odid_state: encodeBasicID failed");
    if (encodeLocationMessage(&enc_loc, &s.location) != ODID_SUCCESS)
        LOG_WARN("odid_state: encodeLocation failed");
    if (encodeSystemMessage(&enc_sys, &s.system) != ODID_SUCCESS)
        LOG_WARN("odid_state: encodeSystem failed");
    if (encodeOperatorIDMessage(&enc_op, &s.operator_id) != ODID_SUCCESS)
        LOG_WARN("odid_state: encodeOperatorID failed");
    if (encodeSelfIDMessage(&enc_self, &s.self_id) != ODID_SUCCESS)
        LOG_WARN("odid_state: encodeSelfID failed");

    /* Build single encoded messages for BT4 (Location most critical) */
    odid_encoded_t single_loc;
    memcpy(single_loc.data, &enc_loc, ODID_MSG_SIZE);

    /* Build message pack for BT5/WiFi */
    /* MsgPackSize is set after adding messages */
    uint8_t n = 0;
    memcpy(pack_data.Messages[n].rawData, &enc_basic, ODID_MESSAGE_SIZE); n++;
    memcpy(pack_data.Messages[n].rawData, &enc_loc,   ODID_MESSAGE_SIZE); n++;
    memcpy(pack_data.Messages[n].rawData, &enc_sys,   ODID_MESSAGE_SIZE); n++;
    memcpy(pack_data.Messages[n].rawData, &enc_op,    ODID_MESSAGE_SIZE); n++;
    memcpy(pack_data.Messages[n].rawData, &enc_self,  ODID_MESSAGE_SIZE); n++;
    pack_data.SingleMessageSize = ODID_MESSAGE_SIZE;
    pack_data.MsgPackSize = n;

    if (encodeMessagePack(&pack_encoded, &pack_data) != ODID_SUCCESS)
        LOG_WARN("odid_state: encodeMessagePack failed");

    odid_pack_t pack;
    pack.count = n;
    for (int i = 0; i < n; i++)
        memcpy(pack.msgs[i].data,
               pack_data.Messages[i].rawData, ODID_MESSAGE_SIZE);

    for (int i = 0; i < s.n_broadcasts; i++)
        s.broadcasts[i].fn(&single_loc, &pack, location_only, s.broadcasts[i].user);
}


void odid_state_init(const odid_config_t *cfg, bool no_arm_check) {
    memset(&s, 0, sizeof(s));
    pthread_mutex_init(&s.mu, NULL);
    s.no_arm_check = no_arm_check;
    s.running = true;

    /* Populate BasicID from config */
    s.basic_id.IDType = cfg->id_type;
    s.basic_id.UAType = cfg->ua_type;
    strncpy(s.basic_id.UASID, cfg->uas_id, sizeof(s.basic_id.UASID) - 1);
    s.basic_id.UASID[sizeof(s.basic_id.UASID) - 1] = '\0';

    /* Populate OperatorID from config */
    s.operator_id.OperatorIdType = cfg->operator_id_type;
    strncpy(s.operator_id.OperatorId, cfg->operator_id,
            sizeof(s.operator_id.OperatorId) - 1);
    s.operator_id.OperatorId[sizeof(s.operator_id.OperatorId) - 1] = '\0';

    /* Populate SelfID from config */
    s.self_id.DescType = cfg->self_id_type;
    strncpy(s.self_id.Desc, cfg->self_id_desc, sizeof(s.self_id.Desc) - 1);
    s.self_id.Desc[sizeof(s.self_id.Desc) - 1] = '\0';

    /* Populate System from config */
    s.system.OperatorLocationType = cfg->op_location_type;
    s.system.ClassificationType   = cfg->classification_type;
    s.system.CategoryEU           = cfg->category;
    s.system.ClassEU              = cfg->ua_class;
    /* Seed operator location from config defaults so we never broadcast 0,0.
     * For TAKEOFF type this is updated from GPS_INT until the drone arms. */
    s.system.OperatorLatitude     = cfg->default_lat;
    s.system.OperatorLongitude    = cfg->default_lon;
    s.system.OperatorAltitudeGeo  = cfg->default_alt;
    s.system.AreaCount            = 1;  /* spec minimum: 1 = no sub-areas */
    /* Timestamp: seconds since 1 Jan 2019 00:00:00 UTC (EN 4709-002 epoch) */
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    /* 2019-01-01 00:00:00 UTC = Unix 1546300800 */
    s.system.Timestamp = (float)(now.tv_sec - 1546300800);

    /* Location defaults — use config values until live GPS overrides */
    s.location.Status         = cfg->default_status;
    s.location.Latitude       = cfg->default_lat;
    s.location.Longitude      = cfg->default_lon;
    s.location.AltitudeGeo    = cfg->default_alt;
    s.location.AltitudeBaro   = cfg->default_alt;
    s.location.HeightType     = ODID_HEIGHT_REF_OVER_TAKEOFF;
    s.location.HorizAccuracy  = ODID_HOR_ACC_UNKNOWN;
    s.location.VertAccuracy   = ODID_VER_ACC_UNKNOWN;
    s.location.BaroAccuracy   = ODID_VER_ACC_UNKNOWN;
    s.location.SpeedAccuracy  = ODID_SPEED_ACC_UNKNOWN;
    s.location.TSAccuracy     = ODID_TIME_ACC_UNKNOWN;

    LOG_INFO("odid_state: initialised from config");
}

void odid_state_add_broadcast(odid_broadcast_fn fn, void *user) {
    if (s.n_broadcasts >= MAX_BROADCASTS) {
        LOG_WARN("odid_state: max broadcasts reached");
        return;
    }
    s.broadcasts[s.n_broadcasts].fn   = fn;
    s.broadcasts[s.n_broadcasts].user = user;
    s.n_broadcasts++;
}

void odid_state_update(const odid_queue_msg_t *msg) {
    pthread_mutex_lock(&s.mu);

    switch (msg->type) {

    case ODID_MSG_BASIC_ID: {
        mavlink_open_drone_id_basic_id_t m;
        memcpy(&m, msg->raw, sizeof(m));
        if (m.id_type != 0) {
            s.basic_id.IDType = m.id_type;
            s.basic_id.UAType = m.ua_type;
            /* MAVLink UASID is 20 bytes */
            memcpy(s.basic_id.UASID, m.uas_id,
                   sizeof(s.basic_id.UASID) < sizeof(m.uas_id)
                   ? sizeof(s.basic_id.UASID) : sizeof(m.uas_id));
            s.has_basic_id = true;
        }
        break;
    }

    case ODID_MSG_LOCATION: {
        mavlink_open_drone_id_location_t m;
        memcpy(&m, msg->raw, sizeof(m));
        /* Convert MAVLink units to opendroneid-core-c units */
        s.location.Latitude         = m.latitude  / 1e7;  /* deg * 1e7 → deg */
        s.location.Longitude        = m.longitude / 1e7;
        s.location.AltitudeBaro     = m.altitude_barometric;  /* m */
        s.location.AltitudeGeo      = m.altitude_geodetic;    /* m */
        s.location.Height           = m.height;               /* m */
        s.location.HeightType       = m.height_reference;
        s.location.Direction        = m.direction / 100.0f;   /* cdeg → deg */
        s.location.SpeedHorizontal  = m.speed_horizontal / 100.0f; /* cm/s → m/s */
        s.location.SpeedVertical    = m.speed_vertical   / 100.0f;
        s.location.Status           = m.status;
        s.location.HorizAccuracy    = m.horizontal_accuracy;
        s.location.VertAccuracy     = m.vertical_accuracy;
        s.location.BaroAccuracy     = m.barometer_accuracy;
        s.location.SpeedAccuracy    = m.speed_accuracy;
        s.location.TSAccuracy       = m.timestamp_accuracy;
        s.location.TimeStamp        = m.timestamp;
        s.has_location = true;
        break;
    }

    case ODID_MSG_GPS_INT: {
        /* Only use as fallback when no OPEN_DRONE_ID_LOCATION received */
        if (!s.has_location) {
            s.location.Latitude        = msg->gps.lat / 1e7;
            s.location.Longitude       = msg->gps.lon / 1e7;
            s.location.AltitudeGeo     = msg->gps.alt / 1000.0f;  /* mm → m */
            s.location.Height          = msg->gps.relative_alt / 1000.0f;
            float vx = msg->gps.vx / 100.0f;
            float vy = msg->gps.vy / 100.0f;
            s.location.SpeedHorizontal = sqrtf(vx*vx + vy*vy);
            s.location.SpeedVertical   = msg->gps.vz / 100.0f;
            if (msg->gps.hdg != 36000)  /* 36000 = unknown */
                s.location.Direction   = msg->gps.hdg / 100.0f;
            /* Update timestamp */
            struct timespec now;
            clock_gettime(CLOCK_REALTIME, &now);
            s.location.TimeStamp = fmodf((float)(now.tv_sec % 3600)
                                         + now.tv_nsec / 1e9f, 3600.0f);
        }
        /* Track operator (takeoff) location from GPS while on the ground.
         * Frozen once armed so TAKEOFF semantics are preserved.
         * Ignore lat=0,lon=0 — that is a no-fix sentinel, not a real location. */
        if (!s.armed && !s.has_system &&
            (msg->gps.lat != 0 || msg->gps.lon != 0)) {
            s.system.OperatorLatitude    = msg->gps.lat / 1e7;
            s.system.OperatorLongitude   = msg->gps.lon / 1e7;
            s.system.OperatorAltitudeGeo = msg->gps.alt / 1000.0f;
            LOG_DEBUG("odid_state: op_loc from GPS_INT lat=%.6f lon=%.6f",
                      s.system.OperatorLatitude, s.system.OperatorLongitude);
        }
        break;
    }

    case ODID_MSG_SYSTEM: {
        mavlink_open_drone_id_system_t m;
        memcpy(&m, msg->raw, sizeof(m));
        /* ArduPilot sends SYSTEM with operator_lat/lon = 0 when no GCS is
         * connected. Only trust non-zero coordinates — otherwise the GPS_INT
         * takeoff-tracking path would be permanently blocked by a bogus 0,0. */
        if (m.operator_latitude != 0 || m.operator_longitude != 0) {
            s.system.OperatorLatitude     = m.operator_latitude  / 1e7;
            s.system.OperatorLongitude    = m.operator_longitude / 1e7;
            s.system.OperatorAltitudeGeo  = m.operator_altitude_geo;
            s.system.OperatorLocationType = m.operator_location_type;
            s.has_system = true;
            LOG_DEBUG("odid_state: SYSTEM op_lat=%.6f op_lon=%.6f",
                      s.system.OperatorLatitude, s.system.OperatorLongitude);
        } else {
            LOG_DEBUG("odid_state: SYSTEM op_lat=0 — ignoring zero operator location");
        }
        s.system.ClassificationType   = m.classification_type;
        s.system.CategoryEU           = m.category_eu;
        s.system.ClassEU              = m.class_eu;
        s.system.AreaCount            = m.area_count;
        s.system.Timestamp            = m.timestamp;
        break;
    }

    case ODID_MSG_OPERATOR_ID: {
        mavlink_open_drone_id_operator_id_t m;
        memcpy(&m, msg->raw, sizeof(m));
        s.operator_id.OperatorIdType = m.operator_id_type;
        memcpy(s.operator_id.OperatorId, m.operator_id,
               sizeof(s.operator_id.OperatorId) < sizeof(m.operator_id)
               ? sizeof(s.operator_id.OperatorId) : sizeof(m.operator_id));
        s.has_operator_id = true;
        break;
    }

    case ODID_MSG_SELF_ID: {
        mavlink_open_drone_id_self_id_t m;
        memcpy(&m, msg->raw, sizeof(m));
        s.self_id.DescType = m.description_type;
        memcpy(s.self_id.Desc, m.description,
               sizeof(s.self_id.Desc) < sizeof(m.description)
               ? sizeof(s.self_id.Desc) : sizeof(m.description));
        s.has_self_id = true;
        break;
    }

    case ODID_MSG_HEARTBEAT: {
        bool was_armed = s.armed;
        s.armed = (msg->hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
        s.has_heartbeat = true;
        if (s.armed != was_armed) {
            LOG_INFO("odid_state: drone %s", s.armed ? "ARMED" : "DISARMED");
            /* Only derive Status from arm state when FC isn't sending
             * OPEN_DRONE_ID_LOCATION (which carries its own Status field). */
            if (!s.has_location)
                s.location.Status = s.armed ? ODID_STATUS_AIRBORNE
                                            : ODID_STATUS_GROUND;
        }
        break;
    }
    }

    pthread_mutex_unlock(&s.mu);
}

void odid_state_run(void) {
    long long last_location = 0;
    long long last_slow     = 0;

    const long long LOCATION_INTERVAL_MS = 1000;
    const long long SLOW_INTERVAL_MS     = 5000;

    LOG_INFO("odid_state: scheduler running");

    while (s.running) {
        long long now = time_ms();

        bool do_location = (now - last_location) >= LOCATION_INTERVAL_MS;
        bool do_slow     = (now - last_slow)     >= SLOW_INTERVAL_MS;

        if (do_location) {
            pthread_mutex_lock(&s.mu);
            bool active = s.no_arm_check || !s.has_heartbeat || s.armed;
            if (active) {
                struct timespec ts;
                clock_gettime(CLOCK_REALTIME, &ts);
                s.system.Timestamp = (float)(ts.tv_sec - 1546300800);
                s.location.TimeStamp = fmodf((float)(ts.tv_sec % 3600)
                                             + ts.tv_nsec / 1e9f, 3600.0f);
                dispatch_all(true);
            }
            pthread_mutex_unlock(&s.mu);
            last_location = now;
        }

        if (do_slow) {
            pthread_mutex_lock(&s.mu);
            bool active = s.no_arm_check || !s.has_heartbeat || s.armed;
            if (active)
                dispatch_all(false);
            pthread_mutex_unlock(&s.mu);
            last_slow = now;
        }

        usleep(50000);  /* 50 ms tick */
    }

    LOG_INFO("odid_state: scheduler stopped");
}

void odid_state_stop(void) {
    s.running = false;
}

bool odid_state_armed(void) {
    return s.armed;
}
