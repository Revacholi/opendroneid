/*
 * main.c — Open Drone ID Daemon
 *
 * Entry point for the ODID broadcaster.
 * Wires together: config → MAVLink reader → ODID state machine
 *                         → Bluetooth (BT4 + BT5 LR) + WiFi beacon
 *
 * Usage: odid-daemon [--config <path>] [--debug] [--no-arm-check]
 *                    [--no-wifi] [--no-bt]
 */

#include "config.h"
#include "mavlink_reader.h"
#include "odid_state.h"
#include "bt_advertiser.h"
#include "wifi_beacon.h"
#include "utils.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>


static volatile sig_atomic_t g_stop = 0;
static bool g_no_wifi = false;
static bool g_no_bt   = false;


static void sig_handler(int sig) {
    (void)sig;
    g_stop = 1;
    odid_state_stop();
    mavlink_reader_stop();
}


static void *mavlink_thread(void *arg) {
    (void)arg;
    odid_queue_msg_t msg;
    while (!g_stop) {
        int r = mavlink_reader_pop(&msg, 200);
        if (r == 1)
            odid_state_update(&msg);
    }
    return NULL;
}


int main(int argc, char *argv[]) {
    const char *config_path = "/etc/odid/odid.conf";

    /* env vars set defaults; CLI args override */
    {
        const char *e;
#define ENV_FLAG(var, field) \
        if ((e = getenv(var)) && (strcmp(e,"1")==0 || strcmp(e,"true")==0)) (field) = true
        if ((e = getenv("ODID_DEBUG")) && (strcmp(e,"1")==0 || strcmp(e,"true")==0))
            g_log_level = LOG_DEBUG;
        ENV_FLAG("ODID_NO_WIFI",      g_no_wifi);
        ENV_FLAG("ODID_NO_BT",        g_no_bt);
#undef ENV_FLAG
    }

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            config_path = argv[++i];
        } else if (strcmp(argv[i], "--debug") == 0) {
            g_log_level = LOG_DEBUG;
        } else if (strcmp(argv[i], "--no-wifi") == 0) {
            g_no_wifi = true;
        } else if (strcmp(argv[i], "--no-bt") == 0) {
            g_no_bt = true;
        } else {
            fprintf(stderr, "Usage: %s [--config PATH] [--debug] [--no-wifi] [--no-bt]\n",
                    argv[0]);
            return 1;
        }
    }

    LOG_INFO("odid-daemon: starting");

    odid_config_t cfg;
    config_set_defaults(&cfg);
    if (access(config_path, R_OK) == 0) {
        if (config_load(config_path, &cfg) < 0) {
            LOG_ERROR("Failed to load config '%s'", config_path);
            return 1;
        }
    } else {
        LOG_WARN("Config file '%s' not found, using defaults", config_path);
    }
    config_apply_env(&cfg);
    if (g_no_wifi) cfg.wifi_beacon_enabled = false;
    if (g_no_bt)   { cfg.bt4_enabled = false; cfg.bt5_enabled = false; }
    config_dump(&cfg);

    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);
    signal(SIGPIPE, SIG_IGN);

    odid_state_init(&cfg);

    bool bt_ok = false;
    if (cfg.bt4_enabled || cfg.bt5_enabled) {
        if (bt_advertiser_init(cfg.bt_adapter, cfg.bt4_enabled, cfg.bt5_enabled) == 0) {
            odid_state_add_broadcast(bt_advertiser_broadcast, NULL);
            bt_ok = true;
        } else {
            LOG_WARN("Bluetooth init failed — continuing without BT broadcast");
        }
    }

    bool wifi_ok = false;
    if (cfg.wifi_beacon_enabled) {
        if (wifi_beacon_init(cfg.wifi_iface) == 0) {
            odid_state_add_broadcast(wifi_beacon_broadcast, NULL);
            wifi_ok = true;
        } else {
            LOG_WARN("WiFi beacon init failed — continuing without WiFi broadcast");
        }
    }

    if (!bt_ok && !wifi_ok) {
        LOG_ERROR("No broadcast channel available — aborting");
        return 1;
    }

    if (cfg.mavlink_use_udp)
        mavlink_reader_init_udp(cfg.udp_host, cfg.udp_port);
    else
        mavlink_reader_init(cfg.serial_device, cfg.serial_baud);
    if (mavlink_reader_start() < 0) {
        LOG_ERROR("MAVLink reader thread failed to start — aborting");
        return 1;
    }

    pthread_t tid;
    pthread_create(&tid, NULL, mavlink_thread, NULL);

    LOG_INFO("odid-daemon: broadcasting on %s%s%s",
             bt_ok   ? "BT4+BT5LR " : "",
             wifi_ok ? "WiFi-Beacon" : "",
             "");

    odid_state_run();  /* returns when odid_state_stop() is called */

    g_stop = 1;
    pthread_join(tid, NULL);
    bt_advertiser_stop();
    wifi_beacon_stop();

    LOG_INFO("odid-daemon: shutdown complete");
    return 0;
}
