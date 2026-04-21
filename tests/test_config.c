/*
 * test_config.c — Config file parser tests
 */

#include "../src/config.h"
#include "../src/utils.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <stdlib.h>

#define CHECK(cond, msg) do { \
    if (!(cond)) { \
        fprintf(stderr, "FAIL: %s  (%s:%d)\n", msg, __FILE__, __LINE__); \
        return 1; \
    } \
    printf("PASS: %s\n", msg); \
} while (0)

static int test_defaults(void) {
    odid_config_t cfg;
    config_set_defaults(&cfg);
    CHECK(cfg.bt4_enabled  == true, "default bt4_enabled");
    CHECK(cfg.bt5_enabled  == true, "default bt5_enabled");
    CHECK(cfg.serial_baud  == 921600, "default baud");
    CHECK(strcmp(cfg.serial_device, "/dev/ttyACM0") == 0, "default device");
    CHECK(strcmp(cfg.wifi_iface, "wlan0") == 0, "default wifi_iface");
    CHECK(strcmp(cfg.bt_adapter, "hci0") == 0, "default bt_adapter");
    return 0;
}

static int test_load_file(void) {
    /* Write a temp config file */
    const char *path = "/tmp/test_odid.conf";
    FILE *f = fopen(path, "w");
    if (!f) { fprintf(stderr, "cannot create temp file\n"); return 1; }
    fprintf(f,
        "[basic_id]\n"
        "uas_id = \"SSRS-TEST-001\"\n"
        "id_type = 1\n"
        "ua_type = 2\n"
        "[operator]\n"
        "operator_id = \"SWE-OP-99999\"\n"
        "operator_id_type = 1\n"
        "[self_id]\n"
        "description = \"Test drone\"\n"
        "description_type = 1\n"
        "[system]\n"
        "category = 3\n"
        "class = 4\n"
        "[serial]\n"
        "device = \"/dev/ttyUSB0\"\n"
        "baud = 115200\n"
        "[broadcast]\n"
        "bt4_enabled = true\n"
        "bt5_enabled = false\n"
        "wifi_beacon_enabled = true\n"
        "wifi_iface = \"wlan1\"\n"
        "bt_adapter = \"hci1\"\n"
    );
    fclose(f);

    odid_config_t cfg;
    config_set_defaults(&cfg);
    CHECK(config_load(path, &cfg) == 0, "config_load returns 0");

    CHECK(strcmp(cfg.uas_id, "SSRS-TEST-001") == 0, "uas_id parsed");
    CHECK(strcmp(cfg.operator_id, "SWE-OP-99999") == 0, "operator_id parsed");
    CHECK(strcmp(cfg.self_id_desc, "Test drone") == 0, "self_id_desc parsed");
    CHECK(cfg.category == 3, "category parsed");
    CHECK(cfg.ua_class == 4, "class parsed");
    CHECK(strcmp(cfg.serial_device, "/dev/ttyUSB0") == 0, "serial device parsed");
    CHECK(cfg.serial_baud == 115200, "serial baud parsed");
    CHECK(cfg.bt4_enabled  == true, "bt4_enabled parsed");
    CHECK(cfg.bt5_enabled  == false, "bt5_enabled false parsed");
    CHECK(cfg.wifi_beacon_enabled == true, "wifi_beacon_enabled parsed");
    CHECK(strcmp(cfg.wifi_iface, "wlan1") == 0, "wifi_iface parsed");
    CHECK(strcmp(cfg.bt_adapter, "hci1") == 0, "bt_adapter parsed");

    remove(path);
    return 0;
}

int main(void) {
    g_log_level = LOG_ERROR;  /* suppress noise in test output */
    int failures = 0;
    failures += test_defaults();
    failures += test_load_file();

    if (failures == 0)
        printf("\nAll config tests passed.\n");
    else
        printf("\n%d test(s) FAILED.\n", failures);
    return failures ? 1 : 0;
}
