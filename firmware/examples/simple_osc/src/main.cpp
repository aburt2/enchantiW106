/*
 * Copyright (c) 2023, Meta
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Copyright (c) 2024 Muhammad Haziq
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_event.h>
#include <errno.h>

// Liblo headers for OSC
#include <charconv>
#include <lo/lo.h>
#include <lo/lo_lowlevel.h>
#include <lo/lo_types.h>

/* STA Mode Configuration */
// Define custom password in ssid_config.h
#if __has_include("ssid_config.h")
# include "ssid_config.h"
#else
    #define WIFI_SSID "tstick-network"
    #define WIFI_PSK  "mappings"
#endif

LOG_MODULE_REGISTER(MAIN);

/******* Variables to Send *********/
lo_address osc1;
lo_address osc2;
lo_server_thread osc_server;
int counter = 0;
int looptime = 0;
int last_time = 0;
bool wifi_on = false;
int start = 0;
int end = 0;
#define TSTICK_SIZE 60
int test_array[TSTICK_SIZE];

struct Sensors {
    float accl [3];
    float gyro [3];
    float magn [3];
    float quat [4];
    float ypr [3];
    float shake [3];
    float jab [3];
    float brush;
    float rub;
    float multibrush [4];
    float multirub [4];
    int count;
    int tap;
    int dtap;
    int ttap;
    int fsr;
    float squeeze;
    float battery;
    float current;
    float voltage;
    float tte;
    float touchAll;         
    float touchTop;         
    float touchMiddle;      
    float touchBottom;      
    int mergedtouch[TSTICK_SIZE];
    int mergeddiscretetouch[TSTICK_SIZE];
    int counter;
    int looptime;
} sensors;

// OSC Helpers
void error(int num, const char *msg, const char *path);
int generic_handler(const char *path, const char *types, lo_arg ** argv,
                    int argc, lo_message data, void *user_data);
void osc_bundle_add_int(lo_bundle puara_bundle,const char *path, int value);
void osc_bundle_add_float(lo_bundle puara_bundle,const char *path, float value);
void osc_bundle_add_int_array(lo_bundle puara_bundle,const char *path, int size, int *value);
void osc_bundle_add_float_array(lo_bundle puara_bundle,const char *path, int size,  float *value);
void updateOSC();
void updateOSC_bundle(lo_bundle bundle);

void error(int num, const char *msg, const char *path) {
    printf("Liblo server error %d in path %s: %s\n", num, path, msg);
    fflush(stdout);
}

int generic_handler(const char *path, const char *types, lo_arg ** argv,
                    int argc, lo_message data, void *user_data) {
    for (int i = 0; i < argc; i++) {
        printf("arg %d '%c' ", i, types[i]);
        lo_arg_pp((lo_type)types[i], argv[i]);
        printf("\n");
    }
    printf("\n");
    fflush(stdout);

    return 1;
}

void osc_bundle_add_int(lo_bundle puara_bundle,const char *path, int value) {
    int ret = 0;
    lo_message tmp_osc = lo_message_new();
    ret = lo_message_add_int32(tmp_osc, value);
    if (ret < 0) {
        lo_message_free(tmp_osc);
        return;
    }
    ret = lo_bundle_add_message(puara_bundle, path, tmp_osc);
}
void osc_bundle_add_float(lo_bundle puara_bundle,const char *path, float value) {
    int ret = 0;
    lo_message tmp_osc = lo_message_new();
    ret = lo_message_add_float(tmp_osc, value);
    if (ret < 0) {
        lo_message_free(tmp_osc);
        return;
    }
    ret = lo_bundle_add_message(puara_bundle, path, tmp_osc);
}
void osc_bundle_add_int_array(lo_bundle puara_bundle,const char *path, int size, int *value) {
    lo_message tmp_osc = lo_message_new();
    for (int i = 0; i < size; i++) {
        lo_message_add_int32(tmp_osc, value[i]);
    }
    lo_bundle_add_message(puara_bundle, path, tmp_osc);
}

void osc_bundle_add_float_array(lo_bundle puara_bundle,const char *path, int size,  float *value) {
    lo_message tmp_osc = lo_message_new();
    for (int i = 0; i < size; i++) {
        lo_message_add_float(tmp_osc, value[i]);
    }
    lo_bundle_add_message(puara_bundle, path, tmp_osc);
}

void updateOSC() {
    // Create a bundle and send it to both IP addresses
    lo_bundle bundle = lo_bundle_new(LO_TT_IMMEDIATE);
    if (!bundle) {
        return;
    }
    updateOSC_bundle(bundle);
    lo_send_bundle(osc1, bundle);

    // free memory from bundle
    lo_bundle_free_recursive(bundle);
}

void updateOSC_bundle(lo_bundle bundle) {
    // Continuously send FSR data
    osc_bundle_add_int(bundle, "raw/fsr", sensors.fsr);
    osc_bundle_add_float(bundle, "instrument/squeeze", sensors.squeeze);

    //Send touch data
    osc_bundle_add_float(bundle, "instrument/touch/all", sensors.touchAll);
    osc_bundle_add_float(bundle, "instrument/touch/top", sensors.touchTop);
    osc_bundle_add_float(bundle, "instrument/touch/middle", sensors.touchMiddle);
    osc_bundle_add_float(bundle, "instrument/touch/bottom", sensors.touchBottom);
    osc_bundle_add_int_array(bundle, "raw/capsense", TSTICK_SIZE, sensors.mergedtouch);
    // Touch gestures
    osc_bundle_add_float(bundle, "instrument/brush", sensors.brush);
    osc_bundle_add_float_array(bundle, "instrument/multibrush", 4, sensors.multibrush);
    osc_bundle_add_float(bundle, "instrument/rub", sensors.rub);
    osc_bundle_add_float_array(bundle, "instrument/multirub", 4, sensors.multirub);
    
    // MIMU data
    osc_bundle_add_float_array(bundle, "raw/accl", 3, sensors.accl);
    osc_bundle_add_float_array(bundle, "raw/gyro", 3, sensors.gyro);
    osc_bundle_add_float_array(bundle, "raw/magn", 3, sensors.magn);
    osc_bundle_add_float_array(bundle, "orientation", 4, sensors.quat);
    osc_bundle_add_float_array(bundle, "ypr", 3, sensors.ypr); 

    // Inertial gestures
    osc_bundle_add_float_array(bundle, "instrument/shakexyz", 3, sensors.shake);
    osc_bundle_add_float_array(bundle, "instrument/jabxyz", 3, sensors.jab);
    // Button Gestures
    osc_bundle_add_int(bundle, "instrument/button/count", sensors.count);
    osc_bundle_add_int(bundle, "instrument/button/tap", sensors.tap);
    osc_bundle_add_int(bundle, "instrument/button/dtap", sensors.dtap);
    osc_bundle_add_int(bundle, "instrument/button/ttap", sensors.ttap);

    // Battery Data
    osc_bundle_add_float(bundle, "battery/percentage", sensors.battery);
    osc_bundle_add_float(bundle, "battery/current", sensors.current);
    osc_bundle_add_float(bundle, "battery/timetoempty", sensors.tte);
    osc_bundle_add_float(bundle, "battery/voltage", sensors.voltage);  

    // Add counter
    osc_bundle_add_int(bundle, "test/counter", sensors.counter);
    osc_bundle_add_int(bundle, "test/looptime", sensors.looptime);
}

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000
#define BOOT_UP_DELAY_MS 500
#define OSC_RATE_TICKS 10
#define USEC_PER_TICK (1000000 / CONFIG_SYS_CLOCK_TICKS_PER_SEC)
// /* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/* Main loop */
static void osc_loop(void *, void *, void *);

static void osc_loop(void *, void *, void *) {
    int ret;
    bool led_state = true;
    
    if (!gpio_is_ready_dt(&led)) {
        LOG_INF("FAILED LED SETUP\n");
        return;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        LOG_INF("FAILED LED CONFIGURATION\n");
        return;
    }
    LOG_INF("Configured LED");

    LOG_INF("Initialising OSC-IP1  ... ");
    osc1 = lo_address_new("192.168.86.230", "8000");
    LOG_INF("Configured OSC  ...");

    // Wait a bit
    k_msleep(500);
    LOG_INF("Starting Sending OSC messages");

    while(1) {

        // Counter
		sensors.counter++;

		// Only send if network was properly configured
        start = k_uptime_ticks();
        updateOSC();
        end = k_uptime_ticks();
        sensors.looptime = (end-start)*USEC_PER_TICK;

        if ((k_uptime_get_32() - last_time) > SLEEP_TIME_MS) {
            ret = gpio_pin_toggle_dt(&led);
            led_state = !led_state;
            last_time = k_uptime_get_32();
        }

        // Sleep thread for a bit
        
        k_sleep(K_TIMEOUT_ABS_TICKS(OSC_RATE_TICKS));
    }
}

// Create main thread
#define MY_STACK_SIZE 8192
#define MY_PRIORITY 5
K_THREAD_DEFINE(my_tid, MY_STACK_SIZE,
                osc_loop, NULL, NULL, NULL,
                MY_PRIORITY, 0, 5000);

/********* MAIN **********/
int main(void)
{
    return 0;
}
