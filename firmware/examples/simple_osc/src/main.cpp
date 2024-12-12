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

// Wifi Password
// Define custom password in ssid_config.h
#if __has_include("ssid_config.h")
# include "ssid_config.h"
#else
    #define WIFI_SSID "tstick-network"
    #define WIFI_PSK  "mappings"
#endif
// static K_SEM_DEFINE(wifi_connected, 0, 1);
// static K_SEM_DEFINE(ipv4_address_obtained, 0, 1);

// static struct net_mgmt_event_callback wifi_cb;
// static struct net_mgmt_event_callback ipv4_cb;
// int scan_result = 0;

// static void handle_wifi_connect_result(struct net_mgmt_event_callback *cb)
// {
//     const struct wifi_status *status = (const struct wifi_status *)cb->info;

//     if (status->status)
//     {
//         printk("Connection request failed (%d)\n", status->status);
//     }
//     else
//     {
//         printk("Connected\n");
//         k_sem_give(&wifi_connected);
//     }
// }

// static void handle_wifi_disconnect_result(struct net_mgmt_event_callback *cb)
// {
//     const struct wifi_status *status = (const struct wifi_status *)cb->info;

//     if (status->status)
//     {
//         printk("Disconnection request (%d)\n", status->status);
//     }
//     else
//     {
//         printk("Disconnected\n");
//         k_sem_take(&wifi_connected, K_NO_WAIT);
//     }
// }

// static void handle_ipv4_result(struct net_if *iface)
// {
//     int i = 0;

//     for (i = 0; i < NET_IF_MAX_IPV4_ADDR; i++) {

//         char buf[NET_IPV4_ADDR_LEN];

//         if (iface->config.ip.ipv4->unicast[i].ipv4.addr_type != NET_ADDR_DHCP) {
//             continue;
//         }

//         printk("IPv4 address: %s\n",
//                 net_addr_ntop(AF_INET,
//                                 &iface->config.ip.ipv4->unicast[i].ipv4.address.in_addr,
//                                 buf, sizeof(buf)));
//         printk("Subnet: %s\n",
//                 net_addr_ntop(AF_INET,
//                                 &iface->config.ip.ipv4->unicast[i].netmask,
//                                 buf, sizeof(buf)));
//         printk("Router: %s\n",
//                 net_addr_ntop(AF_INET,
//                                 &iface->config.ip.ipv4->gw,
//                                 buf, sizeof(buf)));
//         }

//         k_sem_give(&ipv4_address_obtained);
// }

// static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb, uint32_t mgmt_event, struct net_if *iface)
// {
//     switch (mgmt_event)
//     {

//         case NET_EVENT_WIFI_CONNECT_RESULT:
//             handle_wifi_connect_result(cb);
//             break;

//         case NET_EVENT_WIFI_DISCONNECT_RESULT:
//             handle_wifi_disconnect_result(cb);
//             break;

//         case NET_EVENT_IPV4_ADDR_ADD:
//             handle_ipv4_result(iface);
//             break;

//         default:
//             break;
//     }
// }

// void wifi_connect(void)
// {
//     struct net_if *iface = net_if_get_wifi_sta();

//     struct wifi_connect_req_params wifi_params = {0};

//     // Set default parameters
//     wifi_params.band = WIFI_FREQ_BAND_UNKNOWN;
//     wifi_params.channel = WIFI_CHANNEL_ANY;
//     wifi_params.security = WIFI_SECURITY_TYPE_NONE;
//     wifi_params.mfp = WIFI_MFP_OPTIONAL;
//     wifi_params.eap_ver = 1;
//     wifi_params.ignore_broadcast_ssid = 0;

// 	// Save ssid and password
// 	wifi_params.ssid = (const uint8_t *)WIFI_SSID;
// 	wifi_params.psk = (const uint8_t *)WIFI_PSK;
//     wifi_params.ssid_length = strlen(WIFI_SSID);
//     wifi_params.psk_length = strlen(WIFI_PSK);

//     // Attempt to connect to ssid
//     printk("Connecting to SSID: %s\n", wifi_params.ssid);
//     if (net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &wifi_params, sizeof(struct wifi_connect_req_params)))
//     {
//         printk("WiFi Connection Request Failed\n");
//     }
// }

// void wifi_status(void)
// {
//     struct net_if *iface = net_if_get_wifi_sta();
    
//     struct wifi_iface_status status = {0};

//     if (net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface, &status,	sizeof(struct wifi_iface_status)))
//     {
//         printk("WiFi Status Request Failed\n");
//     }

//     printk("\n");

//     if (status.state >= WIFI_STATE_ASSOCIATED) {
//         printk("SSID: %-32s\n", status.ssid);
//         printk("Band: %s\n", wifi_band_txt(status.band));
//         printk("Channel: %d\n", status.channel);
//         printk("Security: %s\n", wifi_security_txt(status.security));
//         printk("RSSI: %d\n", status.rssi);
//     }
// }

// void wifi_disconnect(void)
// {
//     struct net_if *iface = net_if_get_default();

//     if (net_mgmt(NET_REQUEST_WIFI_DISCONNECT, iface, NULL, 0))
//     {
//         printk("WiFi Disconnection Request Failed\n");
//     }
// }

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
#define TSTICK_SIZE 120
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
    osc_bundle_add_int_array(bundle, "instrument/touch/discrete", TSTICK_SIZE, sensors.mergeddiscretetouch);
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
#define OSC_RATE_US 100
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
        printk("FAILED LED SETUP\n");
        return;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        printk("FAILED LED CONFIGURATION\n");
        return;
    }
    printk("Configured LED\n");

    printk("Initialising OSC-IP1  ... \n");
    osc1 = lo_address_new("192.168.86.230", "8000");
    printk("Configured OSC  ... \n");

    // Wait a bit
    k_msleep(500);
    printk("Starting Sending OSC messages\n");

    while(1) {
        // Counter
		sensors.counter++;

		// Only send if network was properly configured
        start = k_uptime_ticks();
        updateOSC();
        end = k_uptime_ticks();
        sensors.looptime = end-start;

        if ((k_uptime_get_32() - last_time) > SLEEP_TIME_MS) {
            ret = gpio_pin_toggle_dt(&led);
            led_state = !led_state;
            last_time = k_uptime_get_32();
        }

        // Sleep thread for a bit
        k_usleep(OSC_RATE_US);
    }
}

// Create main thread
#define MY_STACK_SIZE 8192
#define MY_PRIORITY 5
K_THREAD_DEFINE(my_tid, MY_STACK_SIZE,
                osc_loop, NULL, NULL, NULL,
                MY_PRIORITY, 0, 1000);

/********* MAIN **********/
int main(void)
{
    // Initialise WiFi
	// // // Loop indefinitely
    return 0;
}
