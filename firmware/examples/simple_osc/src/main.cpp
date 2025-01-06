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
#include <zephyr/net/net_if.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>
#include <errno.h>
// Liblo headers for OSC
#include <charconv>
#include <string>
#include "osc.hpp"
#include <lo/lo.h>
#include <lo/lo_lowlevel.h>
#include <lo/lo_types.h>
#include <chrono>
#include <iostream>

// Get Network time
#include <zephyr/net/sntp.h>
#include <sys/time.h>

// Logger
LOG_MODULE_REGISTER(MAIN);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000
#define LOG_RATE_MS     5000
#define BOOT_UP_DELAY_MS 500
#define OSC_RATE_TICKS 10
#define SYNC_TIME_MS 3600
#define USEC_PER_TICK (1000000 / CONFIG_SYS_CLOCK_TICKS_PER_SEC)
// /* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/* STA/AP Mode Configuration */
// Define custom password in ssid_config.h
#if __has_include("ssid_config.h")
# include "ssid_config.h"
#else
    /* STA Mode Configuration */
    #define WIFI_SSID "tstick-network"
    #define WIFI_PSK  "mappings"
    /* AP Mode Configuration */
    #define WIFI_AP_SSID       "TSTick_530"
    #define WIFI_AP_PSK        "mappings"
    #define WIFI_AP_IP_ADDRESS "192.168.4.1"
    #define WIFI_AP_NETMASK    "255.255.255.0"
#endif


// SNTP Settings
#define SNTP_PORT 123
#define SNTP_SERVER "au.pool.ntp.org"

// Networking settings
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/dhcpv4_server.h>
// Network interfaces
static struct net_if *sta_iface;
static struct net_if *ap_iface;

// Parameters
static struct wifi_connect_req_params sta_config;
static struct wifi_connect_req_params ap_config;
static struct net_mgmt_event_callback cb;

// Wifi status
bool wifi_enabled = false;
bool ap_enabled = false;

#define MACSTR "%02X:%02X:%02X:%02X:%02X:%02X"

#define NET_EVENT_WIFI_MASK                                                                        \
	(NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT |                        \
	 NET_EVENT_WIFI_AP_ENABLE_RESULT | NET_EVENT_WIFI_AP_DISABLE_RESULT |                      \
	 NET_EVENT_WIFI_AP_STA_CONNECTED | NET_EVENT_WIFI_AP_STA_DISCONNECTED)


// Functions declarations
static int connect_to_wifi(void);
static void wifi_event_handler(struct net_mgmt_event_callback *cb, uint32_t mgmt_event,
			       struct net_if *iface);


// Function definitions
static int connect_to_wifi(void)
{
	if (!sta_iface) {
		LOG_INF("STA: interface no initialized");
		return -EIO;
	}

	sta_config.ssid = (const uint8_t *)WIFI_SSID;
	sta_config.ssid_length = strlen(WIFI_SSID);
	sta_config.psk = (const uint8_t *)WIFI_PSK;
	sta_config.psk_length = strlen(WIFI_PSK);
	sta_config.security = WIFI_SECURITY_TYPE_PSK;
	sta_config.channel = WIFI_CHANNEL_ANY;
	sta_config.band = WIFI_FREQ_BAND_UNKNOWN;
    sta_config.bandwidth = WIFI_FREQ_BANDWIDTH_20MHZ;
    sta_config.mfp = WIFI_MFP_OPTIONAL;

	LOG_INF("Connecting to SSID: %s\n", sta_config.ssid);

    // Setup command
    int ret = 0;
    // char wifi_shell_cmd[128] =  "wifi connect -s TEST_SSID -p TEST_PSK -k 1 -b 5";
    // sprintf(wifi_shell_cmd, "wifi connect -s %s -p %s -k 1 -b 5", WIFI_SSID, WIFI_PSK);
    // LOG_INF("SHELL CMD: %s", wifi_shell_cmd);
    // ret = shell_execute_cmd(shell_backend_uart_get_ptr(), wifi_shell_cmd);
    
	ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, sta_iface, &sta_config,
			   sizeof(struct wifi_connect_req_params));

	if (ret) {
		LOG_ERR("Unable to Connect to (%s)", WIFI_SSID);
	}

	return ret;
}

static void enable_dhcpv4_server(void)
{
	static struct in_addr addr;
	static struct in_addr netmaskAddr;

	if (net_addr_pton(AF_INET, WIFI_AP_IP_ADDRESS, &addr)) {
		LOG_ERR("Invalid address: %s", WIFI_AP_IP_ADDRESS);
		return;
	}

	if (net_addr_pton(AF_INET, WIFI_AP_NETMASK, &netmaskAddr)) {
		LOG_ERR("Invalid netmask: %s", WIFI_AP_NETMASK);
		return;
	}

	net_if_ipv4_set_gw(ap_iface, &addr);

	if (net_if_ipv4_addr_add(ap_iface, &addr, NET_ADDR_MANUAL, 0) == NULL) {
		LOG_ERR("unable to set IP address for AP interface");
	}

	if (!net_if_ipv4_set_netmask_by_addr(ap_iface, &addr, &netmaskAddr)) {
		LOG_ERR("Unable to set netmask for AP interface: %s", WIFI_AP_NETMASK);
	}

	addr.s4_addr[3] += 10; /* Starting IPv4 address for DHCPv4 address pool. */

	if (net_dhcpv4_server_start(ap_iface, &addr) != 0) {
		LOG_ERR("DHCP server is not started for desired IP");
		return;
	}

	LOG_INF("DHCPv4 server started...\n");
}

static int enable_ap_mode(void)
{
	if (!ap_iface) {
		LOG_INF("AP: is not initialized");
		return -EIO;
	}

	LOG_INF("Turning on AP Mode");
	ap_config.ssid = (const uint8_t *)WIFI_AP_SSID;
	ap_config.ssid_length = strlen(WIFI_AP_SSID);
	ap_config.psk = (const uint8_t *)WIFI_AP_PSK;
	ap_config.psk_length = strlen(WIFI_AP_PSK);
	ap_config.channel = WIFI_CHANNEL_ANY;
	ap_config.band = WIFI_FREQ_BAND_2_4_GHZ;
    ap_config.bandwidth = WIFI_FREQ_BANDWIDTH_20MHZ;

	if (strlen(WIFI_AP_PSK) == 0) {
		ap_config.security = WIFI_SECURITY_TYPE_NONE;
	} else {

		ap_config.security = WIFI_SECURITY_TYPE_PSK;
	}

	enable_dhcpv4_server();

	int ret = net_mgmt(NET_REQUEST_WIFI_AP_ENABLE, ap_iface, &ap_config,
			   sizeof(struct wifi_connect_req_params));
	if (ret) {
		LOG_ERR("NET_REQUEST_WIFI_AP_ENABLE failed, err: %d", ret);
	}

	return ret;
}

static void wifi_event_handler(struct net_mgmt_event_callback *cb, uint32_t mgmt_event,
			       struct net_if *iface)
{
	switch (mgmt_event) {
	case NET_EVENT_WIFI_CONNECT_RESULT: {
		LOG_INF("Connected to %s", WIFI_SSID);
        wifi_enabled = true;
		break;
	}
	case NET_EVENT_WIFI_DISCONNECT_RESULT: {
		LOG_INF("Disconnected from %s", WIFI_SSID);
        wifi_enabled = false;
		break;
	}
	case NET_EVENT_WIFI_AP_ENABLE_RESULT: {
		LOG_INF("AP Mode is enabled. Waiting for station to connect");
        wifi_enabled = false;
		break;
	}
	case NET_EVENT_WIFI_AP_DISABLE_RESULT: {
		LOG_INF("AP Mode is disabled.");
        ap_enabled = false;
		break;
	}
	case NET_EVENT_WIFI_AP_STA_CONNECTED: {
		struct wifi_ap_sta_info *sta_info = (struct wifi_ap_sta_info *)cb->info;

		LOG_INF("station: " MACSTR " joined ", sta_info->mac[0], sta_info->mac[1],
			sta_info->mac[2], sta_info->mac[3], sta_info->mac[4], sta_info->mac[5]);
		break;
	}
	case NET_EVENT_WIFI_AP_STA_DISCONNECTED: {
		struct wifi_ap_sta_info *sta_info = (struct wifi_ap_sta_info *)cb->info;

		LOG_INF("station: " MACSTR " leave ", sta_info->mac[0], sta_info->mac[1],
			sta_info->mac[2], sta_info->mac[3], sta_info->mac[4], sta_info->mac[5]);
		break;
	}
	default:
		break;
	}
}

/******* Variables to Send *********/
lo_address osc1;
lo_address osc2;
oscBundle puara_bundle;
lo_server osc_server;
timeval tv;
uint32_t tv_secs;
double msg_time;
double offset_time;
#define FRAC_TO_SEC 0.00000000023283064365
lo_timetag lo_tt;
lo_timetag lo_offset_tt;
uint32_t last_sync;
uint32_t start_cycle;
int secs_idx;
int bundle_size = 0;
int size_idx;
int counter = 0;
int counter_idx;
int looptime = 0;
int looptime_idx;
int mcu_offset = 0;
int last_time = 0;
int last_log_time = 0;
bool wifi_on = false;
int start = 0;
int end = 0;
#define TEST_SIZE 256
int test_array[TEST_SIZE];
int test_idx;

// OSC Namespace
std::string baseNamespace = "";
std::string oscNamespace;

// OSC Helpers
void error(int num, const char *msg, const char *path);
int generic_handler(const char *path, const char *types, lo_arg ** argv,
                    int argc, lo_message data, void *user_data);
void updateOSC();
void initOSC_bundle();
void updateOSC_bundle();
double timetag_to_double(uint32_t secs, uint32_t frac);
double timeval_to_double(timeval tv);

double timetag_to_double(uint32_t secs, uint32_t frac) {
    return double(secs) + (double(frac) * FRAC_TO_SEC);
}

double timeval_to_double(timeval tv) {
    return double(tv.tv_sec) + ((double(tv.tv_usec) * 0.000001));
}

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

void updateOSC() {
    // Create a bundle and send it to both IP addresses
    updateOSC_bundle();

    if (wifi_enabled && !ap_enabled) {
        puara_bundle.fast_send(osc1, osc_server);
    }
}

void initOSC_bundle() {
    puara_bundle.add(&counter_idx, "test/counter",  counter);
    puara_bundle.add(&looptime_idx, "test/looptime", looptime);
    puara_bundle.add(&size_idx, "test/bundlelength", bundle_size);
    puara_bundle.add(&secs_idx, "test/secs", msg_time);
    puara_bundle.add(&test_idx, "test_array", TEST_SIZE, test_array);
}

void updateOSC_bundle() {
    puara_bundle.update_message(counter_idx,  counter);
    puara_bundle.update_message(looptime_idx, looptime);
    puara_bundle.update_message(size_idx, bundle_size);
    puara_bundle.update_message(secs_idx, msg_time);
    puara_bundle.update_message(test_idx, TEST_SIZE, test_array);
}

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/********* MAIN **********/
int main(void)
{
    // Wait for wifi network manager to be initialised
    k_msleep(5000);
    
    // Adding network call back events
    net_mgmt_init_event_callback(&cb, wifi_event_handler, NET_EVENT_WIFI_MASK);
	net_mgmt_add_event_callback(&cb);

    // Wait for iface to be initialised
    sta_iface = net_if_get_wifi_sta();
    while (!sta_iface) {
        LOG_INF("STA: is not initialized");
        sta_iface = net_if_get_wifi_sta();
    }
    LOG_INF("STA: is initialized");

    // Wait for ap to be initialised
    ap_iface = net_if_get_wifi_sap();
    while (!ap_iface) {
        LOG_INF("AP: is not initialized");
        ap_iface = net_if_get_wifi_sap();
    }
    LOG_INF("AP: is initialized");

    // Connect to wifi
    connect_to_wifi();
    // enable_ap_mode();

    // Start OSC thread
    while (!wifi_enabled) {
        k_msleep(500);
    }
    LOG_INF("Connect to Wifi");

        int ret;
    bool led_state = true;
    
    if (!gpio_is_ready_dt(&led)) {
        LOG_INF("FAILED LED SETUP\n");
        return 1;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_INF("FAILED LED CONFIGURATION\n");
        return 1;
    }
    LOG_INF("Configured LED");

    // Wait a bit
    LOG_INF("Wait until I have an IP Address");
    k_msleep(5000);

    // Get SNTP clock
    /* ipv4 */
    struct sntp_time sntp_time;
    int rv;
    start = k_cycle_get_32();
    rv = sntp_simple(SNTP_SERVER, 3600, &sntp_time);
    start_cycle = k_cycle_get_32();
    mcu_offset = k_cyc_to_us_ceil32((start_cycle-start));

    LOG_INF("status: %d", rv);
	LOG_INF("time since Epoch: high word: %u, low word: %u, fraction: %u", (uint32_t)(sntp_time.seconds >> 32), (uint32_t)sntp_time.seconds, sntp_time.fraction);
    LOG_INF("MCU offset: %d", mcu_offset);
    offset_time = timetag_to_double((uint32_t)sntp_time.seconds, sntp_time.fraction);
    LOG_INF("Offset time %lf", offset_time);

    // Initialise the OSC address
    LOG_INF("Initialising OSC-IP1  ... ");
    osc1 = lo_address_new("192.168.86.230", "8000");
    LOG_INF("Configured OSC  ...");

    // Create a server
    osc_server = lo_server_new("8000", error);
    lo_server_add_method(osc_server, NULL, NULL, generic_handler, NULL);

    // Initialise base namespace
    baseNamespace.append("TStick_520");
    puara_bundle.init(baseNamespace.c_str());
    initOSC_bundle();

    // Do a slow serialising once 
    puara_bundle.serialise();

    // Print bundle size
    LOG_INF("Bundle Size: %d", puara_bundle.data_len);
    LOG_INF("Starting Sending OSC messages");

    while(1) {
        start = k_cycle_get_32();
        // Counter
		counter++;

        // Get bundle size
        bundle_size = puara_bundle.data_len;

        // Update sntp time
        msg_time = offset_time + (double(k_cyc_to_ms_ceil32(k_cycle_get_32() - start_cycle)) * 0.001);

        // // Get time
        // gettimeofday(&tv, NULL);

        // // Get time double
        // msg_time = timeval_to_double(tv);

		// Only send if network was properly configured
        updateOSC();
        end = k_cycle_get_32();
        looptime = k_cyc_to_us_ceil32((end-start));

        if ((k_uptime_get_32() - last_time) > SLEEP_TIME_MS) {
            ret = gpio_pin_toggle_dt(&led);
            led_state = !led_state;
            last_time = k_uptime_get_32();
        }

        // Sleep thread for a bit
        k_msleep(1);
    }
}
