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
#include <lo/lo.h>
#include <lo/lo_lowlevel.h>
#include <lo/lo_types.h>

// Logger
LOG_MODULE_REGISTER(MAIN);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000
#define LOG_RATE_MS     5000
#define BOOT_UP_DELAY_MS 500
#define OSC_RATE_TICKS 10
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
lo_server osc_server;
int counter = 0;
int looptime = 0;
int last_time = 0;
int last_log_time = 0;
bool wifi_on = false;
int start = 0;
int end = 0;
#define TSTICK_SIZE 4
int test_array[TSTICK_SIZE];

std::string baseNamespace = "/";
std::string oscNamespace;

struct Sensors {
    float accl [3];
    float gyro [3];
    float magn [3];
    float mimu [9];
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
    float battery[4];
    float current;
    float voltage;
    float tte;
    float touchAll;         
    float touchTop;         
    float touchMiddle;      
    float touchBottom;      
    int mergedtouch[TSTICK_SIZE];
    int mergeddiscretetouch[TSTICK_SIZE];
    int debug[3];
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
    oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), path);
    int ret = 0;
    lo_message tmp_osc = lo_message_new();
    ret = lo_message_add_int32(tmp_osc, value);
    if (ret < 0) {
        lo_message_free(tmp_osc);
        return;
    }
    ret = lo_bundle_add_message(puara_bundle, oscNamespace.c_str(), tmp_osc);
}
void osc_bundle_add_float(lo_bundle puara_bundle,const char *path, float value) {
    oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), path);
    int ret = 0;
    lo_message tmp_osc = lo_message_new();
    ret = lo_message_add_float(tmp_osc, value);
    if (ret < 0) {
        lo_message_free(tmp_osc);
        return;
    }
    ret = lo_bundle_add_message(puara_bundle, oscNamespace.c_str(), tmp_osc);
}
void osc_bundle_add_int_array(lo_bundle puara_bundle,const char *path, int size, int *value) {
    oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), path);
    lo_message tmp_osc = lo_message_new();
    for (int i = 0; i < size; i++) {
        lo_message_add_int32(tmp_osc, value[i]);
    }
    lo_bundle_add_message(puara_bundle, oscNamespace.c_str(), tmp_osc);
}

void osc_bundle_add_float_array(lo_bundle puara_bundle,const char *path, int size,  float *value) {
    oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), path);
    lo_message tmp_osc = lo_message_new();
    for (int i = 0; i < size; i++) {
        lo_message_add_float(tmp_osc, value[i]);
    }
    lo_bundle_add_message(puara_bundle, oscNamespace.c_str(), tmp_osc);
}

void updateOSC() {
    // Create a bundle and send it to both IP addresses
    lo_bundle bundle = lo_bundle_new(LO_TT_IMMEDIATE);
    if (!bundle) {
        return;
    }

    start = k_uptime_ticks();
    updateOSC_bundle(bundle);
    end = k_uptime_ticks();
    sensors.debug[2] = (end-start)*USEC_PER_TICK;

    if (wifi_enabled && !ap_enabled) {
        lo_send_bundle_from(osc1, osc_server, bundle);
    }
    

    // free memory from bundle
    lo_bundle_free_recursive(bundle);
}

void updateOSC_bundle(lo_bundle bundle) {
    // Continuously send FSR data
    osc_bundle_add_int(bundle, "raw/fsr", sensors.fsr);
    osc_bundle_add_float(bundle, "instrument/squeeze", sensors.squeeze);

    //Send touch data
    osc_bundle_add_float(bundle, "instrument/touch/all", sensors.touchAll);
    // osc_bundle_add_float(bundle, "instrument/touch/top", sensors.touchTop);
    // osc_bundle_add_float(bundle, "instrument/touch/middle", sensors.touchMiddle);
    // osc_bundle_add_float(bundle, "instrument/touch/bottom", sensors.touchBottom);
    osc_bundle_add_int_array(bundle, "raw/capsense", TSTICK_SIZE, sensors.mergedtouch);
    // Touch gestures
    osc_bundle_add_float(bundle, "instrument/brush", sensors.brush);
    osc_bundle_add_float_array(bundle, "instrument/multibrush", 4, sensors.multibrush);
    osc_bundle_add_float(bundle, "instrument/rub", sensors.rub);
    osc_bundle_add_float_array(bundle, "instrument/multirub", 4, sensors.multirub);
    
    // MIMU data
    // osc_bundle_add_float_array(bundle, "raw/accl", 3, sensors.accl);
    // osc_bundle_add_float_array(bundle, "raw/gyro", 3, sensors.gyro);
    // osc_bundle_add_float_array(bundle, "raw/magn", 3, sensors.magn);
    osc_bundle_add_float_array(bundle, "raw/motion", 9, sensors.mimu);
    // osc_bundle_add_float_array(bundle, "orientation", 4, sensors.quat);
    osc_bundle_add_float_array(bundle, "ypr", 3, sensors.ypr); 

    // Inertial gestures
    osc_bundle_add_float_array(bundle, "instrument/shakexyz", 3, sensors.shake);
    osc_bundle_add_float_array(bundle, "instrument/jabxyz", 3, sensors.jab);
    // Button Gestures
    osc_bundle_add_int(bundle, "instrument/button/count", sensors.count);
    // osc_bundle_add_int(bundle, "instrument/button/tap", sensors.tap);
    // osc_bundle_add_int(bundle, "instrument/button/dtap", sensors.dtap);
    // osc_bundle_add_int(bundle, "instrument/button/ttap", sensors.ttap);

    // Battery Data
    osc_bundle_add_float_array(bundle, "battery/status", 4, sensors.battery);
    // osc_bundle_add_float(bundle, "battery/current", sensors.current);
    // osc_bundle_add_float(bundle, "battery/timetoempty", sensors.tte);
    // osc_bundle_add_float(bundle, "battery/voltage", sensors.voltage);  

    // Add counter
    osc_bundle_add_int_array(bundle, "debug", 3, sensors.debug);
}

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

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_INF("FAILED LED CONFIGURATION\n");
        return;
    }
    LOG_INF("Configured LED");

    LOG_INF("Initialising OSC-IP1  ... ");
    osc1 = lo_address_new("192.168.86.230", "8000");
    LOG_INF("Configured OSC  ...");

    // Create a server
    osc_server = lo_server_new("8000", error);
    lo_server_add_method(osc_server, NULL, NULL, generic_handler, NULL);

    // Initialise debug array
    sensors.debug[0] = 0;
    sensors.debug[1] = 0;

    // Initialise base namespace
    baseNamespace.append("TStick_520");
    baseNamespace.append("/");
    oscNamespace = baseNamespace;

    // Wait a bit
    k_msleep(500);
    LOG_INF("Starting Sending OSC messages");

    while(1) {

        // Counter
		sensors.debug[0]++;

		// Only send if network was properly configured
        start = k_uptime_ticks();
        updateOSC();
        end = k_uptime_ticks();
        sensors.debug[1] = (end-start)*USEC_PER_TICK;

        if ((k_uptime_get_32() - last_time) > SLEEP_TIME_MS) {
            ret = gpio_pin_toggle_dt(&led);
            led_state = !led_state;
            last_time = k_uptime_get_32();
        }

        // if ((k_uptime_get_32() - last_log_time) > LOG_RATE_MS) {
        //     k_thread_runtime_stats_t rt_stats_thread;
        //     k_thread_runtime_stats_get(k_current_get(), &rt_stats_thread);
        //     LOG_INF("Cycles: %llu\n", rt_stats_thread.execution_cycles);
        //     last_log_time = k_uptime_get_32();
        // }

        // Sleep thread for a bit
        k_yield();
    }
}

// Create main thread
#define OSC_STACK_SIZE 8192
#define OSC_PRIORITY 5
K_THREAD_STACK_DEFINE(osc_stack_area, OSC_STACK_SIZE);
struct k_thread osc_thread_data;

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

    // Wait a bit
    k_msleep(2000);

    // Connect to wifi
    connect_to_wifi();
    // enable_ap_mode();

    // Start OSC thread
    while (!wifi_enabled) {
        k_msleep(500);
    }
    LOG_INF("Connect to Wifi");

    k_tid_t my_tid = k_thread_create(&osc_thread_data, osc_stack_area,
                            K_THREAD_STACK_SIZEOF(osc_stack_area),
                            osc_loop,
                            NULL, NULL, NULL,
                            OSC_PRIORITY, 0, K_NO_WAIT);


    // Return 0 and let threads handle it
    return 0;
}
