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
#include <zephyr/drivers/adc.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
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

// Sensor fusion
#include "imu_orientation.h"
#include <cmath>

// Logger
LOG_MODULE_REGISTER(MAIN);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000
#define LOG_RATE_MS     5000
#define BOOT_UP_DELAY_MS 500
#define OSC_RATE_TICKS 10
#define USEC_PER_TICK (1000000 / CONFIG_SYS_CLOCK_TICKS_PER_SEC)
#define SEC_PER_TICK float(USEC_PER_TICK) / 1000000.0f

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

    /* OSC Configuration */
    #define OSC_ADDRESS        "192.168.137.1"
    #define OSC_PORT           "8000"
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
oscBundle puara_bundle;
int counter = 0;
int looptime = 0;
int last_time = 0;
int last_log_time = 0;
bool wifi_on = false;
int start = 0;
int end = 0;
#define TSTICK_SIZE 120

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
    int fsr[2];
    float squeeze[2];
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
    int debug[3];
} sensors;



struct Events {
    bool battery;
    bool mag;
} events;

//////////////////////////////////
// Sensor Timers                //
//////////////////////////////////
struct sensor_timers {
    uint32_t timer = 0;
    uint32_t interval;

    sensor_timers(int period) : interval(period) {};
};

struct sensor_timers battery(5000);
struct sensor_timers magnetometer(4);
struct sensor_timers led(SLEEP_TIME_MS);
struct sensor_timers sensor_fusion(1);

// OSC Helpers
void error(int num, const char *msg, const char *path);
int generic_handler(const char *path, const char *types, lo_arg ** argv,
                    int argc, lo_message data, void *user_data);
void updateOSC();
void initOSC_bundle();
void updateOSC_bundle();

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
    start = k_uptime_ticks();
    updateOSC_bundle();
    end = k_uptime_ticks();
    sensors.debug[2] = (end-start)*USEC_PER_TICK;

    if (wifi_enabled && !ap_enabled) {
        lo_send_bundle_from(osc1, osc_server, puara_bundle.bundle);
    }
}

void initOSC_bundle() {
    // Continuously send FSR data
    puara_bundle.add_array("raw/fsr", 2, sensors.fsr);
    puara_bundle.add_array("instrument/squeeze", 2, sensors.squeeze);

    //Send touch data
    puara_bundle.add("instrument/touch/all", sensors.touchAll);
    puara_bundle.add("instrument/touch/top", sensors.touchTop);
    puara_bundle.add("instrument/touch/middle", sensors.touchMiddle);
    puara_bundle.add("instrument/touch/bottom", sensors.touchBottom);
    puara_bundle.add_array("raw/capsense", TSTICK_SIZE, sensors.mergedtouch);
    // Touch gestures
    puara_bundle.add("instrument/brush", sensors.brush);
    puara_bundle.add_array("instrument/multibrush", 4, sensors.multibrush);
    puara_bundle.add("instrument/rub", sensors.rub);
    puara_bundle.add_array("instrument/multirub", 4, sensors.multirub);
    
    // MIMU data
    puara_bundle.add_array("raw/accl", 3, sensors.accl);
    puara_bundle.add_array("raw/gyro", 3, sensors.gyro);
    puara_bundle.add_array("raw/magn", 3, sensors.magn);
    puara_bundle.add_array("ypr", 3, sensors.ypr); 

    // Inertial gestures
    puara_bundle.add_array("instrument/shakexyz", 3, sensors.shake);
    puara_bundle.add_array("instrument/jabxyz", 3, sensors.jab);
    // Button Gestures
    puara_bundle.add("instrument/button/count", sensors.count);
    puara_bundle.add("instrument/button/tap", sensors.tap);
    puara_bundle.add("instrument/button/dtap", sensors.dtap);
    puara_bundle.add("instrument/button/ttap", sensors.ttap);

    // Battery Data
    puara_bundle.add("battery/percentage", sensors.battery);
    puara_bundle.add("battery/current", sensors.current);
    puara_bundle.add("battery/timetoempty", sensors.tte);
    puara_bundle.add("battery/voltage", sensors.voltage);  

    // Add counter
    puara_bundle.add_array("debug", 3, sensors.debug);
}

void updateOSC_bundle() {
    // Continuously send FSR data
    puara_bundle.update_message(0, 2, sensors.fsr);
    puara_bundle.update_message(1, 2, sensors.squeeze);

    //Send touch data
    puara_bundle.update_message(2, sensors.touchAll);
    puara_bundle.update_message(3, sensors.touchTop);
    puara_bundle.update_message(4, sensors.touchMiddle);
    puara_bundle.update_message(5, sensors.touchBottom);
    puara_bundle.update_message(6, TSTICK_SIZE, sensors.mergedtouch);
    // Touch gestures
    puara_bundle.update_message(7, sensors.brush);
    puara_bundle.update_message(8, 4, sensors.multibrush);
    puara_bundle.update_message(9, sensors.rub);
    puara_bundle.update_message(10, 4, sensors.multirub);
    
    // MIMU data
    puara_bundle.update_message(11, 3, sensors.accl);
    puara_bundle.update_message(12, 3, sensors.gyro);
    puara_bundle.update_message(13, 3, sensors.magn);
    puara_bundle.update_message(14, 3, sensors.ypr); 

    // Inertial gestures
    puara_bundle.update_message(15, 3, sensors.shake);
    puara_bundle.update_message(16, 3, sensors.jab);
    // Button Gestures
    puara_bundle.update_message(17, sensors.count);
    puara_bundle.update_message(18, sensors.tap);
    puara_bundle.update_message(19, sensors.dtap);
    puara_bundle.update_message(20, sensors.ttap);

    // Battery Data
    puara_bundle.update_message(21, sensors.battery);
    puara_bundle.update_message(22, sensors.current);
    puara_bundle.update_message(23, sensors.tte);
    puara_bundle.update_message(24, sensors.voltage);

    // Add counter
    puara_bundle.update_message(25, 3, sensors.debug);
}

// /* The devicetree node identifier for the "led0" alias. */
#define ORANGELED_NODE DT_ALIAS(led0)
#define BLUELED_NODE DT_ALIAS(led1)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec orange_led = GPIO_DT_SPEC_GET(ORANGELED_NODE, gpios);
static const struct gpio_dt_spec blue_led = GPIO_DT_SPEC_GET(BLUELED_NODE, gpios);


/* ADC Config from devicetree */
#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};

// ADC structures
uint16_t buf;
struct adc_sequence sequence = {
    .buffer = &buf,
    /* buffer size in bytes, not number of samples */
    .buffer_size = sizeof(buf),
};

/*
 * Get button configuration from the devicetree sw0 alias. This is mandatory.
 */
#define SW0_NODE	DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS_OKAY(SW0_NODE)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
							      {0});
static struct gpio_callback button_cb_data;

void button_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	sensors.count++;
}

/*
* Sensor Devices
*/
const struct device *const fuelgauge = DEVICE_DT_GET_ONE(maxim_max17262);
const struct device *const imu = DEVICE_DT_GET_ONE(invensense_icm42670);
const struct device *const magn = DEVICE_DT_GET_ONE(memsic_mmc56x3);
bool led_state = true;

// IMU orientation
IMU_Orientation orientation;

// Define some conversions
#define MS_PER_HOUR 3600000.0f

/* Sensor Helpers */
void readIMU();
void readTouch();
void readAnalog();
void readBattery();
void changeLED();
void updateMIMU();
void readSensor();

void readIMU() {
    // Read data from IMU
    sensor_sample_fetch(imu);

    // Store sensor info
	struct sensor_value accel[3], gyro[3];
    sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ,accel);
    sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ,gyro);

    // Save data to sensor array
    sensors.accl[0] = sensor_value_to_float(&accel[0]);
    sensors.accl[1] = sensor_value_to_float(&accel[1]);
    sensors.accl[2] = sensor_value_to_float(&accel[2]);
    sensors.gyro[0] = sensor_value_to_float(&gyro[0]);
    sensors.gyro[1] = sensor_value_to_float(&gyro[1]);
    sensors.gyro[2] = sensor_value_to_float(&gyro[2]);

    // Read magnetometer at specified rate
    if ((k_uptime_get_32() - magnetometer.timer) > magnetometer.interval) {
        sensor_sample_fetch(magn);

        struct sensor_value mag[3];
        sensor_channel_get(magn, SENSOR_CHAN_MAGN_XYZ,mag);
        // store in uTesla
        sensors.magn[0] = sensor_value_to_float(&mag[0]) * 100.0f;
        sensors.magn[1] = sensor_value_to_float(&mag[1]) * 100.0f;
        sensors.magn[2] = sensor_value_to_float(&mag[2]) * 100.0f;

        // Update timer
        uint32_t now =  k_uptime_get_32();
        magnetometer.timer = now;

        // Update sensor fusion
        uint32_t deltaT = (now- sensor_fusion.timer) * 0.001f;
        sensor_fusion.timer = now;

        // Set Values
        orientation.setAccelerometerValues(sensors.accl[0], sensors.accl[1], sensors.accl[2]);
        orientation.setGyroscopeRadianValues(sensors.gyro[0],sensors.gyro[1],sensors.gyro[2], deltaT);
        orientation.setMagnetometerValues(sensors.magn[0], sensors.magn[1], sensors.magn[2]);

        // Update sensor fusion
        orientation.update();

        // Get values and store them
        sensors.ypr[0] = float(orientation.euler.azimuth) * 180.0f / float(M_PI);
        sensors.ypr[1] = float(orientation.euler.pitch) * 180.0f / float(M_PI);
        sensors.ypr[2] = float(orientation.euler.roll) * 180.0f / float(M_PI);

        // normalise to 0 - 360
        for (int i = 0; i < 3; i++) {
            if (sensors.ypr[i] < 0) {
                sensors.ypr[i] = 360.0f + sensors.ypr[i];
            }
        }
    }
}

void updateMIMU() {
    // Read IMU data
    readIMU();

    // Perform sensor fusion
    // TODO
}

// Data from the fuel gauge
void readBattery() {
    // Read battery stats from fuel gauge
    sensor_sample_fetch(fuelgauge);

    // Store battery info
    struct sensor_value voltage, avg_current, percentage, tte, ttf;
    sensor_channel_get(fuelgauge, SENSOR_CHAN_GAUGE_VOLTAGE, &voltage);
    sensor_channel_get(fuelgauge, SENSOR_CHAN_GAUGE_AVG_CURRENT,&avg_current);
    sensor_channel_get(fuelgauge, SENSOR_CHAN_GAUGE_STATE_OF_CHARGE,&percentage);
    sensor_channel_get(fuelgauge, SENSOR_CHAN_GAUGE_TIME_TO_EMPTY,&tte);
    sensor_channel_get(fuelgauge, SENSOR_CHAN_GAUGE_TIME_TO_FULL,&ttf);

    // Save to sensors array
    sensors.battery = sensor_value_to_float(&percentage);
    sensors.tte = (sensor_value_to_float(&tte) - sensor_value_to_float(&ttf)) / MS_PER_HOUR; // negative if the battery is charging
    sensors.voltage = sensor_value_to_float(&voltage);
    sensors.current = sensor_value_to_float(&avg_current);

    // Save events vector
    events.battery = true;
}

void changeLED() {
    // TODO: USE PWM instead of toggle
    if ((k_uptime_get_32() - last_time) > SLEEP_TIME_MS) {
        gpio_pin_toggle_dt(&blue_led);
        led_state = !led_state;
        last_time = k_uptime_get_32();
    }
}


/* Main loop */
static void osc_loop(void *, void *, void *);

static void osc_loop(void *, void *, void *) {
    LOG_INF("Initialising OSC-IP1  ... ");
    osc1 = lo_address_new(OSC_ADDRESS, OSC_PORT);
    LOG_INF("Configured OSC  ...");

    // Check if sensor device is ready
    if (!device_is_ready(fuelgauge)) {
		LOG_ERR("fuelgauge: device not ready.\n");
		return;
	}
    LOG_INF("Configured Fuel Gauge  ...");
    if (!device_is_ready(imu)) {
		LOG_ERR("imu: device not ready.\n");
		return;
	}
    LOG_INF("Configured IMU  ...");
    if (!device_is_ready(magn)) {
		LOG_ERR("magnetometer: device not ready.\n");
		return;
	}
    LOG_INF("Configured Magnetometer  ...");

    /* Configure adc channels individually prior to sampling. */
	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		if (!adc_is_ready_dt(&adc_channels[i])) {
			LOG_ERR("ADC controller device %s not ready\n", adc_channels[i].dev->name);
			return;
		}

		int err = adc_channel_setup_dt(&adc_channels[i]);
		if (err < 0) {
			LOG_ERR("Could not setup channel #%d (%d)\n", i, err);
			return;
		}
	}
    LOG_INF("Configured ADC  ...");

    // Initialise Button
    gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
    LOG_INF("Configured Buttons  ...");

    // Create a server
    osc_server = lo_server_new("8000", error);
    lo_server_add_method(osc_server, NULL, NULL, generic_handler, NULL);

    // Initialise debug array
    sensors.debug[0] = 0;
    sensors.debug[1] = 0;
    sensors.debug[2] = 0;

    // Initialise base namespace
    baseNamespace.append("TStick_520");
    oscNamespace = baseNamespace;

    // Init puara bundle
    puara_bundle.init(baseNamespace.c_str());
    initOSC_bundle();

    LOG_INF("Bundle initialised with %d messages", puara_bundle.num_messages);


    // Wait a bit
    k_msleep(500);
    LOG_INF("Starting Sending OSC messages");

    while(1) {
        // Get data from fuelgauge
        start = k_uptime_ticks();
        if ((k_uptime_get_32() - battery.timer) > battery.interval) {
            readBattery();
            battery.timer = k_uptime_get_32();
        } else {
            events.battery = false;
        }

        // Read ADC
        for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
            (void)adc_sequence_init_dt(&adc_channels[i], &sequence);

            // Save reading
            sensors.fsr[i] = (int32_t)buf;
        }
        
        // Get Data from IMU and magnetometer
        updateMIMU();

        // Counter
		sensors.debug[0]++;

		// Time Sensor read length
        end = k_uptime_ticks();
        sensors.debug[1] = (end-start)*USEC_PER_TICK;

        // Send OSC
        updateOSC();
        
        // Update LED
        changeLED();

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

    // Turn on Orange LED
    if (!gpio_is_ready_dt(&orange_led)) {
        LOG_INF("FAILED ORANGE LED SETUP\n");
        return 1;
    }

    int ret;
    ret = gpio_pin_configure_dt(&orange_led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        LOG_INF("FAILED ORANGE LED CONFIGURATION\n");
        return 1;
    }
    // Turn off BLue LED
    if (!gpio_is_ready_dt(&blue_led)) {
        LOG_INF("FAILED BLUE LED SETUP\n");
        return 1;
    }

    ret = gpio_pin_configure_dt(&blue_led, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_INF("FAILED BLUE LED CONFIGURATION\n");
        return 1;
    }
    LOG_INF("Configured LED");

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

    // toggle off LED once wifi is connected
    ret = gpio_pin_toggle_dt(&orange_led);

    k_tid_t my_tid = k_thread_create(&osc_thread_data, osc_stack_area,
                            K_THREAD_STACK_SIZEOF(osc_stack_area),
                            osc_loop,
                            NULL, NULL, NULL,
                            OSC_PRIORITY, 0, K_NO_WAIT);


    // Return 0 and let threads handle it
    return 0;
}
