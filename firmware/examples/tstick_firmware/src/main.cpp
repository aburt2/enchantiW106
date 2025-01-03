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
#include <zephyr/devicetree.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>
#include <errno.h>

// Drivers
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/sensor.h>

// Power management
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>

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

// Sensors
#include "enchanti_touch.h"

// Logger
LOG_MODULE_REGISTER(MAIN);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000
#define LOG_RATE_MS     5000
#define BOOT_UP_DELAY_MS 500
#define OSC_RATE_TICKS 100
#define USEC_PER_TICK (1000000 / CONFIG_SYS_CLOCK_TICKS_PER_SEC)
#define SEC_PER_TICK float(USEC_PER_TICK) / 1000000.0f
#define USEC_PER_CYCLE (1000000 / CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC)

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
#define TSTICK_SIZE 60

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


struct uLiblo {
    int accl;
    int gyro;
    int magn;
    int mimu;
    int quat;
    int ypr;
    int shake;
    int jab;
    int brush;
    int rub;
    int multibrush;
    int multirub;
    int count;
    int tap;
    int dtap;
    int ttap;
    int fsr;
    int squeeze;
    int battery;
    int current;
    int voltage;
    int tte;
    int touchAll;         
    int touchTop;         
    int touchMiddle;      
    int touchBottom;      
    int mergedtouch;
    int mergeddiscretetouch;
    int debug;
} ulo;



struct Events {
    bool battery;
    bool mag;
    bool button_pressed;
    bool button_held;
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
struct sensor_timers debounce(25);
struct sensor_timers button_hold(5000);

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
    start = k_cycle_get_32();
    updateOSC_bundle();
    
    if (wifi_enabled && !ap_enabled) {
        puara_bundle.fast_send(osc1, osc_server);
    }
    end = k_cycle_get_32();
    sensors.debug[2] = k_cyc_to_us_ceil32(end - start);
}

void initOSC_bundle() {
    // Continuously send FSR data
    puara_bundle.add(&ulo.fsr, "raw/fsr", 2, sensors.fsr);
    puara_bundle.add(&ulo.squeeze, "instrument/squeeze", 2, sensors.squeeze);

    //Send touch data
    puara_bundle.add(&ulo.touchAll, "instrument/touch/all", sensors.touchAll);
    puara_bundle.add(&ulo.touchTop, "instrument/touch/top", sensors.touchTop);
    puara_bundle.add(&ulo.touchMiddle, "instrument/touch/middle", sensors.touchMiddle);
    puara_bundle.add(&ulo.touchBottom, "instrument/touch/bottom", sensors.touchBottom);
    puara_bundle.add(&ulo.mergedtouch, "raw/capsense", TSTICK_SIZE, sensors.mergedtouch);
    // Touch gestures
    puara_bundle.add(&ulo.brush, "instrument/brush", sensors.brush);
    puara_bundle.add(&ulo.multibrush, "instrument/multibrush", 4, sensors.multibrush);
    puara_bundle.add(&ulo.rub, "instrument/rub", sensors.rub);
    puara_bundle.add(&ulo.multirub, "instrument/multirub", 4, sensors.multirub);
    
    // MIMU data
    puara_bundle.add(&ulo.accl, "raw/accl", 3, sensors.accl);
    puara_bundle.add(&ulo.gyro, "raw/gyro", 3, sensors.gyro);
    puara_bundle.add(&ulo.magn, "raw/magn", 3, sensors.magn);
    puara_bundle.add(&ulo.ypr, "ypr", 3, sensors.ypr); 
    puara_bundle.add(&ulo.quat,"orientation", 4, sensors.quat); 

    // Inertial gestures
    puara_bundle.add(&ulo.shake, "instrument/shakexyz", 3, sensors.shake);
    puara_bundle.add(&ulo.jab, "instrument/jabxyz", 3, sensors.jab);
    // Button Gestures
    puara_bundle.add(&ulo.count, "instrument/button/count", sensors.count);
    puara_bundle.add(&ulo.tap, "instrument/button/tap", sensors.tap);
    puara_bundle.add(&ulo.dtap, "instrument/button/dtap", sensors.dtap);
    puara_bundle.add(&ulo.ttap, "instrument/button/ttap", sensors.ttap);

    // Battery Data
    puara_bundle.add(&ulo.battery, "battery/percentage", sensors.battery);
    puara_bundle.add(&ulo.current, "battery/current", sensors.current);
    puara_bundle.add(&ulo.tte, "battery/timetoempty", sensors.tte);
    puara_bundle.add(&ulo.voltage, "battery/voltage", sensors.voltage);  

    // Add counter
    puara_bundle.add(&ulo.debug, "debug", 3, sensors.debug);
}

void updateOSC_bundle() {
    // Continuously send FSR data
    puara_bundle.update_message(ulo.fsr, 2, sensors.fsr);
    puara_bundle.update_message(ulo.squeeze, 2, sensors.squeeze);

    //Send touch data
    puara_bundle.update_message(ulo.touchAll, sensors.touchAll);
    puara_bundle.update_message(ulo.touchTop, sensors.touchTop);
    puara_bundle.update_message(ulo.touchMiddle, sensors.touchMiddle);
    puara_bundle.update_message(ulo.touchBottom, sensors.touchBottom);
    puara_bundle.update_message(ulo.mergedtouch, TSTICK_SIZE, sensors.mergedtouch);
    // Touch gestures
    puara_bundle.update_message(ulo.brush, sensors.brush);
    puara_bundle.update_message(ulo.multibrush, 4, sensors.multibrush);
    puara_bundle.update_message(ulo.rub, sensors.rub);
    puara_bundle.update_message(ulo.multirub, 4, sensors.multirub);
    
    // MIMU data
    puara_bundle.update_message(ulo.accl, 3, sensors.accl);
    puara_bundle.update_message(ulo.gyro, 3, sensors.gyro);
    puara_bundle.update_message(ulo.magn, 3, sensors.magn);
    puara_bundle.update_message(ulo.ypr, 3, sensors.ypr); 
    puara_bundle.update_message(ulo.quat, 4, sensors.quat); 

    // Inertial gestures
    puara_bundle.update_message(ulo.shake, 3, sensors.shake);
    puara_bundle.update_message(ulo.jab, 3, sensors.jab);
    // Button Gestures
    puara_bundle.update_message(ulo.count, sensors.count);
    puara_bundle.update_message(ulo.tap, sensors.tap);
    puara_bundle.update_message(ulo.dtap, sensors.dtap);
    puara_bundle.update_message(ulo.ttap, sensors.ttap);

    // Battery Data
    puara_bundle.update_message(ulo.battery, sensors.battery);
    puara_bundle.update_message(ulo.current, sensors.current);
    puara_bundle.update_message(ulo.tte, sensors.tte);
    puara_bundle.update_message(ulo.voltage, sensors.voltage);

    // Add counter
    puara_bundle.update_message(ulo.debug, 3, sensors.debug);
}

// /* The devicetree node identifier for the "led0" alias. */
#define ORANGELED_NODE DT_ALIAS(led0)
#define BLUELED_NODE DT_ALIAS(led1)
#define LDOEN_NODE DT_ALIAS(ldoen1)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec orange_led = GPIO_DT_SPEC_GET(ORANGELED_NODE, gpios);
static const struct gpio_dt_spec blue_led = GPIO_DT_SPEC_GET(BLUELED_NODE, gpios);
static const struct gpio_dt_spec ldo_en = GPIO_DT_SPEC_GET(LDOEN_NODE, gpios);


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
    if (!events.button_pressed) button_hold.timer = k_uptime_get_32();
	events.button_pressed = true;
}

/*
* Sensor Devices
*/
const struct device *const fuelgauge = DEVICE_DT_GET_ONE(maxim_max17262);
const struct device *const imu = DEVICE_DT_GET_ONE(invensense_icm42670);
const struct device *const magn = DEVICE_DT_GET_ONE(memsic_mmc56x3);
EnchantiTouch touch;
enchanti_touch_config tstick_touchconfig = {
    -1, // default use the trill craft device
    ENCHANTI_BASETOUCHSIZE, // default touch size
    0, // noise threshold
    Mode::DIFF, // touch processing mode
    COMMS::I2C_MODE, // comm mode 
};

bool led_state = true;


// IMU orientation
IMU_Orientation orientation;
float accelg[3];
// Define some conversions
#define MS_PER_HOUR 3600000.0f
#define MS2_TO_G 1.0f/9.8f // conversion from ms2 to gs
#define RAD_TO_DEGREES 180.0f / M_PIF

/* Sensor Helpers */
void readIMU();
void readButton();
void readTouch();
void readAnalog();
void readBattery();
void changeLED();
void updateMIMU();
int initDevices();
float normDegree(float val);

void readButton() {
    // Check if debounce time had passed
    uint32_t now = k_uptime_get_32();
    if (now - debounce.timer < debounce.interval) return; // if debounce hasn't passed do not bother
    debounce.timer = now;

    // Read button
    int val = gpio_pin_get_dt(&button);

    if (val == 1) {
        // We are holding the button
        if ((now - button_hold.timer) > button_hold.interval) {
            events.button_held = true;
        }
    } else if (events.button_pressed) {
        // We have released the button
        if (events.button_held || events.button_pressed) {
            events.button_held = false;
            events.button_pressed = false;
        }
        sensors.count++;
    }
}

void readAnalog() {
    // Read analog sensor
    for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
        (void)adc_sequence_init_dt(&adc_channels[i], &sequence);
        int err;
        err = adc_read_dt(&adc_channels[i], &sequence);
        if (err < 0) {
            LOG_ERR("Could not read (%d)\n", err);
            continue;
        }

        /*
        * If using differential mode, the 16 bit value
        * in the ADC sample buffer should be a signed 2's
        * complement value.
        */
        if (adc_channels[i].channel_cfg.differential) {
            sensors.fsr[i] = (int32_t)((int16_t)buf);
        } else {
            sensors.fsr[i] = (int32_t)buf;
        }
    }
}

void readTouch() {
    // Read touch data
    touch.readTouch();
    touch.cookData();

    // Store in arrays for sensing
    memcpy(sensors.mergedtouch, touch.touch, sizeof(int) * TSTICK_SIZE);
    memcpy(sensors.mergeddiscretetouch, touch.discreteTouch, sizeof(int) * TSTICK_SIZE);
}

void readIMU() {
    // Read data from IMU
    sensor_sample_fetch(imu);

    // Store sensor info
	struct sensor_value accel[3], gyro[3];
    sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ,accel);
    sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ,gyro);

    // Save data to sensor array
    sensors.accl[0] = -sensor_value_to_float(&accel[0]);
    sensors.accl[1] = sensor_value_to_float(&accel[1]); // fix axis orientation
    sensors.accl[2] = sensor_value_to_float(&accel[2]);
    sensors.gyro[0] = sensor_value_to_float(&gyro[0]);
    sensors.gyro[1] = sensor_value_to_float(&gyro[1]);
    sensors.gyro[2] = sensor_value_to_float(&gyro[2]);
    
    // Also save accel data in gs for sensor fusion
    accelg[0] = sensors.accl[0] * MS2_TO_G;
    accelg[1] = sensors.accl[1] * MS2_TO_G;
    accelg[2] = sensors.accl[2] * MS2_TO_G;

    // Read magnetometer at specified rate
    if ((k_uptime_get_32() - magnetometer.timer) > magnetometer.interval) {
        sensor_sample_fetch(magn);

        struct sensor_value mag[3];
        sensor_channel_get(magn, SENSOR_CHAN_MAGN_XYZ,mag);
        // store in uTesla
        sensors.magn[0] = sensor_value_to_float(&mag[1]) * 100.0f; // x axis of T-Stick is the Y Axis of the magnetomer
        sensors.magn[1] = sensor_value_to_float(&mag[0]) * 100.0f; // y axis of T-Stick is the x Axis of the magnetomer
        sensors.magn[2] = sensor_value_to_float(&mag[2]) * 100.0f;

        // Update timer
        magnetometer.timer = k_uptime_get_32();
        events.mag = true;
    }
}

float normDegree(float val) {
    // Normalise degrees from -180,180 to 0,360 degrees
    if (val < 0) val += 360.0f;
    return val;
}

void updateMIMU() {
    // Update my data, at least 1kHz
    if (k_uptime_get_32() - sensor_fusion.timer > 0) {
        // Read IMU data
        readIMU();

        // Update sensor fusion
        uint32_t deltaT = (k_uptime_get_32() - sensor_fusion.timer) * 0.001f;

        // Perform sensor fusion
        orientation.setAccelerometerValues(accelg[0], accelg[1], accelg[2]);
        orientation.setGyroscopeRadianValues(sensors.gyro[0],sensors.gyro[1],sensors.gyro[2], deltaT);
        orientation.setMagnetometerValues(sensors.magn[0], sensors.magn[1], sensors.magn[2]);

        orientation.setAccelerometerValues(sensors.accl[0], sensors.accl[1], sensors.accl[2]);
        orientation.setGyroscopeRadianValues(sensors.gyro[0],sensors.gyro[1],sensors.gyro[2], deltaT);
        orientation.setMagnetometerValues(sensors.magn[0], sensors.magn[1], sensors.magn[2]);

        // Update sensor fusion
        orientation.update();

        // Save orientation
        sensors.ypr[0] = orientation.euler.azimuth * RAD_TO_DEGREES;
        sensors.ypr[1] = orientation.euler.pitch * RAD_TO_DEGREES;
        sensors.ypr[2] = orientation.euler.roll * RAD_TO_DEGREES;

        // normalise roll and heading to 0 - 360
        sensors.ypr[2] = normDegree(sensors.ypr[2]);

        // Save quaternions
        sensors.quat[0] = orientation.quaternion.w;
        sensors.quat[1] = orientation.quaternion.x; 
        sensors.quat[2] = orientation.quaternion.y;
        sensors.quat[3] = orientation.quaternion.z;

        // Clean up float precision (round to 1 decimal place)
        for (int i = 0; i < 3; i++) {
            sensors.ypr[i] = roundf(sensors.ypr[i] * 10) / 10;
        }

        // Update sensor fusion timer
        sensor_fusion.timer = k_uptime_get_32();;
        events.mag = false;
    }
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

int initDevices() {
    // Error variable
    int ret;

    // Initialise Button
    if (!gpio_is_ready_dt(&button)) {
		LOG_ERR("Error: button device %s is not ready\n",
		       button.port->name);
		return 1;
	}
	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		LOG_ERR("Error %d: failed to configure %s pin %d\n",
		       ret, button.port->name, button.pin);
		return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		LOG_ERR("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, button.port->name, button.pin);
		return ret;
	}
    gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
    LOG_INF("Configured Buttons  ...");

    // Wait for wifi network manager to be initialised
    k_msleep(BOOT_UP_DELAY_MS);

    // Turn on Orange LED
    if (!gpio_is_ready_dt(&orange_led)) {
        LOG_INF("FAILED ORANGE LED SETUP\n");
        return 1;
    }

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

    // Enable LDO
    ret = gpio_pin_configure_dt(&ldo_en, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        LOG_INF("FAILED LDO CONFIGURATION\n");
        return 1;
    }
    LOG_INF("Configured LDO");

    // Check if sensor device is ready
    if (!device_is_ready(fuelgauge)) {
		LOG_ERR("fuelgauge: device not ready.\n");
		return 1;
	}
    LOG_INF("Configured Fuel Gauge  ...");
    if (!device_is_ready(imu)) {
		LOG_ERR("imu: device not ready.\n");
		return 1;
	}
    LOG_INF("Configured IMU  ...");
    if (!device_is_ready(magn)) {
		LOG_ERR("magnetometer: device not ready.\n");
		return 1;
	}
    LOG_INF("Configured Magnetometer  ...");

    // Init touch
    int err = touch.initTouch(tstick_touchconfig);
    if (err == 0) {
        LOG_ERR("touch: device not ready.\n");
        return 1;
    }
    LOG_INF("Configured Touch sensor  ...");

    /* Configure adc channels individually prior to sampling. */
	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		if (!adc_is_ready_dt(&adc_channels[i])) {
			LOG_ERR("ADC controller device %s not ready\n", adc_channels[i].dev->name);
			return 1;
		}

		int err = adc_channel_setup_dt(&adc_channels[i]);
		if (err < 0) {
			LOG_ERR("Could not setup channel #%d (%d)\n", i, err);
			return 1;
		}
        LOG_INF("Configured ADC %s ...", adc_channels[i].dev->name);
	}

    // if we got through all of that
    return 0;
}
/********* MAIN **********/
int main(void)
{
    // Initialise devices
    int ret;
    ret = initDevices();
    if (ret != 0) {
        LOG_ERR("Devices failed to init");
        return 1;
    }

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

    // Don't start another thread just use main
    LOG_INF("Initialising OSC-IP1  ... ");
    osc1 = lo_address_new(OSC_ADDRESS, OSC_PORT);
    LOG_INF("Configured OSC  ...");

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
    LOG_INF("Wait until I have an IP Address");
    k_msleep(5000);
    LOG_INF("Starting Sending OSC messages");
    
    // Serialise the bundle once
    puara_bundle.serialise();

    while(1) {
        // Get data from fuelgauge
        start = k_cycle_get_32();
        if ((k_uptime_get_32() - battery.timer) > battery.interval) {
            readBattery();
            battery.timer = k_uptime_get_32();
        } else {
            events.battery = false;
        }
        // Read Button
        readButton();
        
        // Read ADC
        readAnalog();

        // Read touch
        // TODO: use non blocking i2c call
        readTouch();
        
        // Get Data from IMU and magnetometer
        updateMIMU();

        // Counter
		sensors.debug[0]++;

		// Time Sensor read length
        end = k_cycle_get_32();
        sensors.debug[1] = k_cyc_to_us_ceil32(end-start);

        // Send OSC
        updateOSC();
        
        // Update LED
        changeLED();

        // Sleep thread for a bit
        k_yield();
    }

    //     // Test serialisation speed
    // int num_loops = 1000;
    // while(1) {
    //     // Test serialise speed
    //     start = k_uptime_ticks();
    //     for (int i = 0; i < num_loops; i++) {
    //         puara_bundle.fast_serialise();
    //     }
    //     end = k_uptime_ticks();
    //     sensors.debug[1] = ((end - start) * USEC_PER_TICK) / num_loops;
    //     LOG_INF("Serialising time: %d", sensors.debug[1]);
    //     // Test updating speed
    //     start = k_uptime_ticks();
    //     for (int i = 0; i < num_loops; i++) {
    //         updateOSC();
    //     }
    //     end = k_uptime_ticks();
    //     sensors.debug[2] = ((end - start) * USEC_PER_TICK) / num_loops;
    //     sensors.debug[2] = sensors.debug[2];
    //     LOG_INF("Sending time: %d", sensors.debug[2]);
    //     LOG_INF("Bundle Size: %d", puara_bundle.data_len);

    //     // Update counter
    //     sensors.debug[0]++;

    //     // sleep a biy
    //     k_yield();
    // }

    // NOtes for Performance
    // Sending OSC data ~= 160 - 300us
    // IMU sensor fusion + data gathering ~= 300us
    // ADC ~= 100us
    // Readfing touch ~= 1000us
}