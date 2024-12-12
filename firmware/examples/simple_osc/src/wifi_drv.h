// // function definitions for the nxp wifi driver 

// /**
//  * Copyright 2023-2024 NXP
//  * SPDX-License-Identifier: Apache-2.0
//  *
//  * @file nxp_wifi_drv.c
//  * Shim layer between wifi driver connection manager and zephyr
//  * Wi-Fi L2 layer
//  */

// #define DT_DRV_COMPAT nxp_wifi

// #include <zephyr/net/ethernet.h>
// #include <zephyr/net/dns_resolve.h>
// #include <zephyr/device.h>
// #include <soc.h>
// #include <ethernet/eth_stats.h>
// #include <zephyr/logging/log.h>

// #include <zephyr/net/net_if.h>
// #include <zephyr/net/wifi_mgmt.h>
// #ifdef CONFIG_PM_DEVICE
// #include <zephyr/pm/device.h>
// #endif
// #ifdef CONFIG_WIFI_NM
// #include <zephyr/net/wifi_nm.h>
// #endif
// #include <wlan.h>

// #ifdef __cplusplus
// extern "C" {
// #endif

// /*******************************************************************************
//  * Definitions
//  ******************************************************************************/
// #ifndef NXP_WIFI_SET_FUNC_ATTR
// #ifdef CONFIG_NXP_WIFI_TC_RELOCATE
// #define NXP_WIFI_SET_FUNC_ATTR __ramfunc
// #else
// #define NXP_WIFI_SET_FUNC_ATTR
// #endif
// #endif

// static int nxp_wifi_recv(struct net_if *iface, struct net_pkt *pkt);

// /*******************************************************************************
//  * Prototypes
//  ******************************************************************************/
// #ifdef CONFIG_NXP_WIFI_STA_AUTO_CONN
// static void nxp_wifi_auto_connect(void);
// #endif

// /* Callback Function passed to WLAN Connection Manager. The callback function
//  * gets called when there are WLAN Events that need to be handled by the
//  * application.
//  */
// int nxp_wifi_wlan_event_callback(enum wlan_event_reason reason, void *data);

// static int nxp_wifi_wlan_init(void);
// static int nxp_wifi_wlan_start(void);

// #ifdef CONFIG_NXP_WIFI_SOFTAP_SUPPORT

// static int nxp_wifi_start_ap(const struct device *dev, struct wifi_connect_req_params *params);
// static int nxp_wifi_stop_ap(const struct device *dev);
// static int nxp_wifi_ap_config_params(const struct device *dev, struct wifi_ap_config_params *params);
// #endif

// static int nxp_wifi_process_results(unsigned int count);

// static int nxp_wifi_scan(const struct device *dev, struct wifi_scan_params *params,
// 			 scan_result_cb_t cb);

// static int nxp_wifi_version(const struct device *dev, struct wifi_version *params);

// static int nxp_wifi_connect(const struct device *dev, struct wifi_connect_req_params *params);

// static int nxp_wifi_disconnect(const struct device *dev);

// static int nxp_wifi_status(const struct device *dev, struct wifi_iface_status *status);
// #if defined(CONFIG_NET_STATISTICS_WIFI)
// static int nxp_wifi_stats(const struct device *dev, struct net_stats_wifi *stats);
// #endif

// #ifdef CONFIG_NXP_WIFI_STA_AUTO_CONN
// static void nxp_wifi_auto_connect(void);
// #endif

// static int nxp_wifi_11k_cfg(const struct device *dev, struct wifi_11k_params *params);

// static int nxp_wifi_power_save(const struct device *dev, struct wifi_ps_params *params);

// int nxp_wifi_get_power_save(const struct device *dev, struct wifi_ps_config *config);

// static int nxp_wifi_reg_domain(const struct device *dev, struct wifi_reg_domain *reg_domain);

// #ifdef CONFIG_NXP_WIFI_11AX_TWT
// static int nxp_wifi_set_twt(const struct device *dev, struct wifi_twt_params *params);
// #endif

// static void nxp_wifi_sta_init(struct net_if *iface);

// #ifdef CONFIG_NXP_WIFI_SOFTAP_SUPPORT

// static void nxp_wifi_uap_init(struct net_if *iface);

// #endif

// static NXP_WIFI_SET_FUNC_ATTR int nxp_wifi_send(const struct device *dev, struct net_pkt *pkt);

// static NXP_WIFI_SET_FUNC_ATTR int nxp_wifi_recv(struct net_if *iface, struct net_pkt *pkt);

// #ifdef CONFIG_NXP_RW610
// extern void WL_MCI_WAKEUP0_DriverIRQHandler(void);
// extern void WL_MCI_WAKEUP_DONE0_DriverIRQHandler(void);
// #endif

// static int nxp_wifi_dev_init(const struct device *dev);

// static int nxp_wifi_set_config(const struct device *dev, enum ethernet_config_type type,
// 			       const struct ethernet_config *config);

// #if defined(CONFIG_PM_DEVICE) && defined(CONFIG_NXP_RW610)
// void device_pm_dump_wakeup_source(void);

// static int device_wlan_pm_action(const struct device *dev, enum pm_device_action pm_action);
// #endif

// static const struct wifi_mgmt_ops nxp_wifi_sta_mgmt = {
// 	.get_version = nxp_wifi_version,
// 	.scan = nxp_wifi_scan,
// 	.connect = nxp_wifi_connect,
// 	.disconnect = nxp_wifi_disconnect,
// 	.reg_domain = nxp_wifi_reg_domain,
// #ifdef CONFIG_NXP_WIFI_SOFTAP_SUPPORT
// 	.ap_enable = nxp_wifi_start_ap,
// 	.ap_disable = nxp_wifi_stop_ap,
// #endif
// 	.iface_status = nxp_wifi_status,
// #if defined(CONFIG_NET_STATISTICS_WIFI)
// 	.get_stats = nxp_wifi_stats,
// #endif
// 	.cfg_11k = nxp_wifi_11k_cfg,
// 	.set_power_save = nxp_wifi_power_save,
// 	.get_power_save_config = nxp_wifi_get_power_save,
// #ifdef CONFIG_NXP_WIFI_11AX_TWT
// 	.set_twt = nxp_wifi_set_twt,
// #endif
// };

// #if defined(CONFIG_WIFI_NM_WPA_SUPPLICANT)
// static const struct zep_wpa_supp_dev_ops nxp_wifi_drv_ops = {
// 	.init = wifi_nxp_wpa_supp_dev_init,
// 	.deinit                   = wifi_nxp_wpa_supp_dev_deinit,
// 	.scan2                    = wifi_nxp_wpa_supp_scan2,
// 	.scan_abort               = wifi_nxp_wpa_supp_scan_abort,
// 	.get_scan_results2        = wifi_nxp_wpa_supp_scan_results_get,
// 	.deauthenticate           = wifi_nxp_wpa_supp_deauthenticate,
// 	.authenticate             = wifi_nxp_wpa_supp_authenticate,
// 	.associate                = wifi_nxp_wpa_supp_associate,
// 	.set_key                  = wifi_nxp_wpa_supp_set_key,
// 	.set_supp_port            = wifi_nxp_wpa_supp_set_supp_port,
// 	.signal_poll              = wifi_nxp_wpa_supp_signal_poll,
// 	.send_mlme                = wifi_nxp_wpa_supp_send_mlme,
// 	.get_wiphy                = wifi_nxp_wpa_supp_get_wiphy,
// 	.get_capa                 = wifi_nxp_wpa_supp_get_capa,
// 	.get_conn_info            = wifi_nxp_wpa_supp_get_conn_info,
// 	.set_country              = wifi_nxp_wpa_supp_set_country,
// 	.get_country              = wifi_nxp_wpa_supp_get_country,
// #ifdef CONFIG_NXP_WIFI_SOFTAP_SUPPORT
// #ifdef CONFIG_WIFI_NM_HOSTAPD_AP
// 	.hapd_init                = wifi_nxp_hostapd_dev_init,
// 	.hapd_deinit              = wifi_nxp_hostapd_dev_deinit,
// #endif
// 	.init_ap                  = wifi_nxp_wpa_supp_init_ap,
// 	.set_ap                   = wifi_nxp_hostapd_set_ap,
// 	.stop_ap                  = wifi_nxp_hostapd_stop_ap,
// 	.sta_remove               = wifi_nxp_hostapd_sta_remove,
// 	.sta_add                  = wifi_nxp_hostapd_sta_add,
// 	.do_acs                   = wifi_nxp_hostapd_do_acs,
// #endif
// 	.dpp_listen               = wifi_nxp_wpa_dpp_listen,
// 	.remain_on_channel        = wifi_nxp_wpa_supp_remain_on_channel,
// 	.cancel_remain_on_channel = wifi_nxp_wpa_supp_cancel_remain_on_channel,
// };
// #endif

// static const struct net_wifi_mgmt_offload nxp_wifi_sta_apis = {
// 	.wifi_iface.iface_api.init = nxp_wifi_sta_init,
// 	.wifi_iface.set_config = nxp_wifi_set_config,
// 	.wifi_iface.send = nxp_wifi_send,
// 	.wifi_mgmt_api = &nxp_wifi_sta_mgmt,
// #if defined(CONFIG_WIFI_NM_WPA_SUPPLICANT)
// 	.wifi_drv_ops = &nxp_wifi_drv_ops,
// #endif
// };

// NET_DEVICE_INIT_INSTANCE(wifi_nxp_sta, "ml", 0, nxp_wifi_dev_init, PM_DEVICE_DT_INST_GET(0),
// 			 &g_mlan,
// #ifdef CONFIG_WIFI_NM_WPA_SUPPLICANT
// 			 &wpa_supp_ops,
// #else
// 			 NULL,
// #endif
// 			 CONFIG_WIFI_INIT_PRIORITY, &nxp_wifi_sta_apis, ETHERNET_L2,
// 			 NET_L2_GET_CTX_TYPE(ETHERNET_L2), NET_ETH_MTU);

// #ifdef CONFIG_NXP_WIFI_SOFTAP_SUPPORT
// static const struct wifi_mgmt_ops nxp_wifi_uap_mgmt = {
// 	.ap_enable = nxp_wifi_start_ap,
// 	.ap_disable = nxp_wifi_stop_ap,
// 	.iface_status = nxp_wifi_status,
// #if defined(CONFIG_NET_STATISTICS_WIFI)
// 	.get_stats = nxp_wifi_stats,
// #endif
// 	.set_power_save = nxp_wifi_power_save,
// 	.get_power_save_config = nxp_wifi_get_power_save,
// 	.ap_config_params = nxp_wifi_ap_config_params,
// };

// static const struct net_wifi_mgmt_offload nxp_wifi_uap_apis = {
// 	.wifi_iface.iface_api.init = nxp_wifi_uap_init,
// 	.wifi_iface.set_config = nxp_wifi_set_config,
// 	.wifi_iface.send = nxp_wifi_send,
// 	.wifi_mgmt_api = &nxp_wifi_uap_mgmt,
// #if defined(CONFIG_WIFI_NM_WPA_SUPPLICANT)
// 	.wifi_drv_ops = &nxp_wifi_drv_ops,
// #endif
// };
// #endif

// #ifdef __cplusplus
// }
// #endif
