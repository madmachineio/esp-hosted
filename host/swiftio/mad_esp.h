/*
 * @Copyright (c) 2022, MADMACHINE LIMITED
 * @Author: Frank Li(lgl88911@163.com)
 * @SPDX-License-Identifier: MIT
 */

#ifndef _MAD_ESP_H_
#define _MAD_ESP_H_

#define MAD_ESP_WIFI_SSID_LEN       32
#define MAD_ESP_WIFI_BSSID_LEN      17
#define MAD_ESP_WIFI_PWD_LEN        64

typedef enum {
	MAD_ESP_WIFI_AUTH_OPEN = 0,             /* Authenticate mode : open */
	MAD_ESP_WIFI_AUTH_WEP,                  /* Authenticate mode : WEP */
	MAD_ESP_WIFI_AUTH_WPA_PSK,              /* Authenticate mode : WPA_PSK */
	MAD_ESP_WIFI_AUTH_WPA2_PSK,             /* Authenticate mode : WPA2_PSK */
	MAD_ESP_WIFI_AUTH_WPA_WPA2_PSK,         /* Authenticate mode : WPA_WPA2_PSK */
	MAD_ESP_WIFI_AUTH_WPA2_ENTERPRISE,      /* Authenticate mode : WPA2_ENTERPRISE */
	MAD_ESP_WIFI_AUTH_WPA3_PSK,             /* Authenticate mode : WPA3_PSK */
	MAD_ESP_WIFI_AUTH_WPA2_WPA3_PSK,        /* Authenticate mode : WPA2_WPA3_PSK */
	MAD_ESP_WIFI_AUTH_MAX,                  /* Authenticate mode : WAPI_PSK */
} mad_esp_wifi_encryption_mode_t;

typedef enum {
	MAD_ESP_HW_LINK_EVT_ACTIVE = 0,
	MAD_ESP_WIFI_EVT_AP_STACONNECTED,
	MAD_ESP_WIFI_EVT_AP_STADISCONNECTED,
	MAD_ESP_WIFI_EVT_STA_CONNECTED,
	MAD_ESP_WIFI_EVT_STA_DISCONNECTED,
} mad_esp_event_t;

typedef struct {
	void *spi;
	void *spi_cs_gpio;
	void *hand_gpio;
	void *ready_gpio;
	void *reset_gpio;
} mad_esp_interface;

typedef struct {
	char ssid[MAD_ESP_WIFI_SSID_LEN + 1];   /* Target AP SSID */
	char pwd[MAD_ESP_WIFI_PWD_LEN + 1];     /* Target AP Password */
	char bssid[MAD_ESP_WIFI_BSSID_LEN + 1]; /* Target AP bssid (MAC address) */
	unsigned char channel;                  /* Target AP channel */
	int rssi;                               /* Target AP rssi */
	int encryption_mode;                    /* Target AP encryption mode */
} mad_esp_wifi_sta_info_t;

typedef struct {
	char ssid[MAD_ESP_WIFI_SSID_LEN + 1];           /* Target AP SSID */
	char bssid[MAD_ESP_WIFI_BSSID_LEN + 1];         /* Target AP bssid (MAC address) */
	unsigned char channel;                          /* Target AP channel */
	int rssi;                                       /* Target AP rssi */
	int encryption_mode;                            /* Target AP encryption mode */
} mad_esp_wifi_scan_list_t;

int mad_esp_init(mad_esp_interface *esp_if,
		 void (*evt_handler)(mad_esp_event_t, void *),
		 void *param);

int mad_esp_sta_open(char *ssid,
		     char *password,
		     int encryption_mode,
		     int (*rx)(unsigned char *, unsigned int));

int mad_esp_sta_close(void);

int mad_esp_sta_tx(unsigned char *data, unsigned int len);

int mad_esp_sta_mac_get(char *mac);

int mad_esp_sta_info(mad_esp_wifi_sta_info_t *info);

int mad_esp_ap_open(char *ssid,
		    char *pwd,
		    int channel,
		    int bandwidth,
		    int ssid_hidden,
		    int max_connections,
		    int encryption_mode,
		    int (*rx)(unsigned char *, unsigned int));


int mad_esp_ap_close(void);

int mad_esp_ap_mac_get(char *mac);

int mad_esp_ap_tx(unsigned char *data, unsigned int len);

const mad_esp_wifi_scan_list_t *mad_esp_wifi_scan_list(int *ap_count);

#endif /* _MAD_ESP_H_ */