/*
 * @Copyright (c) 2022, MADMACHINE LIMITED
 * @Author: Frank Li(lgl88911@163.com)
 * @SPDX-License-Identifier: MIT
 */

#include <string.h>
#include <errno.h>

#include "mad_esp.h"
#include "spi_drv.h"
#include "netdev_if.h"
#include "commands.h"


#define WIFI_MAX_STR_LEN        19

static char station_mac[WIFI_MAX_STR_LEN];
static char ap_mac[WIFI_MAX_STR_LEN];
static mad_esp_wifi_scan_list_t *wifi_list;
static int wifi_ap_count = 0;
static void (*if_evt_handler)(mad_esp_event_t, void *);
static void *if_evt_param = NULL;

struct mad_esp_wifi_data {
	void *virtual_netif;
	int (*rx)(unsigned char *, unsigned int);
	unsigned char mac_addr[6];
};

struct mad_esp_wifi_data wifi_data[MAX_NETWORK_INTERFACES];

static void wifi_recv(struct network_handle *handle, void *data)
{
	struct mad_esp_wifi_data *wifi_if = data;
	struct net_pbuf *rx_buffer = NULL;

	rx_buffer = network_read(wifi_if->virtual_netif, 0);
	if (rx_buffer && wifi_if->rx) {
		wifi_if->rx(rx_buffer->payload, rx_buffer->len);

		if (rx_buffer->payload) {
			free(rx_buffer->payload);
			rx_buffer->payload = NULL;
		}
		if (rx_buffer) {
			free(rx_buffer);
			rx_buffer = NULL;
		}
	}
}

static wifi_auth_mode_t wifi_encryption_mode_covert(mad_esp_wifi_encryption_mode_t mode)
{
	wifi_auth_mode_t ret;

	switch (mode) {
	case MAD_ESP_WIFI_AUTH_OPEN:
		ret = WIFI_AUTH_OPEN;
		break;
	case MAD_ESP_WIFI_AUTH_WEP:
		ret = WIFI_AUTH_WEP;
		break;
	case MAD_ESP_WIFI_AUTH_WPA_PSK:
		ret = WIFI_AUTH_WPA_PSK;
		break;
	case MAD_ESP_WIFI_AUTH_WPA2_PSK:
		ret = WIFI_AUTH_WPA2_PSK;
		break;
	case MAD_ESP_WIFI_AUTH_WPA_WPA2_PSK:
		ret = WIFI_AUTH_WPA_WPA2_PSK;
		break;
	case MAD_ESP_WIFI_AUTH_WPA2_ENTERPRISE:
		ret = WIFI_AUTH_WPA2_ENTERPRISE;
		break;

	case MAD_ESP_WIFI_AUTH_WPA3_PSK:
		ret = WIFI_AUTH_WPA3_PSK;
		break;

	case MAD_ESP_WIFI_AUTH_WPA2_WPA3_PSK:
		ret = WIFI_AUTH_WPA2_WPA3_PSK;
		break;

	default:
		ret = WIFI_AUTH_MAX; // Not support
		break;
	}
	return ret;
}


static void esp_if_event_handler(unsigned char event)
{
	switch (event) {
	case SPI_DRIVER_ACTIVE:
	{
		/* Initiate control path now */
		control_path_platform_init();
		if_evt_handler(MAD_ESP_HW_LINK_EVT_ACTIVE, if_evt_param);
		break;
	}
	default:
		if_evt_handler(event, if_evt_param);
		break;
	}
}


int mad_esp_init(mad_esp_interface *esp_if,
		 void (*evt_handler)(mad_esp_event_t, void *),
		 void *param)
{
	esp_device_if_reset(esp_if->reset_gpio);

	network_init();
	if_evt_handler = evt_handler;
	if_evt_param = param;
	esp_device_if_init(esp_if->spi,
			   esp_if->spi_cs_gpio,
			   esp_if->hand_gpio,
			   esp_if->ready_gpio,
			   esp_if_event_handler);

	return 0;
}

int mad_esp_sta_open(char *ssid,
		     char *password,
		     int encryption_mode,
		     int (*rx)(unsigned char *, unsigned int))
{
	int ret = 0;
	esp_hosted_control_config_t esp_wifi_config = { 0 };

	wifi_data[STA_INTERFACE_ID].rx = rx;
	wifi_data[STA_INTERFACE_ID].virtual_netif = network_open(STA_INTERFACE,
								 wifi_recv,
								 &wifi_data[STA_INTERFACE_ID]);
	if (wifi_data[STA_INTERFACE_ID].virtual_netif == NULL) {
		return -EIO;
	}

	strcpy((char * )esp_wifi_config.station.ssid, ssid);
	strcpy((char * )esp_wifi_config.station.pwd, password);
	esp_wifi_config.station.encryption_mode = wifi_encryption_mode_covert(encryption_mode);

	ret = wifi_get_mac(WIFI_MODE_STA, station_mac);
	if (ret) {
		return ret;
	}
	return wifi_set_ap_config(esp_wifi_config);
}


int mad_esp_sta_close(void)
{
	struct mad_esp_wifi_data *wifi_if = &wifi_data[STA_INTERFACE_ID];
	void *virtual_netif = wifi_if->virtual_netif;

	if (wifi_if->virtual_netif == NULL) {
		return -1;
	}

	wifi_if->virtual_netif = NULL;

	wifi_disconnect_ap();

	return network_close(virtual_netif);
}

int mad_esp_sta_mac_get(char *mac)
{
	printf("esp sta mac %s\n", station_mac);
	return convert_mac_to_bytes(mac, station_mac);
}


int mad_esp_sta_tx(unsigned char *data, unsigned int len)
{
	struct mad_esp_wifi_data *wifi_if = &wifi_data[STA_INTERFACE_ID];

	if (wifi_if->virtual_netif == NULL) {
		return -1;
	}

	struct net_pbuf *tx_buf = malloc(sizeof(struct net_pbuf));
	if (tx_buf == NULL) {
		return -ENOMEM;
	}

	tx_buf->len = len;
	tx_buf->payload = malloc(len);
	if (tx_buf->payload == NULL) {
		free(tx_buf);
		return -ENOMEM;
	}

	memcpy(tx_buf->payload, data, len);

	return network_write(wifi_if->virtual_netif, tx_buf);
}


int mad_esp_sta_info(mad_esp_wifi_sta_info_t *info)
{
	esp_hosted_control_config_t connected_info = { 0 };

	memset(connected_info.station.ssid, '\0', sizeof(connected_info.station.ssid));
	int ret = wifi_get_ap_config(&connected_info);
	if (ret) {
		return ret;
	}

	info->channel = connected_info.station.channel;
	info->rssi = connected_info.station.rssi;
	info->encryption_mode = connected_info.station.encryption_mode;
	strcpy(info->bssid, connected_info.station.bssid);
	strcpy(info->ssid, connected_info.station.ssid);

	return ret;
}


int mad_esp_ap_open(char *ssid,
		    char *pwd,
		    int channel,
		    int bandwidth,
		    int ssid_hidden,
		    int max_connections,
		    int encryption_mode,
		    int (*rx)(unsigned char *, unsigned int))
{
	int ret = 0;
	esp_hosted_control_config_t esp_wifi_config = { 0 };

	wifi_data[SOFTAP_INTERFACE_ID].rx = rx;
	wifi_data[SOFTAP_INTERFACE_ID].virtual_netif = network_open(SOFTAP_INTERFACE,
								    wifi_recv,
								    &wifi_data[SOFTAP_INTERFACE_ID]);
	if (wifi_data[SOFTAP_INTERFACE_ID].virtual_netif == NULL) {
		return -EIO;
	}

	strcpy(esp_wifi_config.softap.ssid, ssid);
	strcpy(esp_wifi_config.softap.pwd, pwd);
	esp_wifi_config.softap.channel = channel;
	esp_wifi_config.softap.bandwidth = bandwidth;
	esp_wifi_config.softap.ssid_hidden = ssid_hidden;
	esp_wifi_config.softap.max_connections = max_connections;
	esp_wifi_config.softap.encryption_mode = wifi_encryption_mode_covert(encryption_mode);

	ret = wifi_get_mac(WIFI_MODE_AP, ap_mac);
	if (ret) {
		return ret;
	}
	return wifi_set_softap_config(esp_wifi_config);
}


int mad_esp_ap_close(void)
{
	struct mad_esp_wifi_data *wifi_if = &wifi_data[SOFTAP_INTERFACE_ID];
	void *virtual_netif = wifi_if->virtual_netif;

	if (wifi_if->virtual_netif == NULL) {
		return -1;
	}

	wifi_if->virtual_netif = NULL;

	wifi_stop_softap();

	return network_close(virtual_netif);
}

int mad_esp_ap_mac_get(char *mac)
{
	return convert_mac_to_bytes(mac, ap_mac);
}



int mad_esp_ap_tx(unsigned char *data, unsigned int len)
{
	struct mad_esp_wifi_data *wifi_if = &wifi_data[SOFTAP_INTERFACE_ID];

	if (wifi_if->virtual_netif == NULL) {
		return -1;
	}

	struct net_pbuf *tx_buf = malloc(sizeof(struct net_pbuf));
	if (tx_buf == NULL) {
		return -ENOMEM;
	}

	tx_buf->len = len;
	tx_buf->payload = malloc(len);
	if (tx_buf->payload == NULL) {
		free(tx_buf);
		return -ENOMEM;
	}

	memcpy(tx_buf->payload, data, len);

	return network_write(wifi_if->virtual_netif, tx_buf);
}


const mad_esp_wifi_scan_list_t *mad_esp_wifi_scan_list(int *ap_count)
{
	esp_hosted_wifi_scanlist_t *ap_list = wifi_ap_scan_list(ap_count);

	if (*ap_count > wifi_ap_count) {
		wifi_ap_count = *ap_count;
		wifi_list = realloc(wifi_list, wifi_ap_count * sizeof(esp_hosted_wifi_scanlist_t));
	}

	if (wifi_list) {
		for (int i = 0; i < wifi_ap_count; i++) {
			wifi_list[i].rssi = ap_list[i].rssi;
			wifi_list[i].channel = ap_list[i].channel;
			wifi_list[i].encryption_mode = ap_list[i].encryption_mode;
			strcpy(wifi_list[i].ssid, ap_list[i].ssid);
			strcpy(wifi_list[i].bssid, ap_list[i].bssid);
		}
	}

	return wifi_list;
}





