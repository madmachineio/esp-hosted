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
#include "ctrl_api.h"

#define DEV_STA 0
#define DEV_AP 1
#define DEV_NUM 2

#define CTRL_CMD_DEFAULT_REQ() {					\
		.msg_type = CTRL_REQ,					\
		.ctrl_resp_cb = NULL,					\
		.cmd_timeout_sec = DEFAULT_CTRL_RESP_TIMEOUT /*30 sec*/	\
}

#define CLEANUP_CTRL_MSG(msg) do {							\
		if (msg) {								\
			if (msg->free_buffer_handle) {					\
				if (msg->free_buffer_func) {				\
					msg->free_buffer_func(msg->free_buffer_handle);	\
					msg->free_buffer_handle = NULL;			\
				}							\
			}								\
			free(msg);							\
			msg = NULL;							\
		}									\
} while (0);

#define container_of(ptr, type, member) ({			      \
		const __typeof(((type *)0)->member) * __mptr = (ptr); \
		(type *)((char *)__mptr - offsetof(type, member)); })


static mad_esp_wifi_scan_list_t *wifi_list;
static int wifi_ap_count = 0;
static void (*if_evt_handler)(mad_esp_event_t, void *);
static void *if_evt_param = NULL;

struct mad_esp_wifi_data {
	struct network_handle *handle;
	int (*rx)(unsigned char *, unsigned int);
	unsigned char mac[MAX_MAC_STR_LEN];
};

struct mad_esp_wifi_data wifi_data[DEV_NUM];

static void wifi_recv(struct network_handle *handle)
{
	struct mad_esp_wifi_data *wifi_if = NULL; //= container_of(handle, struct mad_esp_wifi_data, handle);
	struct pbuf *rx_buffer = NULL;

	for(int i=0; i<DEV_NUM; i++){
		if(wifi_data[i].handle == handle){
			wifi_if = &wifi_data[i];
			break;
		}
	}

	//printf("recv handle %p find %p\n", handle, wifi_if->handle);

	if(wifi_if == NULL){
		printf("no find handle\n");
		return;
	}
	rx_buffer = network_read(wifi_if->handle, 0);
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

static wifi_auth_mode_e wifi_encryption_mode_covert(mad_esp_wifi_encryption_mode_t mode)
{
	wifi_auth_mode_e ret;

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
		printf("get event %d\n", event);
		//control_path_platform_init();
		init_hosted_control_lib();
		if_evt_handler(MAD_ESP_HW_LINK_EVT_ACTIVE, if_evt_param);
		break;
	}
	default:
		if_evt_handler(event, if_evt_param);
		break;
	}
}

static int resp_check(ctrl_cmd_t *resp)
{
	if (!resp || (resp->msg_type != CTRL_RESP)) {
		if (resp) {
			printf("Msg type is not response[%u]\n\r", resp->msg_type);
		}
		return FAILURE;
	}

	if ((resp->msg_id <= CTRL_RESP_BASE) || (resp->msg_id >= CTRL_RESP_MAX)) {
		printf("Response Msg ID[%u] is not correct\n\r", resp->msg_id);
		return FAILURE;
	}

	if (resp->resp_event_status != SUCCESS) {
		printf("resp_event_status[%u]\n\r", resp->resp_event_status);
		return FAILURE;
	}

	return SUCCESS;
}

static void resp_clean(ctrl_cmd_t *resp)
{
	CLEANUP_CTRL_MSG(resp);
}

static int resp_get_mac(ctrl_cmd_t *resp, char *mac)
{
	if (resp->msg_id == CTRL_RESP_GET_MAC_ADDR) {
		memcpy(mac, resp->u.wifi_mac.mac, MAX_MAC_STR_LEN);
		return SUCCESS;
	}

	return FAILURE;
}

static int resp_set_station(ctrl_cmd_t *resp)
{
	if (resp->msg_id == CTRL_RESP_SET_WIFI_MODE) {
		return SUCCESS;
	}

	return FAILURE;
}


static int resp_connect_ap(ctrl_cmd_t *resp, bool *connected)
{
	*connected = false;

	if (resp->msg_id == CTRL_RESP_CONNECT_AP) {
		*connected = true;
	}

	return SUCCESS;
}

static int resp_disconnect_ap(ctrl_cmd_t *resp, bool *disconnected)
{
	*disconnected = false;

	if (resp->msg_id == CTRL_RESP_DISCONNECT_AP) {
		*disconnected = true;
	}

	return SUCCESS;
}

static int resp_get_sta_info(ctrl_cmd_t *resp, mad_esp_wifi_sta_info_t *info)
{
	if (resp->msg_id == CTRL_RESP_GET_AP_CONFIG) {
		wifi_ap_config_t *p = &resp->u.wifi_ap_config;
		info->channel = p->channel;
		info->rssi = p->rssi;
		info->encryption_mode = p->encryption_mode;
		strcpy(info->bssid, p->bssid);
		strcpy(info->ssid, p->ssid);
		return SUCCESS;
	}

	return FAILURE;
}

static int resp_start_softap(ctrl_cmd_t *resp)
{
	if (resp->msg_id == CTRL_RESP_START_SOFTAP) {
		return SUCCESS;
	}

	return FAILURE;
}

static int resp_stop_softap(ctrl_cmd_t *resp)
{
	if (resp->msg_id == CTRL_RESP_STOP_SOFTAP) {
		return SUCCESS;
	}

	return FAILURE;
}

static int resp_set_priv_cmd(ctrl_cmd_t *resp)
{
	if (resp->msg_id == CTRL_RESP_SET_PRIV_CMD) {
		return SUCCESS;
	}

	return FAILURE;
}


static mad_esp_wifi_scan_list_t *resp_scan_list(ctrl_cmd_t *resp, int *ap_count)
{
	if (resp->msg_id != CTRL_RESP_GET_AP_SCAN_LIST) {
		return NULL;
	}


	wifi_ap_scan_list_t *w_scan_p = &resp->u.wifi_ap_scan;
	wifi_scanlist_t *list = w_scan_p->out_list;

	if (!w_scan_p->count) {
		printf("No AP found\n\r");
		*ap_count = 0;
		return NULL;
	}
	if (!list) {
		printf("Failed to get scanned AP list\n\r");
		*ap_count = 0;
		return NULL;
	} else {
		*ap_count = w_scan_p->count;
		if (*ap_count > wifi_ap_count) {
			wifi_ap_count = *ap_count;
			wifi_list = realloc(wifi_list, wifi_ap_count * sizeof(mad_esp_wifi_scan_list_t));
		}

		printf("Number of available APs is %d\n\r", w_scan_p->count);
		if (wifi_list) {
			for (int i = 0; i < wifi_ap_count; i++) {
				wifi_list[i].rssi = list[i].rssi;
				wifi_list[i].channel = list[i].channel;
				wifi_list[i].encryption_mode = list[i].encryption_mode;
				strcpy(wifi_list[i].ssid, list[i].ssid);
				strcpy(wifi_list[i].bssid, list[i].bssid);
			}
		}

		return wifi_list;
	}
}


int mad_esp_init(mad_esp_interface *esp_if,
		 void (*evt_handler)(mad_esp_event_t, void *),
		 void *param)
{
	swiftio_spi_reset(esp_if->reset_gpio);

	network_init();
	if_evt_handler = evt_handler;
	if_evt_param = param;
	swiftio_spi_init(esp_if->spi,
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
	ctrl_cmd_t connect_req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t mac_req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t sta_req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;
	bool connected = false;

	wifi_data[DEV_STA].rx = rx;
	wifi_data[DEV_STA].handle = network_open(STA_INTERFACE, wifi_recv);
	if (wifi_data[DEV_STA].handle == NULL) {
		return -EIO;
	}

	printf("open handle %p\n", wifi_data[DEV_STA].handle);

	sta_req.u.wifi_mode.mode = WIFI_MODE_STA;
	resp = wifi_set_mode(sta_req);
	ret = resp_check(resp);
	if (ret) {
		return ret;
	}

	ret = resp_set_station(resp);
	resp_clean(resp);

	if (ret) {
		return ret;
	}

	printf("set to sta pass\n");
	
	mac_req.u.wifi_mac.mode = WIFI_MODE_STA;
	resp = wifi_get_mac(mac_req);
	ret = resp_check(resp);
	if (ret) {
		printf("%s %d\n", __FILE__, __LINE__);
		return ret;
	}

	ret = resp_get_mac(resp, wifi_data[DEV_STA].mac);

	resp_clean(resp);

	if (ret) {
		printf("get mac fail\n");
		return ret;
	}

	strcpy((char *)&connect_req.u.wifi_ap_config.ssid, ssid);
	strcpy((char *)&connect_req.u.wifi_ap_config.pwd, password);
	connect_req.u.wifi_ap_config.is_wpa3_supported = true;
	connect_req.u.wifi_ap_config.listen_interval = true;
	connect_req.u.wifi_ap_config.encryption_mode = wifi_encryption_mode_covert(encryption_mode);

	resp = NULL;
	resp = wifi_connect_ap(connect_req);
	ret = resp_check(resp);
	if (ret) {
		printf("%s %d\n", __FILE__, __LINE__);
		return ret;
	}

	ret = resp_connect_ap(resp, &connected);

	resp_clean(resp);

	if (connected == true) {
		ret = 0;
	} else  {
		ret = -1;
	}


	return ret;
}


int mad_esp_sta_close(void)
{
	struct mad_esp_wifi_data *wifi_if = &wifi_data[DEV_STA];
	void *handle = wifi_if->handle;
	int ret = 0;
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;
	bool disconnected = true;

	if (wifi_if->handle == NULL) {
		return -1;
	}

	resp = wifi_disconnect_ap(req);

	ret = resp_check(resp);
	if (ret) {
		return ret;
	}

	ret = resp_disconnect_ap(resp, &disconnected);

	resp_clean(resp);

	if (disconnected) {
		ret = 0;
		network_close(handle);
		wifi_if->handle = NULL;
	} else  {
		ret = -1;
	}

	return;
}

int mad_esp_sta_mac_get(char *mac)
{
	printf("esp sta mac %s\n", wifi_data[DEV_STA].mac);
	return convert_mac_to_bytes(mac, wifi_data[DEV_STA].mac);
}


int mad_esp_sta_tx(unsigned char *data, unsigned int len)
{
	struct mad_esp_wifi_data *wifi_if = &wifi_data[DEV_STA];
	struct pbuf *buffer = NULL;
	int ret;

	if (wifi_if->handle == NULL) {
		return -1;
	}

	buffer = malloc(sizeof(struct pbuf));
	if (buffer == NULL) {
		return -ENOMEM;
	}

	buffer->payload = malloc(len);
	if (buffer->payload == NULL) {
		free(buffer);
		return -ENOMEM;
	}


	buffer->len = len;

	memcpy(buffer->payload, data, len);

	ret = network_write(wifi_if->handle, buffer);

	return ret;
}


int mad_esp_sta_info(mad_esp_wifi_sta_info_t *info)
{
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;
	int ret = 0;

	resp = wifi_get_ap_config(req);

	ret = resp_check(resp);
	if (ret) {
		return ret;
	}

	ret = resp_get_sta_info(resp, info);

	resp_clean(resp);

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
	ctrl_cmd_t open_req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t mac_req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	wifi_data[DEV_AP].rx = rx;
	wifi_data[DEV_AP].handle = network_open(SOFTAP_INTERFACE, wifi_recv);
	if (wifi_data[DEV_AP].handle == NULL) {
		return -EIO;
	}

	mac_req.u.wifi_mac.mode = WIFI_MODE_AP;
	resp = wifi_get_mac(mac_req);
	ret = resp_check(resp);
	if (ret) {
		return ret;
	}

	ret = resp_get_mac(resp, wifi_data[DEV_AP].mac);

	resp_clean(resp);

	if (ret) {
		return ret;
	}

	strncpy((char *)&open_req.u.wifi_softap_config.ssid,
		ssid, MAX_MAC_STR_LEN - 1);
	strncpy((char *)&open_req.u.wifi_softap_config.pwd,
		pwd, MAX_MAC_STR_LEN - 1);
	open_req.u.wifi_softap_config.channel = channel;
	open_req.u.wifi_softap_config.encryption_mode = wifi_encryption_mode_covert(encryption_mode);
	open_req.u.wifi_softap_config.max_connections = max_connections;
	open_req.u.wifi_softap_config.ssid_hidden = ssid_hidden;
	open_req.u.wifi_softap_config.bandwidth = bandwidth;

	resp = wifi_start_softap(open_req);

	ret = resp_check(resp);
	if (ret) {
		return ret;
	}

	ret = resp_start_softap(resp);

	resp_clean(resp);

	return ret;
}


int mad_esp_ap_close(void)
{
	struct mad_esp_wifi_data *wifi_if = &wifi_data[DEV_AP];
	void *handle = wifi_if->handle;
	int ret = 0;
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	if (wifi_if->handle == NULL) {
		return -1;
	}

	resp = wifi_stop_softap(req);

	resp = resp_stop_softap(resp);
	ret = resp_check(resp);
	if (ret) {
		return ret;
	}

	ret = resp_stop_softap(resp);

	resp_clean(resp);


	return network_close(handle);
}


int mad_esp_ap_mac_get(char *mac)
{
	return convert_mac_to_bytes(mac, wifi_data[DEV_AP].mac);
}



int mad_esp_ap_tx(unsigned char *data, unsigned int len)
{
	struct mad_esp_wifi_data *wifi_if = &wifi_data[DEV_AP];
	struct pbuf *buffer = NULL;
	int ret;

	if (wifi_if->handle == NULL) {
		return -1;
	}

	buffer = malloc(sizeof(struct pbuf));
	if (buffer == NULL) {
		return -ENOMEM;
	}

	buffer->payload = malloc(len);
	if (buffer->payload == NULL) {
		free(buffer);
		return -ENOMEM;
	}


	buffer->len = len;

	memcpy(buffer->payload, data, len);

	ret = network_write(wifi_if->handle, buffer);

	return ret;
}



const mad_esp_wifi_scan_list_t *mad_esp_wifi_scan_list(int *ap_count)
{

	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();

	req.cmd_timeout_sec = 300;
	int ret = 0;
	mad_esp_wifi_scan_list_t *list;

	ctrl_cmd_t *resp = NULL;

	resp = wifi_ap_scan_list(req);
	ret = resp_check(resp);
	if (ret) {
		return ret;
	}

	list = resp_scan_list(resp, ap_count);

	resp_clean(resp);

	return list;
}


int mad_swift_set_priv(int cmd, unsigned char *data, int length)
{
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;
	int ret = 0;

	req.u.priv_cmd.cmd = cmd;
	req.u.priv_cmd.data = data;
	req.u.priv_cmd.length = length;

	resp = set_priv_cmd(req);

	ret = resp_check(resp);
	if (ret) {
		return ret;
	}

	ret = resp_set_priv_cmd(resp);

	resp_clean(resp);

	return ret;
}




