// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Portting for swiftio 2022 - Modify by Madmachine
/** Includes **/
#include <stdlib.h>
#include "string.h"
#include "trace.h"
#include "spi_drv.h"
#include "adapter.h"
#include "serial_drv.h"
#include "netdev_if.h"

#include "hosted_spi.h"
#include "swift_gpio.h"

/** Constants/Macros **/
#define SPI_LOGD
#define SPI_LOGI        printf
#define SPI_LOGE        printf


#define SEND_QUEUE_SIZE                 32
#define RECV_QUEUE_SIZE                 32

#define RECV_TASK_STACK_SIZE            (64 * 1024)
#define TRANS_TASK_STACK_SIZE           (4 * 1024)

#define RECV_TASK_PRIORITY                      -2
#define TRANS_TASK_PRIORITY                     -3


#define MAX_PAYLOAD_SIZE (MAX_SPI_BUFFER_SIZE-sizeof(struct esp_payload_header))

static int esp_netdev_open(netdev_handle_t netdev);
static int esp_netdev_close(netdev_handle_t netdev);
static int esp_netdev_xmit(netdev_handle_t netdev, struct pbuf *net_buf);
static int if_transaction(uint8_t *txbuff);

static struct esp_private *esp_priv[MAX_NETWORK_INTERFACES];

static struct netdev_ops esp_net_ops = {
	.netdev_open = esp_netdev_open,
	.netdev_close = esp_netdev_close,
	.netdev_xmit = esp_netdev_xmit,
};

/** Exported variables **/
static void *trans_sem = NULL;
static void *trans_mutex = NULL;
static void *sendtoesp_queue = NULL;
static void *recvfromesp_queue = NULL;

static void *trans_thread = NULL;
static void *recv_thread = NULL;

static void *esp_hand_gpio = NULL;
static void *esp_ready_gpio = NULL;
static void *esp_reset_gpio = NULL;

/* callback of event handler */
static void (*spi_drv_evt_handler_fp) (uint8_t);

/** function declaration **/
/** Exported functions **/
static void transaction_task(void *arg, void *p2, void *p3);
static void process_rx_task(void *arg, void *p2, void *p3);

static uint8_t * get_tx_buffer(uint8_t *is_valid_tx_buf);
static void deinit_netdev(void);

/**
  * @brief  get private interface of expected type and number
  * @param  if_type - interface type
  *         if_num - interface number
  * @retval interface handle if found, else NULL
  */
static struct esp_private * get_priv(uint8_t if_type, uint8_t if_num)
{
	int i = 0;

	for (i = 0; i < MAX_NETWORK_INTERFACES; i++) {
		if((esp_priv[i]) &&
			(esp_priv[i]->if_type == if_type) &&
			(esp_priv[i]->if_num == if_num))
			return esp_priv[i];
	}

	return NULL;
}

/**
  * @brief  open virtual network device
  * @param  netdev - network device
  * @retval 0 on success
  */
static int esp_netdev_open(netdev_handle_t netdev)
{
	return SWIFTIO_OK;
}

/**
  * @brief  close virtual network device
  * @param  netdev - network device
  * @retval 0 on success
  */
static int esp_netdev_close(netdev_handle_t netdev)
{
	return SWIFTIO_OK;
}

/**
  * @brief  transmit on virtual network device
  * @param  netdev - network device
  *         net_buf - buffer to transmit
  * @retval None
  */
static int esp_netdev_xmit(netdev_handle_t netdev, struct pbuf *net_buf)
{
	struct esp_private *priv;
	int ret;

	if (!netdev || !net_buf)
		return SWIFTIO_FAIL;
	priv = (struct esp_private *) netdev_get_priv(netdev);

	if (!priv)
		return SWIFTIO_FAIL;

	ret = send_to_slave(priv->if_type, priv->if_num,
			net_buf->payload, net_buf->len);
	free(net_buf);

	return ret;
}

/**
  * @brief  create virtual network device
  * @param  None
  * @retval None
  */
static int init_netdev(void)
{
	void *ndev = NULL;
	int i = 0;
	struct esp_private *priv = NULL;
	char *if_name = STA_INTERFACE;
	uint8_t if_type = ESP_STA_IF;

	for (i = 0; i < MAX_NETWORK_INTERFACES; i++) {
		/* Alloc and init netdev */
		ndev = netdev_alloc(sizeof(struct esp_private), if_name);
		if (!ndev) {
			deinit_netdev();
			return SWIFTIO_FAIL;
		}

		priv = (struct esp_private *) netdev_get_priv(ndev);
		if (!priv) {
			deinit_netdev();
			return SWIFTIO_FAIL;
		}

		priv->netdev = ndev;
		priv->if_type = if_type;
		priv->if_num = 0;

		if (netdev_register(ndev, &esp_net_ops)) {
			deinit_netdev();
			return SWIFTIO_FAIL;
		}

		if_name = SOFTAP_INTERFACE;
		if_type = ESP_AP_IF;

		esp_priv[i] = priv;
	}

	return SWIFTIO_OK;
}

/**
  * @brief  destroy virtual network device
  * @param  None
  * @retval None
  */
static void deinit_netdev(void)
{
	for (int i = 0; i < MAX_NETWORK_INTERFACES; i++) {
		if (esp_priv[i]) {
			if (esp_priv[i]->netdev) {
				netdev_unregister(esp_priv[i]->netdev);
				netdev_free(esp_priv[i]->netdev);
			}
			esp_priv[i] = NULL;
		}
	}
}

/**
  * @brief detection callback, used as SPI handshake GPIO
  * @param NA
  * @retval None
  */
void esp_device_if_isr(void *param)
{
	SPI_LOGD("get hand or ready %p\r\n", param);
	if (trans_sem) {
		SPI_LOGD("send sem\r\n");
		swifthal_os_sem_give(trans_sem);
	}
}


/** function definition **/

/** Exported Functions **/

void swiftio_spi_reset(void *reset_gpio)
{
	if (reset_gpio != NULL) {
		esp_reset_gpio = reset_gpio;
	}

	SPI_LOGI("esp_reset_gpio %p\n", esp_reset_gpio);
	if (esp_reset_gpio != NULL) {
		swifthal_gpio_set(esp_reset_gpio, 0);
		swifthal_ms_sleep(100);
		swifthal_gpio_set(esp_reset_gpio, 1);
	}
}

/**
  * @brief  spi driver initialize
  * @param  spi_drv_evt_handler - event handler of type spi_drv_events_e
  * @retval None
  */
void swiftio_spi_init(void *spi,
			void *spi_cs_gpio,
			void *hand_gpio,
			void *ready_gpio,
			void(*spi_drv_evt_handler)(uint8_t))
{
	swiftio_ret_t retval = SWIFTIO_OK;

	esp_hand_gpio = hand_gpio;
	esp_ready_gpio = ready_gpio;
	

	/* register callback */
	spi_drv_evt_handler_fp = spi_drv_evt_handler;
	
	/* spi handshake semaphore */
	trans_sem = swifthal_os_sem_create(1, 1000);
	if (trans_sem == NULL) {
		SPI_LOGE("[sem] no mem \r\n");
		return;
	}

	SPI_LOGI("create trans_sem %x\r\n", trans_sem);

	retval = init_netdev();
	if (retval) {
		printf("netdev failed to init\n\r");
		assert(retval==SWIFTIO_OK);
	}

	trans_mutex = swifthal_os_mutex_create();
	if (trans_mutex == NULL) {
		SPI_LOGE("[mutex] no mem \r\n");
		return;
	}

	/* Queue - tx */
	
	SPI_LOGI("create trans_mutex %x\r\n", trans_mutex);
	
	sendtoesp_queue = swifthal_os_mq_create(sizeof(interface_buffer_handle_t), SEND_QUEUE_SIZE);
	if (sendtoesp_queue == NULL) {
		SPI_LOGE("[sendtoesp_queue] no mem \r\n");
		return;
	}

	SPI_LOGI("create sendtoesp_queue %x\r\n", sendtoesp_queue);

	recvfromesp_queue = swifthal_os_mq_create(sizeof(interface_buffer_handle_t), RECV_QUEUE_SIZE);
	if (recvfromesp_queue == NULL) {
		SPI_LOGE("[recvfromesp_queue] no mem \r\n");
		return;
	}

	SPI_LOGI("create recvfromesp_queue %x\r\n", recvfromesp_queue);
	

	trans_thread = swifthal_os_task_create("transaction_task",
					       transaction_task,
					       NULL, NULL, NULL,
					       TRANS_TASK_PRIORITY,
					       TRANS_TASK_STACK_SIZE);

	SPI_LOGI("create trans_thread %x\r\n", trans_thread);

	recv_thread = swifthal_os_task_create("process_rx_task",
					      process_rx_task,
					      NULL, NULL, NULL,
					      RECV_TASK_PRIORITY,
					      RECV_TASK_STACK_SIZE);

	SPI_LOGI("create recv_thread %x\r\n", recv_thread);

	swifthal_gpio_interrupt_config(esp_hand_gpio, SWIFT_GPIO_INT_MODE_RISING_EDGE);
	swifthal_gpio_interrupt_config(esp_ready_gpio, SWIFT_GPIO_INT_MODE_RISING_EDGE);
	swifthal_gpio_interrupt_callback_install(esp_hand_gpio, esp_hand_gpio, esp_device_if_isr);
	swifthal_gpio_interrupt_callback_install(esp_ready_gpio, esp_ready_gpio, esp_device_if_isr);
	SPI_LOGI("enable esp_hand_gpio %x esp_ready_gpio %x\r\n", esp_hand_gpio, esp_ready_gpio);
	swifthal_gpio_interrupt_enable(esp_hand_gpio);
	swifthal_gpio_interrupt_enable(esp_ready_gpio);

	SPI_LOGI("enable int \r\n");
}


/**
  * @brief  Schedule SPI transaction if -
  *         a. valid TX buffer is ready at SPI host (STM)
  *         b. valid TX buffer is ready at SPI peripheral (ESP)
  *         c. Dummy transaction is expected from SPI peripheral (ESP)
  * @param  argument: Not used
  * @retval None
  */

static void check_and_execute_spi_transaction(void)
{
	uint8_t * txbuff = NULL;
	uint8_t is_valid_tx_buf = 0;
	int gpio_handshake = 0;
	int gpio_rx_data_ready = 0;


	/* handshake line SET -> slave ready for next transaction */
	gpio_handshake = swifthal_gpio_get(esp_hand_gpio);

	/* data ready line SET -> slave wants to send something */
	gpio_rx_data_ready = swifthal_gpio_get(esp_ready_gpio);

	if (gpio_handshake == 1) {

		/* Get next tx buffer to be sent */
		txbuff = get_tx_buffer(&is_valid_tx_buf);

		if ((gpio_rx_data_ready == 1) ||
		    (is_valid_tx_buf)) {

			/* Execute transaction only if EITHER holds true-
			 * a. A valid tx buffer to be transmitted towards slave
			 * b. Slave wants to send something (Rx for host)
			 */
			
			swifthal_os_mutex_lock(trans_mutex, -1);
			if_transaction(txbuff);
			swifthal_os_mutex_unlock(trans_mutex);
		}
	}
}

/**
  * @brief  Send to slave via SPI
  * @param  iface_type -type of interface
  *         iface_num - interface number
  *         wbuffer - tx buffer
  *         wlen - size of wbuffer
  * @retval sendbuf - Tx buffer
  */
swiftio_ret_t send_to_slave(uint8_t iface_type, uint8_t iface_num,
		uint8_t * wbuffer, uint16_t wlen)
{
	interface_buffer_handle_t buf_handle = {0};

	if (!wbuffer || !wlen || (wlen > MAX_PAYLOAD_SIZE)) {
		printf("write fail: buff(%p) 0? OR (0<len(%u)<=max_poss_len(%u))?\n\r",
				wbuffer, wlen, MAX_PAYLOAD_SIZE);
		if(wbuffer) {
			free(wbuffer);
			wbuffer = NULL;
		}
		return SWIFTIO_FAIL;
	}
	memset(&buf_handle, 0, sizeof(buf_handle));

	buf_handle.if_type = iface_type;
	buf_handle.if_num = iface_num;
	buf_handle.payload_len = wlen;
	buf_handle.payload = wbuffer;
	buf_handle.priv_buffer_handle = wbuffer;
	buf_handle.free_buf_handle = free;

	swifthal_os_mq_send(sendtoesp_queue, &buf_handle, -1);

	check_and_execute_spi_transaction();

	return SWIFTIO_OK;
}

/** Local functions **/

/**
  * @brief  Full duplex transaction SPI transaction for ESP32 hardware
  * @param  txbuff: TX SPI buffer
  * @retval SWIFTIO_OK / SWIFTIO_FAIL
  */
static int if_transaction(uint8_t *txbuff)
{
	uint8_t *rxbuff = NULL;
	interface_buffer_handle_t buf_handle = {0};
	struct  esp_payload_header *payload_header;
	uint16_t len, offset;
	uint16_t rx_checksum = 0, checksum = 0;

	/* Allocate rx buffer */
	rxbuff = (uint8_t *)malloc(MAX_SPI_BUFFER_SIZE);
	assert(rxbuff);
	memset(rxbuff, 0, MAX_SPI_BUFFER_SIZE);

	if(!txbuff) {
		/* Even though, there is nothing to send,
		 * valid resetted txbuff is needed for SPI driver
		 */
		txbuff = (uint8_t *)malloc(MAX_SPI_BUFFER_SIZE);
		assert(txbuff);
		memset(txbuff, 0, MAX_SPI_BUFFER_SIZE);
	}

	/* SPI transaction */
	int retval = hosted_spi_transceive((uint8_t *) txbuff, (uint8_t *) rxbuff, MAX_SPI_BUFFER_SIZE,
					   -1);

	switch(retval)
	{
		case 0:

			/* Transaction successful */

			/* create buffer rx handle, used for processing */
			payload_header = (struct esp_payload_header *) rxbuff;

			/* Fetch length and offset from payload header */
			len = le16toh(payload_header->len);
			offset = le16toh(payload_header->offset);

			if ((!len) ||
				(len > MAX_PAYLOAD_SIZE) ||
				(offset != sizeof(struct esp_payload_header))) {

				/* Free up buffer, as one of following -
				 * 1. no payload to process
				 * 2. input packet size > driver capacity
				 * 3. payload header size mismatch,
				 * wrong header/bit packing?
				 * */
				if (rxbuff) {
					free(rxbuff);
					rxbuff = NULL;
				}
				/* Give chance to other tasks */
				swifthal_os_task_yield();

			} else {
				rx_checksum = le16toh(payload_header->checksum);
				payload_header->checksum = 0;
				checksum = compute_checksum(rxbuff, len+offset);
				if (checksum == rx_checksum) {
					buf_handle.priv_buffer_handle = rxbuff;
					buf_handle.free_buf_handle = free;
					buf_handle.payload_len = len;
					buf_handle.if_type     = payload_header->if_type;
					buf_handle.if_num      = payload_header->if_num;
					buf_handle.payload     = rxbuff + offset;
					buf_handle.seq_num     = le16toh(payload_header->seq_num);
					buf_handle.flag        = payload_header->flags;

				SPI_LOGD("rev if and send recvfromesp_queue\n");
				if (0 != swifthal_os_mq_send(recvfromesp_queue, &buf_handle, -1)) {
					SPI_LOGE("Failed to send buffer\n\r");
					goto done;
				}
			} else {
				SPI_LOGE("rev if checksum fail\n");
					if (rxbuff) {
						free(rxbuff);
						rxbuff = NULL;
					}
				}
			}

			/* Free input TX buffer */
			if (txbuff) {
				free(txbuff);
				txbuff = NULL;
			}
			break;

	default:
		SPI_LOGE("default handler: Error in SPI transaction\n\r");
			goto done;
			break;
	}

	return SWIFTIO_OK;

done:
	/* error cases, abort */
	if (txbuff) {
		free(txbuff);
		txbuff = NULL;
	}

	if (rxbuff) {
		free(rxbuff);
		rxbuff = NULL;
	}
	return SWIFTIO_FAIL;
}



/**
  * @brief  Task for SPI transaction
  * @param  argument: Not used
  * @retval None
  */
static void transaction_task(void *arg, void *p2, void *p3)
{
	for (;;) {

		if (trans_sem != NULL) {
			/* Wait till slave is ready for next transaction */
			if (swifthal_os_sem_take(trans_sem, -1) == 0) {
				check_and_execute_spi_transaction();
			}
		}
	}
}

/**
  * @brief  RX processing task
  * @param  argument: Not used
  * @retval None
  */
static void process_rx_task(void *arg, void *p2, void *p3)
{
	interface_buffer_handle_t buf_handle = { 0 };
	uint8_t *payload = NULL;
	struct pbuf *buffer = NULL;
	struct esp_priv_event *event = NULL;
	struct esp_private *priv = NULL;

	while (1) {
		int ret = swifthal_os_mq_recv(recvfromesp_queue, &buf_handle, -1);
		if (ret != 0) {
			SPI_LOGE("recv recvfromesp_queue %d\n", ret);
			continue;
		}

		/* point to payload */
		payload = buf_handle.payload;

		/* process received buffer for all possible interface types */
		if (buf_handle.if_type == ESP_SERIAL_IF) {

			/* serial interface path */
			serial_rx_handler(&buf_handle);

		} else if((buf_handle.if_type == ESP_STA_IF) ||
				(buf_handle.if_type == ESP_AP_IF)) {
			priv = get_priv(buf_handle.if_type, buf_handle.if_num);

			if (priv) {
				buffer = (struct pbuf *)malloc(sizeof(struct pbuf));
				assert(buffer);

				buffer->len = buf_handle.payload_len;
				buffer->payload = malloc(buf_handle.payload_len);
				assert(buffer->payload);

				memcpy(buffer->payload, buf_handle.payload,
						buf_handle.payload_len);

				netdev_rx(priv->netdev, buffer);
			}

		} else if (buf_handle.if_type == ESP_PRIV_IF) {
			/* priv transaction received */

			event = (struct esp_priv_event *) (payload);
			if (event->event_type == ESP_PRIV_EVENT_INIT) {
				/* halt spi transactions for some time,
				 * this is one time delay, to give breathing
				 * time to slave before spi trans start */
				swifthal_ms_sleep(50);
				if (spi_drv_evt_handler_fp) {
					spi_drv_evt_handler_fp(SPI_DRIVER_ACTIVE);
				}
			} else {
				/* User can re-use this type of transaction */
			}
		}

		/* Free buffer handle */
		/* When buffer offloaded to other module, that module is
		 * responsible for freeing buffer. In case not offloaded or
		 * failed to offload, buffer should be freed here.
		 */
		if (buf_handle.free_buf_handle) {
			buf_handle.free_buf_handle(buf_handle.priv_buffer_handle);
		}
	}
}


/**
  * @brief  Next TX buffer in SPI transaction
  * @param  argument: Not used
  * @retval sendbuf - Tx buffer
  */
static uint8_t * get_tx_buffer(uint8_t *is_valid_tx_buf)
{
	struct  esp_payload_header *payload_header;
	uint8_t *sendbuf = NULL;
	uint8_t *payload = NULL;
	uint16_t len = 0;
	interface_buffer_handle_t buf_handle = {0};

	*is_valid_tx_buf = 0;

	/* Check if higher layers have anything to transmit, non blocking.
	 * If nothing is expected to send, queue receive will fail.
	 * In that case only payload header with zero payload
	 * length would be transmitted.
	 */
	if (swifthal_os_mq_recv(sendtoesp_queue, &buf_handle, 0) == 0) {
		len = buf_handle.payload_len;
	}

	if (len) {

		sendbuf = (uint8_t *) malloc(MAX_SPI_BUFFER_SIZE);
		if (!sendbuf) {
			SPI_LOGE("malloc failed\n\r");
			goto done;
		}

		memset(sendbuf, 0, MAX_SPI_BUFFER_SIZE);

		*is_valid_tx_buf = 1;

		/* Form Tx header */
		payload_header = (struct esp_payload_header *) sendbuf;
		payload = sendbuf + sizeof(struct esp_payload_header);
		payload_header->len     = htole16(len);
		payload_header->offset  = htole16(sizeof(struct esp_payload_header));
		payload_header->if_type = buf_handle.if_type;
		payload_header->if_num  = buf_handle.if_num;
		memcpy(payload, buf_handle.payload, min(len, MAX_PAYLOAD_SIZE));
		payload_header->checksum = htole16(compute_checksum(sendbuf,
				sizeof(struct esp_payload_header)+len));;
	}

done:
	/* free allocated buffer */
	if (buf_handle.free_buf_handle)
		buf_handle.free_buf_handle(buf_handle.priv_buffer_handle);

	return sendbuf;
}
