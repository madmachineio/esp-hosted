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

#include "trace.h"
#include "string.h"
#include "netdev_stub.h"
#include "swift_os.h"


struct netdev *ndev_db[MAX_INTERFACE];
static uint8_t ndev_index = 0;

/**
  * @brief  initialize detdev
  * @param  None
  * @retval None
  */
void netdev_init(void)
{
	int i;

	for (i = 0; i < MAX_INTERFACE; i++) {
		ndev_db[i] = NULL;
	}
}

/**
  * @brief  open netdev
  * @param  ndev - netdev
  * @retval 0 on success
  */
int netdev_open(netdev_handle_t ndev)
{
	if (!ndev)
		return -EIO;

	ndev->rx_q = swifthal_os_mq_create(sizeof(struct net_pbuf), RX_QUEUE_SIZE);
	if (ndev->rx_q == NULL) {
        printk("ndev->rx_q] no mem\n");
        return ENOMEM;
    }

	ndev->state = NETDEV_STATE_UP;
	return 0;
}

/**
  * @brief  close netdev
  * @param  ndev - netdev
  * @retval None
  */
void netdev_close(netdev_handle_t ndev)
{
	if (!ndev)
		return;

	ndev->state = NETDEV_STATE_DOWN;

    k_msleep(200);
	/* reset queue */
	swifthal_os_mq_destory(ndev->rx_q);
	ndev->rx_q = NULL;

	ndev->net_handle = NULL;
}

/**
  * @brief  get netdev handle from interface name
  * @param  if_name - interface name
  * @retval netdev handle
  */
struct netdev * netdev_get(char *if_name)
{
	int i = 0;
	struct netdev *ndev;

	if (!if_name)
		return NULL;

	while (i < MAX_INTERFACE) {
		ndev = ndev_db[i];

		if (ndev) {
			if (strncmp(if_name, ndev->name, MAX_IF_NAME_SIZE) == 0)
				return ndev;
		}

		i++;
	}

	return NULL;
}

/**
  * @brief  allocate netdev handle for interface
  * @param  sizeof_priv - size of priv interface
  *         name - interface name
  * @retval allocated netdev
  */
netdev_handle_t netdev_alloc(uint32_t sizeof_priv, char *name)
{
	struct netdev *ndev = NULL;

	if (!name)
		return NULL;

	ndev = (struct netdev *) malloc(sizeof(struct netdev));

	if (ndev) {
		memset(ndev, 0, sizeof(struct netdev));
		memcpy(ndev->name, name, MAX_IF_NAME_SIZE);

		ndev->priv = malloc(sizeof_priv);

		if (!ndev->priv) {
			printf("Failed to allocate memory for priv\n");
			free(ndev);
			ndev = NULL;
			return NULL;
		}
	} else {
		printf("Failed to allocate memory for net dev\n");
	}

	return ndev;
}


/**
  * @brief  free netdev's private handle
  * @param  dev - netdev handle
  * @retval None
  */
void netdev_free(netdev_handle_t dev)
{
	struct netdev *ndev = (struct netdev *) dev;

	if (ndev) {
		if (ndev->priv) {
			free(ndev->priv);
			ndev->priv = NULL;
		}

		if (ndev->net_handle) {
			free(ndev->net_handle);
			ndev->net_handle = NULL;
		}


		free(ndev);
		ndev = NULL;
	}
}


/**
  * @brief  get netdev's private interface
  * @param  dev - private interface
  * @retval private interface handle on success else NULL
  */
void * netdev_get_priv(netdev_handle_t dev)
{
	struct netdev *ndev = (struct netdev *) dev;

	if (ndev) {
		return ndev->priv;
	}

	return NULL;
}


/**
  * @brief  register netdev
  * @param  dev - private interface
  *         ops - network options to register
  * @retval 0 on success, else -1
  */
int netdev_register(netdev_handle_t dev, struct netdev_ops *ops)
{
	struct netdev *ndev = (struct netdev *) dev;

	if (!ndev || !ops) {
		printf ("Invalid arguments\n");
		return -1;
	}

	ndev->net_ops = ops;
	ndev_db[ndev_index % MAX_INTERFACE] = ndev;

	ndev_index++;

	return 0;
}


/**
  * @brief  unregister netdev
  * @param  dev - private interface
  * @retval 0 on success, else -1
  */
int netdev_unregister(netdev_handle_t dev)
{
	struct netdev *ndev = (struct netdev *) dev;

	if (!ndev) {
		printf ("Invalid arguments\n");
		return -1;
	}

	ndev->net_ops = NULL;
	ndev->state = NETDEV_STATE_DOWN;

	return 0;
}

/**
  * @brief  receive on netdev
  * @param  dev - private interface
  *         net_buf - buffer received
  * @retval 0 on success, else -1
  */
int netdev_rx(netdev_handle_t dev, struct net_pbuf *net_buf)
{
	struct netdev *ndev = (struct netdev *) dev;
	struct network_handle *net_handle;

	if (!ndev || !net_buf) {
		printf ("Invalid arguments\n");
        k_msleep(50);
		return -1;
	}

	if (ndev->state == NETDEV_STATE_UP) {
		swifthal_os_mq_send(ndev->rx_q, net_buf, -1); //TODO: check return value

		net_handle = (struct network_handle *) ndev->net_handle;

		if (net_handle->net_rx_callback)
			net_handle->net_rx_callback(net_handle, net_handle->arg);

		free(net_buf);
		net_buf = NULL;

	} else {
		goto done;
	}

	return 0;

done:
	if (net_buf) {
		if (net_buf->payload) {
			free(net_buf->payload);
			net_buf->payload = NULL;
		}
		free(net_buf);
		net_buf = NULL;
	}
    k_msleep(50);
	return -1;
}
