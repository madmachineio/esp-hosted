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


/** prevent recursive inclusion **/
#ifndef __SPI_DRV_H
#define __SPI_DRV_H

#ifdef __cplusplus
extern "C" {
#endif

/** Includes **/
#include "common.h"
#include "swift_os.h"

/** constants/macros **/
#define STA_INTERFACE_ID		0
#define SOFTAP_INTERFACE_ID		1
#define MAX_NETWORK_INTERFACES  2

#define STA_INTERFACE           "ESP_STATION"
#define SOFTAP_INTERFACE        "ESP_SOFTAP"


typedef enum spi_drv_events_s {
	SPI_DRIVER_ACTIVE
} spi_drv_events_e;

/** Exported Structures **/

/** Exported variables **/

/** Inline functions **/

/** Exported Functions **/
/**
  * @brief  host access esp device interface initialize
  * @param  spi_drv_evt_handler - event handler of type spi_drv_events_e
  * @retval None
  */

void esp_device_if_init(void *spi, void *spi_cs_gpio,
									void *hand_gpio, void *ready_gpio,
									void(*if_evt_handler)(uint8_t));

int esp_device_if_transaction(uint8_t iface_type, uint8_t iface_num,
								uint8_t * wbuffer, uint16_t wlen);

void esp_device_if_reset(void *reset_gpio);

struct esp_private {
	uint8_t     if_type;
	uint8_t     if_num;
	void        *netdev;
};

#ifdef __cplusplus
}
#endif

#endif
