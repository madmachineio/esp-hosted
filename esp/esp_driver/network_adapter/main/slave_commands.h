// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <esp_err.h>
#define min(X, Y) (((X) < (Y)) ? (X) : (Y))

#define SSID_LENGTH             32
#define PASSWORD_LENGTH         64
#define BSSID_LENGTH            19
#define TIMEOUT_IN_SEC          (1000 / portTICK_RATE_MS)
#define MAC_LEN                 6

typedef struct {
    uint8_t ssid[SSID_LENGTH];
    uint8_t pwd[PASSWORD_LENGTH];
    uint8_t bssid[BSSID_LENGTH];
    uint8_t chnl;
    uint8_t max_conn;
    int8_t rssi;
    bool ssid_hidden;
    wifi_auth_mode_t ecn;
    uint8_t bw;
    uint16_t count;
} credentials_t;

esp_err_t data_transfer_handler(uint32_t session_id,const uint8_t *inbuf, ssize_t inlen,uint8_t **outbuf, ssize_t *outlen, void *priv_data);
void mad_swift_hw_init(void);

