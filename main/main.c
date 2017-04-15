/*
 *  Copyright 2017 Sam Leitch
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#include "driver/gpio.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "mq_receiver.h"
#include "nvs_flash.h"

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

static void mq_receive_loop(void* param);

void app_main(void)
{
    nvs_flash_init();
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    wifi_config_t sta_config = {
        .sta = {
            .ssid = "ssid",
            .password = "password",
            .bssid_set = false
        }
    };
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_connect() );

    xTaskCreate(mq_receive_loop, "mq_receive_loop", 2048, NULL, 5, NULL);
}

static void mq_receive_loop(void* param) {
  mq_receiver_t ctx = mq_receiver_init(RMT_CHANNEL_1, GPIO_NUM_27, 5);
  gpio_config_t config = {
      GPIO_SEL_25 | GPIO_SEL_26,
      GPIO_MODE_OUTPUT,
      GPIO_PULLUP_ENABLE,
      GPIO_PULLDOWN_DISABLE,
      GPIO_INTR_DISABLE
  };
  gpio_config(&config);

  gpio_set_level(GPIO_NUM_25, 1);
  gpio_set_level(GPIO_NUM_26, 1);

  mq_event_t event;
  while (true) {
    bool success = mq_receiver_receive(ctx, &event, 1000);
    if (success) {
      ESP_LOGI(__FUNCTION__, "Received MQ event for %x", event.wand_id);
      if(event.wand_id == 0x1337a981U) gpio_set_level(GPIO_NUM_25, 1);
      if(event.wand_id == 0x281ecd81U) gpio_set_level(GPIO_NUM_26, 1);
    } else {
      gpio_set_level(GPIO_NUM_25, 0);
      gpio_set_level(GPIO_NUM_26, 0);
    }
  }
}
