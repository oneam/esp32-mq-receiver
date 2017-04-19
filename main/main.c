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
#include "ws2812rmt.h"

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

#define GREEN_LED GPIO_NUM_25
#define BLUE_LED GPIO_NUM_26

struct led_params {
  gpio_num_t gpio;
  xSemaphoreHandle trigger;
};

void led_loop(void *params) {
  struct led_params *p = (struct led_params*)params;
  gpio_set_direction(p->gpio, GPIO_MODE_OUTPUT);
  gpio_set_pull_mode(p->gpio, GPIO_PULLUP_ONLY);
  gpio_set_intr_type(p->gpio, GPIO_INTR_DISABLE);

  while(true) {
    gpio_set_level(p->gpio, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(p->gpio, 0);
    xSemaphoreTake(p->trigger, portMAX_DELAY);
  }
}

#define NUM_LEDS 24
#define LED_RING_GPIO GPIO_NUM_14
#define LED_RING_RMT RMT_CHANNEL_0

struct led_ring_params {
  rgb_t colors[NUM_LEDS];
};

rgb_t black = {  0,  0,  0 };
rgb_t blue  = {  0,  0, 16 };
rgb_t green = {  0, 16,  0 };
rgb_t red   = { 16,  0,  0 };

void led_ring_loop(void* params) {
  struct led_ring_params *p = (struct led_ring_params*)params;

  ws2812rmt_t led_ring = ws2812rmt_init(LED_RING_RMT, LED_RING_GPIO, NUM_LEDS);

  for(int i=0; i<NUM_LEDS; ++i) {
    p->colors[i] = black;
  }
  p->colors[0] = red;

  while(true) {
    ws2812rmt_set_colors(led_ring, p->colors, NUM_LEDS, false);

    vTaskDelay(pdMS_TO_TICKS(50));

    for(int i=NUM_LEDS-1; i>0; --i) {
      p->colors[i] = p->colors[i-1];
    }

    if(rgb_equal(p->colors[0], p->colors[NUM_LEDS - 1])) {
      p->colors[0] = black;
    }
  }
}

#define MQ_GPIO GPIO_NUM_27
#define MQ_RMT RMT_CHANNEL_1

struct mq_params {
  struct led_params green_led;
  struct led_params blue_led;
  struct led_ring_params led_ring;
};

static void mq_receive_loop(void* params) {
  struct mq_params *p = (struct mq_params*)params;

  mq_receiver_t ctx = mq_receiver_init(MQ_RMT, MQ_GPIO);
  mq_event_t event;

  while (true) {
    mq_result_t result = mq_receiver_receive(ctx, &event, portMAX_DELAY);
    if (result != MQ_SUCCESS) continue;

    ESP_LOGI(__FUNCTION__, "Received MQ event for %x", event.wand_id);
    if(event.wand_id == 0x1337a981U) {
      p->led_ring.colors[0] = green;
      xSemaphoreGive(p->green_led.trigger);
    }

    if(event.wand_id == 0x281ecd81U) {
      p->led_ring.colors[0] = blue;
      xSemaphoreGive(p->blue_led.trigger);
    }
  }
}

struct mq_params app_params;

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

    app_params.green_led.gpio = GREEN_LED;
    app_params.green_led.trigger = xSemaphoreCreateBinary();

    app_params.blue_led.gpio = BLUE_LED;
    app_params.blue_led.trigger = xSemaphoreCreateBinary();

    xTaskCreate(led_ring_loop, "led_ring_loop", 2048, &(app_params.led_ring), 5, NULL);
    xTaskCreate(led_loop, "green_led_loop", 2048, &(app_params.green_led), 5, NULL);
    xTaskCreate(led_loop, "blue_led_loop", 2048, &(app_params.blue_led), 5, NULL);
    xTaskCreate(mq_receive_loop, "mq_receive_loop", 2048, &app_params, 5, NULL);
}
