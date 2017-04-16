/*
 * mq_receiver.c
 *
 *  Created on: Apr 9, 2017
 *      Author: sam
 */

#include "mq_receiver.h"
#include <freertos/semphr.h>
#include "esp_log.h"

#define MQ_LOG "mq_receiver"

/* Context for storing transmit data */
struct mq_receiver_s {
  rmt_channel_t channel;
  RingbufHandle_t rx_buffer; /* Ring buffer where RX data is stored */
};

struct mq_receiver_s mq_receiver_ctx[8];

mq_receiver_t mq_receiver_init(rmt_channel_t channel, gpio_num_t gpio_num, int queue_len) {
  ESP_LOGI(MQ_LOG, "Initializing mq_receiver context with channel %d and gpio_num %d", channel, gpio_num);
  rmt_config_t rmt;
  rmt.channel = channel;
  rmt.gpio_num = gpio_num;
  rmt.mem_block_num = 1; /* Number of memory blocks. Not memory block number. */
  rmt.clk_div = 100;
  rmt.rmt_mode = RMT_MODE_RX;
  rmt.rx_config.filter_en = true;
  rmt.rx_config.filter_ticks_thresh = 50;
  rmt.rx_config.idle_threshold = 2000;
  ESP_ERROR_CHECK(rmt_config(&rmt));

  ESP_ERROR_CHECK(rmt_driver_install(channel, 2048, 0));

  mq_receiver_t ctx = mq_receiver_ctx + channel;
  ctx->channel = channel;
  ESP_ERROR_CHECK(rmt_get_ringbuf_handler(channel, &(ctx->rx_buffer)));

  return ctx;
}

static bool mq_receiver_decode(rmt_item32_t *items, int num_items, mq_event_t* packet) {
  if(!items) return false;
  if(num_items != 56) return false;

  uint64_t raw = 0;

  for(int i=0; i < 55; ++i) {
    rmt_item32_t item = items[i];
    ESP_LOGV(MQ_LOG, "RX[%d]: %d, %d, %d, %d", i, item.duration0, item.level0, item.duration1, item.level1);
    int ratio = item.duration1 / item.duration0; // ratio will be 0-1 for 1 bit and 2-3 for 0 bit
    if(ratio > 1) {
      raw <<= 1;
    } else {
      raw = (raw << 1) | 1;
    }
  }

  ESP_LOGD(MQ_LOG, "RX: %llx", raw);
  packet->magnitude = (uint16_t)(raw & 0xffff);
  packet->wand_id = (uint32_t)(raw >> 16);
  return true;
}

bool mq_receiver_receive(mq_receiver_t ctx, mq_event_t *event, int timeout_ms) {
  TickType_t timeout = timeout_ms <= 0 ? portMAX_DELAY : portTICK_PERIOD_MS * timeout_ms;
  rmt_rx_start(ctx->channel, true);

  size_t items_size = 0;
  void* items = xRingbufferReceive(ctx->rx_buffer, &items_size, timeout);
  ESP_LOGD(MQ_LOG, "Received %d bytes", items_size);

  int num_items = items_size / sizeof(rmt_item32_t);
  bool success = mq_receiver_decode(items, num_items, event);

  if(items) vRingbufferReturnItem(ctx->rx_buffer, items);

  rmt_rx_stop(ctx->channel);
  return success;
}

void mq_receiver_uninit(mq_receiver_t *ctx) {
  if(ctx == NULL) return;
  rmt_rx_stop((*ctx)->channel);
  rmt_driver_uninstall((*ctx)->channel);
  ctx = NULL;
}
