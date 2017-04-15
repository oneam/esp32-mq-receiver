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
  volatile rmt_item32_t* rx_buffer; /* The RMT buffer for the channel */
  rmt_isr_handle_t isr_handle; /* The RMT ISR for RX */
  QueueHandle_t rx_queue; /* Queue where MQ events are sent */
};

struct mq_receiver_s mq_receiver_ctx[8];

void mq_receiver_isr_handler(void* arg);

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
  rmt_config(&rmt);

  mq_receiver_t ctx = mq_receiver_ctx + channel;
  ctx->channel = channel;
  ctx->rx_buffer = RMTMEM.chan[channel].data32;
  ctx->rx_queue = xQueueCreate(queue_len, sizeof(mq_event_t));

  rmt_isr_register(mq_receiver_isr_handler, ctx, 0, &ctx->isr_handle);
  rmt_set_rx_intr_en(ctx->channel, true);
  rmt_set_err_intr_en(ctx->channel, true);
  rmt_rx_start(ctx->channel, true);

  return ctx;
}

static int mq_receiver_length(mq_receiver_t ctx) {
  for(int i=0; i < 64; ++i) {
    rmt_item32_t item = ctx->rx_buffer[i];
    //ESP_LOGD(MQ_LOG, "RX[%d]: %d, %d, %d, %d", i, item.duration0, item.level0, item.duration1, item.level1);
    if(item.duration0 == 0) return i*2;
    if(item.duration1 == 0) return i*2+1;
  }

  return 128;
}

static bool mq_receiver_decode(mq_receiver_t ctx, mq_event_t* packet) {
  int length = mq_receiver_length(ctx);
  if(length != 111) return false;

  uint64_t raw = 0;

  for(int i=0; i < 55; ++i) {
    rmt_item32_t item = ctx->rx_buffer[i];
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

void mq_receiver_isr_handler(void* arg) {
  mq_receiver_t ctx = (mq_receiver_t)arg;
  rmt_rx_stop(ctx->channel);

  if(RMT.int_st.ch1_err) {
    ESP_LOGE(MQ_LOG, "RX Error");
    RMT.int_clr.ch1_err = 1;
  }

  if(RMT.int_st.ch1_rx_end) {
    mq_event_t packet;
    bool packet_decoded = mq_receiver_decode(ctx, &packet);

    if(packet_decoded) {
      BaseType_t xHigherPriorityTaskWoken;
      xQueueSendFromISR(ctx->rx_queue, &packet, &xHigherPriorityTaskWoken);
      if(xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR();
    }

    RMT.int_clr.ch1_rx_end = 1;
  }

  rmt_rx_start(ctx->channel, true);
}

bool mq_receiver_receive(mq_receiver_t ctx, mq_event_t *event, int timeout_ms) {
  TickType_t timeout = timeout_ms <= 0 ? portMAX_DELAY : portTICK_PERIOD_MS * timeout_ms;
  return xQueueReceive(ctx->rx_queue, event, timeout);
}

void mq_receiver_uninit(mq_receiver_t *ctx) {
  if(ctx == NULL) return;
  rmt_set_rx_intr_en((*ctx)->channel, false);
  rmt_isr_deregister((*ctx)->isr_handle);
  vQueueDelete((*ctx)->rx_queue);
  ctx = NULL;
}
