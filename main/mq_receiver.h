/*
 * mq_receiver.h
 *
 *  Created on: Apr 9, 2017
 *      Author: Sam Leitch
 */

#ifndef MAIN_MQ_RECEIVER_H_
#define MAIN_MQ_RECEIVER_H_

#include <driver/gpio.h>
#include <driver/rmt.h>

typedef struct mq_packet_s {
  uint32_t wand_id;
  uint16_t magnitude;
} mq_event_t;

/** Context used to MQ receiver channel */
typedef struct mq_receiver_s* mq_receiver_t;

mq_receiver_t mq_receiver_init(rmt_channel_t channel, gpio_num_t gpio_num, int queue_len);

bool mq_receiver_receive(mq_receiver_t ctx, mq_event_t *event, int timeout_ms);

void mq_receiver_uninit(mq_receiver_t *ctx);

#endif /* MAIN_MQ_RECEIVER_H_ */
