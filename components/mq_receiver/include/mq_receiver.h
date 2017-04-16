/*
 *  Receives signals from the RMT driver and parses MQ wand ids
 *
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

#ifndef MAIN_MQ_RECEIVER_H_
#define MAIN_MQ_RECEIVER_H_

#include <driver/gpio.h>
#include <driver/rmt.h>


/** Result code for mq_receiver_receive */
typedef enum mq_result_e {
  MQ_SUCCESS, /** Success */
  MQ_NOT_INITIALIZED, /** Receiver not initialized */
  MQ_TIMEOUT, /** Timed out without receiving data */
  MQ_DECODE_FAILED /** Received invalid data */
} mq_result_t;


/** Event fired when a wand wave is received and decoded */
typedef struct mq_event_s {
  uint32_t wand_id; /** Unique ID given to each wand */
  uint16_t magnitude; /** Increases when the wand is waved harder */
} mq_event_t;


/** Context used to MQ receiver channel */
typedef struct mq_receiver_s* mq_receiver_t;


/** Initializes an MQ receiver */
mq_receiver_t mq_receiver_init(rmt_channel_t channel, gpio_num_t gpio_num, int queue_len);


/** Starts the RMT driver receiving. */
mq_result_t mq_receiver_receive(mq_receiver_t ctx, mq_event_t *event, int timeout_ms);


/** releases resources used by the MQ receiver */
void mq_receiver_uninit(mq_receiver_t *ctx);

#endif /* MAIN_MQ_RECEIVER_H_ */
