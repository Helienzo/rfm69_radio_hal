/**
 * @file:       hal_radio.h
 * @author:     Lucas Wennerholm <lucas.wennerholm@gmail.com>
 * @brief:      Header file for radio HAL layer
 *
 * @license: Apache 2.0
 *
 * Copyright 2025 Lucas Wennerholm
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef HAL_RADIO_H
#define HAL_RADIO_H
#include "pico/stdlib.h"

#include "pico/stdlib.h"
#include "pico/mutex.h"
#include "hardware/spi.h"
#include "rp2x_rfm69.h"
#include "hal_gpio.h"
#include "c_buffer.h"

#ifndef CONTAINER_OF
#define CONTAINER_OF(ptr, type, member)	(type *)((char *)(ptr) - offsetof(type,member))
#endif

#ifndef HAL_RADIO_SPI_INST
#define HAL_RADIO_SPI_INST spi0
#endif /* HAL_RADIO_SPI_INST */

#ifndef HAL_RADIO_SPI_BAUD_RATE
#define HAL_RADIO_SPI_BAUD_RATE 10000000
#endif /* HAL_RADIO_SPI_BAUD_RATE */

#ifndef HAL_RADIO_PIN_MISO
#define HAL_RADIO_PIN_MISO (16)
#endif /* HAL_RADIO_PIN_MISO */

#ifndef HAL_RADIO_PIN_CS
#define HAL_RADIO_PIN_CS   (17)
#endif /* HAL_RADIO_PIN_CS */

#ifndef HAL_RADIO_PIN_SCK
#define HAL_RADIO_PIN_SCK  (18)
#endif /* HAL_RADIO_PIN_SCK */

#ifndef HAL_RADIO_PIN_MOSI
#define HAL_RADIO_PIN_MOSI (19)
#endif /* HAL_RADIO_PIN_MOSI */

#ifndef HAL_RADIO_PIN_RST
#define HAL_RADIO_PIN_RST (20)
#endif /* HAL_RADIO_PIN_RST */

#ifndef HAL_RADIO_PIN_DIO0
#define HAL_RADIO_PIN_DIO0 (21)
#endif /* HAL_RADIO_PIN_DIO0 */

#ifndef HAL_RADIO_PIN_DIO1
#define HAL_RADIO_PIN_DIO1 (15)
#endif /* HAL_RADIO_PIN_DIO1 */

#ifndef HAL_RADIO_FIFO_THRESHOLD
#define HAL_RADIO_FIFO_THRESHOLD (32) // Default is one less than 50% of fifo
#endif /* HAL_RADIO_FIFO_THRESHOLD */

#ifndef HAL_RADIO_MAX_BUFFER_SIZE
#define HAL_RADIO_MAX_BUFFER_SIZE (128) // The RFM69 radio actually supports 255
#endif /* HAL_RADIO_MAX_BUFFER_SIZE */

#define HAL_RADIO_PACKET_SIZE_SIZE 1
#define HAL_RADIO_PACKET_ADDR_SIZE 1
#define HAL_RADIO_PACKET_OVERHEAD  (HAL_RADIO_PACKET_SIZE_SIZE + HAL_RADIO_PACKET_ADDR_SIZE)
#define HAL_RADIO_MAX_PACKET_SIZE  (HAL_RADIO_MAX_BUFFER_SIZE - HAL_RADIO_PACKET_OVERHEAD)

typedef enum {
    HAL_RADIO_INTERRUPT_IN_QUEUE = 1,
    HAL_RADIO_SUCCESS            = 0,
    HAL_RADIO_NULL_ERROR         = -10001,
    HAL_RADIO_GEN_ERROR          = -10002,
    HAL_RADIO_DRIVER_ERROR       = -10003,
    HAL_RADIO_GPIO_ERROR         = -10004,
    HAL_RADIO_INVALID_SIZE       = -10005,
    HAL_RADIO_SEND_FAIL          = -10006,
    HAL_RADIO_RECEIVE_FAIL       = -10007,
    HAL_RADIO_BUSY               = -10008,
    HAL_RADIO_BUFFER_ERROR       = -10009,
    HAL_RADIO_INVALID_RATE       = -10010,
    HAL_RADIO_CONFIG_ERROR       = -10011,
    HAL_RADIO_SEND_INTERRUPTED   = -10012,
} halRadioErr_t;

typedef enum {
    HAL_RADIO_BITRATE_12_5 = 1,
    HAL_RADIO_BITRATE_25,
    HAL_RADIO_BITRATE_50,
    HAL_RADIO_BITRATE_100,
    HAL_RADIO_BITRATE_150,
    HAL_RADIO_BITRATE_200,
    HAL_RADIO_BITRATE_250,
    HAL_RADIO_BITRATE_300,
} halRadioBitrate_t;

// These valus are the expected return values from a callback, any other return value
// will be returned as an error
typedef enum {
    HAL_RADIO_CB_SUCCESS, // The radio will be switched to default mode
    HAL_RADIO_CB_DO_NOTHING, // Inform hal that higher layer is in control
} halRadioCbRetval_t;

typedef enum {
    HAL_RADIO_IDLE,
    HAL_RADIO_TX, // Radio is activly transmitting
    HAL_RADIO_TX_IDLE, // Radio is in TX but not transmitting
    HAL_RADIO_TX_QUEUE, // FiFo prepared for TX
    HAL_RADIO_RX,
} halRadioMode_t;

// Packet meta data
typedef struct {
    uint8_t  address;
    uint64_t time;
} halRadioPackage_t;

// Forward declaration of the interface
typedef struct halRadioInterface halRadioInterface_t;

/**
 * This callback gets called when a new packet has been received.
 * Input: Pointer to the upstream interface
 * Input: Pointer to the new packet info
 * Returns: halRadioCbRetval_t, any other value will be considered an error and halt execution
 */
typedef int32_t (*halRadioPackageCb_t)(halRadioInterface_t *interface, halRadioPackage_t *package);

/**
 * This callback gets called when an outgoing packet is finnished transmitting
 * Input: Pointer to the upstream interface
 * Input: Pointer to the sent packet info
 * Returns: halRadioCbRetval_t, any other value will be considered an error and halt execution
 */
typedef int32_t (*halRadioPackageSentCb_t)(halRadioInterface_t *interface, halRadioPackage_t *package, halRadioErr_t result);

// Definition of interface
struct halRadioInterface {
    halRadioPackageCb_t     package_cb;
    halRadioPackageSentCb_t pkg_sent_cb;
    cBuffer_t               *pkt_buffer;
};

// Configuration struct for the halRadio
typedef struct {
    uint8_t  rx_address;
    uint8_t  broadcast_address;
    uint8_t  bitrate;
    uint16_t channel;
    int8_t   power_dbm;
} halRadioConfig_t;

typedef struct {
    // Thread/ISR safety management
    mutex_t mutex;

    // Radio configuration
    halRadioConfig_t config;
    uint8_t          current_packet_size;
    uint8_t          radio_state;

    // Gpio interfaces
    halGpioInterface_t gpio_dio0;
    halGpioInterface_t gpio_dio1;
    volatile uint8_t   gpio_interrupt;

    // Callbacks and radio mode
    // Mode is used to keep track of sending and receiving in NB mode
    halRadioMode_t       mode;
    halRadioInterface_t *package_callback;
    halRadioPackage_t    active_package;

    // Hardware driver instance
    rfm69_context_t rfm;
} halRadio_t;

/**
 * This manages the interrupts and processes the radio
 * Returns: halRadioErr_t
*/
int32_t halRadioProcess(halRadio_t *inst);

/**
 * Check if there is an event in the queue
 * Returns: halRadioErr_t
*/
int32_t halRadioEventInQueue(halRadio_t *inst);

/**
 * Initialize a radio instance
 * Input: Pointer to instance
 * Input: Hal radio config
 * Returns: halRadioErr_t
 */
int32_t halRadioInit(halRadio_t *inst, halRadioConfig_t hal_config);

/**
 * Send package and wait for completion
 * Input: Pointer to instance
 * Input: Pointer buffer structure
 * Input: Destination address
 * Returns: halRadioErr_t
 */
int32_t halRadioSendPackageBlocking(halRadio_t *inst, cBuffer_t *pkt_buffer, uint8_t address);

/**
 * Send package non blocking
 * Input: Pointer to instance
 * Input: Pointer to CB interface
 * Input: Address
 * Returns: halRadioErr_t
 */
int32_t halRadioSendPackageNB(halRadio_t *inst, halRadioInterface_t *interface, uint8_t address);

/**
 * Send what has been queued in the radio
 * Input: Pointer to instance
 * Returns: halRadioErr_t
 */
int32_t halRadioQueueSend(halRadio_t *inst, bool wait_for_tx_mode);

/**
 * Queue a packet in the radio, and prepare for sending
 * Input: Pointer to instance
 * Input: Pointer to CB interface
 * Input: Address
 * Returns: halRadioErr_t
 */
int32_t halRadioQueuePackage(halRadio_t *inst, halRadioInterface_t *interface, uint8_t address);

/**
 * Receive package, try to receive a package during specific time
 * Input: Pointer to instance
 * Input: Pointer to c_buffer to populate with data
 * Input: timeout time
 * Returns: halRadioErr_t
 */
int32_t halRadioReceivePackageBlocking(halRadio_t *inst, cBuffer_t *rx_buffer, uint32_t time_ms);

/**
 * Receive package blocking call to interface callbacks
 * Input: Pointer to instance
 * Input: Pointer to insterface
 * Input: timeout time
 * Returns: halRadioErr_t
 */
int32_t halRadioReceivePackageBlockingInterface(halRadio_t *inst, halRadioInterface_t *interface, uint32_t time_ms);

/**
 * Enable package receive in non blocking mode
 * Input: Pointer to instance
 * Input: Pointer to CB interface
 * Returns: halRadioErr_t
 */
int32_t halRadioReceivePackageNB(halRadio_t *inst, halRadioInterface_t *interface, bool wait_for_rx_mode);

/**
 * Cancel packet Receive
 * Input: Pointer to instance
 * Returns: halRadioErr_t
 */
int32_t halRadioCancelReceive(halRadio_t *inst);

/**
 * Cancel packet Transmit
 * Input: Pointer to instance
 * Returns: halRadioErr_t
 */
int32_t halRadioCancelTransmit(halRadio_t *inst);

/**
 * Enter transmit mode
 * Input: Pointer to instance
 * Returns: halRadioErr_t
 */
int32_t halRadioEnterTX(halRadio_t *inst);

/**
 * Get the current hal radio mode
 * Returns: halRadioErr_t or mode
 */
int32_t halRadioGetMode(halRadio_t *inst);

/**
 * Convert a bitrate to estimated delay for sending a specific number of bytes
 * Input: Pointer to radio instance
 * Input: Bitrate enum
 * Input: Num bytes to send
 * Returns: halRadioErr_t or time
 */
int32_t halRadioBitRateToDelayUs(halRadio_t *inst, halRadioBitrate_t bitrate, uint8_t num_bytes);

/**
 * Calculate how many us it takes to transfer num_bytes over the spi interface to the radio.
 * Input: Pointer to radio instance
 * Input: Number of bytes to send
 * Returns: halRadioErr_t or time
 */
int32_t halRadioSpiDelayEstimateUs(halRadio_t *inst, uint8_t num_bytes);

/**
 * Check if the radio is busy. This can be used for ISR and thread safety checks.
 * When using the Non Blocking functions the halRadio does run in the background in ISR
 * or main context through the process function. Checking with this function before
 * access in ISR or multiple threadcontext adds a layer of protection.
 *
 * Input: Pointer to radio instance
 * Returns: halRadioErr_t
 */
int32_t halRadioCheckBusy(halRadio_t *inst);

#endif /* HAL_RADIO_H */
