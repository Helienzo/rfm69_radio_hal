/**
 * @file:       hal_radio.c
 * @author:     Lucas Wennerholm <lucas.wennerholm@gmail.com>
 * @brief:      Implementation of radio HAL layer
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

#include "hal_radio.h"

#ifndef LOG
#define LOG(f_, ...) printf((f_), ##__VA_ARGS__)
#endif

#ifndef LOG_DEBUG_BUSY
#define LOG_DEBUG_BUSY(f_, ...) printf((f_), ##__VA_ARGS__)
#endif

#ifndef LOG_DEBUG
#define LOG_DEBUG(f_, ...)
#endif

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#define PACKET_RX_POLL_TIME_MS 10

typedef enum {
    HAL_RADIO_REC_IDLE,
    HAL_RADIO_REC_DATA,
    HAL_RADIO_REC_END,
    HAL_RADIO_TX_DATA,
} halRadioReceiveState_t;

// Function declarations

/**
 * The byteWiseRead function is needed due to a quirk in the RFM69 chip.
 * If the fifo needs to be read on-the-fly during an active RF transmission,
 * which is the case when the packet is larger than the FIFO size we are forced
 * to read one byte at a time over the SPI. One can speculate that this is
 * needed for the chip to have time to populate the fifo with new incomming data.
 * Perhaps it cannot do that and at the same time clock out a long stream of bytes.
 */
static int32_t byteWiseRead(halRadio_t *inst, uint8_t num_bytes);

/**
 * The byteWiseWrite function is needed due to a quirk in the RFM69 chip.
 * If the fifo needs to be written on-the-fly during an active RF transmission,
 * which is the case when the packet is larger than the FIFO size we are forced
 * to write one byte at a time over the SPI. One can speculate that this is
 * needed for the chip to have time to pull bytes from the fifo. Perhaps it cannot
 * do that and at the same time clock in a long stream of bytes over SPI.
 */
static int32_t byteWiseWrite(halRadio_t *inst, cBuffer_t *tx_buf, uint8_t num_bytes);

// Constant for calculating time used by the spi to send X bytes
static const float kHalRadioSpiUsPerByte = (8.0f * 1000000.0f)/HAL_RADIO_SPI_BAUD_RATE;

static int32_t managePacketSent(halRadio_t *inst) {
    bool taken = mutex_try_enter(&inst->mutex, NULL);
    if (!taken) {
        LOG_DEBUG_BUSY("BUSY ERROR %i\n", 0);
        return HAL_RADIO_BUSY;
    }

    // Read the packet sent flag
    bool state = false;
    int32_t cb_res = HAL_RADIO_CB_SUCCESS;
    // TODO is this check enough, perhaps there is a risk that we end up here twice on the same packet?
    if (!rfm69_irq2_flag_state(&inst->rfm, RFM69_IRQ2_FLAG_PACKET_SENT, &state)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    // If the package was not sent this is an invalid interrupt, bad ..
    // TODO we must check if this is a bad interrupt, a duplicate message sent or something else.
    if (!state) {
        // Try to return to default mode
        if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_STDBY)) {
            mutex_exit(&inst->mutex);
            return HAL_RADIO_DRIVER_ERROR;
        }

        // Reset the GPIO callback
        if ((halGpioDisableIrqCb(HAL_RADIO_PIN_DIO0)) != HAL_GPIO_SUCCESS) {
            mutex_exit(&inst->mutex);
            return HAL_RADIO_GPIO_ERROR;
        }

        // Reset the GPIO callback
        if ((halGpioDisableIrqCb(HAL_RADIO_PIN_DIO1)) != HAL_GPIO_SUCCESS) {
            mutex_exit(&inst->mutex);
            return HAL_RADIO_GPIO_ERROR;
        }

        mutex_exit(&inst->mutex);
        // Reset the radio mode
        inst->mode = HAL_RADIO_IDLE;

        // Notify that the package send failed
        // TODO it is not really a send fail, this is a weird interrupt, might not be related to any packet
        if (inst->package_callback != NULL && inst->package_callback->pkg_sent_cb != NULL) {
            cb_res = inst->package_callback->pkg_sent_cb(inst->package_callback, &inst->active_package, HAL_RADIO_SEND_FAIL);
        }
        inst->active_package.address = 0;
    } else {
        inst->mode = HAL_RADIO_TX_IDLE;

        mutex_exit(&inst->mutex);
        // Notify that the package was successfully sent
        if (inst->package_callback != NULL && inst->package_callback->pkg_sent_cb != NULL) {
            cb_res = inst->package_callback->pkg_sent_cb(inst->package_callback, &inst->active_package, HAL_RADIO_SUCCESS);
        }
        inst->active_package.address = 0;
    }

    taken = mutex_try_enter(&inst->mutex, NULL);
    if (!taken) {
        LOG_DEBUG_BUSY("BUSY ERROR %i\n", 1);
        return HAL_RADIO_BUSY;
    }

    // Manage result from callback
    switch(cb_res) {
        case HAL_RADIO_CB_SUCCESS:
            // Try to return radio to default mode
            if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_STDBY)) {
                mutex_exit(&inst->mutex);
                return HAL_RADIO_DRIVER_ERROR;
            }

            // Reset the GPIO callback
            if ((halGpioDisableIrqCb(HAL_RADIO_PIN_DIO0)) != HAL_GPIO_SUCCESS) {
                mutex_exit(&inst->mutex);
                return HAL_RADIO_GPIO_ERROR;
            }

            // Reset the GPIO callback
            if ((halGpioDisableIrqCb(HAL_RADIO_PIN_DIO1)) != HAL_GPIO_SUCCESS) {
                mutex_exit(&inst->mutex);
                return HAL_RADIO_GPIO_ERROR;
            }

            inst->mode = HAL_RADIO_IDLE;
            mutex_exit(&inst->mutex);
            return HAL_RADIO_SUCCESS;
        case HAL_RADIO_CB_DO_NOTHING:
            // Do nothing, indicates that the higher layer has called a hal function
            break;
        default:
            // Try to return radio to default mode
            if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_STDBY)) {
                mutex_exit(&inst->mutex);
                return HAL_RADIO_DRIVER_ERROR;
            }

            // Reset the GPIO callback
            if ((halGpioDisableIrqCb(HAL_RADIO_PIN_DIO0)) != HAL_GPIO_SUCCESS) {
                mutex_exit(&inst->mutex);
                return HAL_RADIO_GPIO_ERROR;
            }

            // Reset the GPIO callback
            if ((halGpioDisableIrqCb(HAL_RADIO_PIN_DIO1)) != HAL_GPIO_SUCCESS) {
                mutex_exit(&inst->mutex);
                return HAL_RADIO_GPIO_ERROR;
            }

            inst->mode = HAL_RADIO_IDLE;
            mutex_exit(&inst->mutex);
            return cb_res;
    }

    mutex_exit(&inst->mutex);
    return HAL_RADIO_SUCCESS;
}

static int32_t cleanUpDuringRx(halRadio_t *inst, int32_t ret_val) {
    // Reset the radio receive state
    inst->radio_state = HAL_RADIO_REC_IDLE;

    // If the radio is busy we must exit here
    if (ret_val == HAL_RADIO_BUSY) {
        return ret_val;
    }

    bool taken = mutex_try_enter(&inst->mutex, NULL);
    if (!taken) {
        LOG_DEBUG_BUSY("BUSY ERROR %i %i\n", 33, ret_val);
        return HAL_RADIO_BUSY; // Not good
    }

    // Check if we need to empty the buffer
    bool not_empty = true;
    uint32_t counter = 0;

    while (not_empty && (counter < RFM69_FIFO_SIZE)) {
        if (!rfm69_irq2_flag_state(&inst->rfm, RFM69_IRQ2_FLAG_FIFO_NOT_EMPTY, &not_empty)) {
            mutex_exit(&inst->mutex);
            return HAL_RADIO_DRIVER_ERROR; // Fatal error
        }

        if (not_empty) {
            // Read the byte
            uint8_t buf = 0;
            if (!rfm69_read(&inst->rfm, RFM69_REG_FIFO, &buf, 1)) {
                mutex_exit(&inst->mutex);
                return HAL_RADIO_DRIVER_ERROR; // Fatal error
            }
            counter++;
        }
    }

    mutex_exit(&inst->mutex);
    return ret_val;
}

static int32_t managePayloadReady(halRadio_t *inst) {
    bool taken = mutex_try_enter(&inst->mutex, NULL);
    if (!taken) {
        LOG_DEBUG_BUSY("BUSY ERROR %i\n", 3);
        return cleanUpDuringRx(inst, HAL_RADIO_BUSY); // Fatal error
    }

    LOG_DEBUG("PAYLOAD READY INTERRUPT %u\n", inst->current_packet_size);
    bool state = false;

    // TODO we could improve speed by reading the IRQ register once and checking mulitple flags
    if (!rfm69_irq2_flag_state(&inst->rfm, RFM69_IRQ2_FLAG_PAYLOAD_READY, &state)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR; // Fatal error
    }

    // Check if any package was received, and that the interrupt was valid
    if (!state) {
        // Try to return to default mode
        if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_STDBY)) {
            mutex_exit(&inst->mutex);
            return HAL_RADIO_DRIVER_ERROR; // Fatal error
        }

        mutex_exit(&inst->mutex);
        // This is not a critical error but this packet is lost
        return cleanUpDuringRx(inst, HAL_RADIO_RECEIVE_FAIL);
    }

    // Get the CRC state
    if (!rfm69_irq2_flag_state(&inst->rfm, RFM69_IRQ2_FLAG_CRC_OK, &state)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    // Get the receive buffer
    cBuffer_t *rx_buf = inst->package_callback->pkt_buffer;

    // This will just trigger a regular Payload Ready
    if (inst->radio_state == HAL_RADIO_REC_END) {
        // Reset the radio state
        inst->radio_state = HAL_RADIO_REC_IDLE;
    } else if (inst->radio_state != HAL_RADIO_REC_IDLE) {
        // This state is invalid at this point and the packet is lost

        // Clear the buffer used for receiving data
        if (cBufferClear(rx_buf) < 0) {
            mutex_exit(&inst->mutex);
            return HAL_RADIO_BUFFER_ERROR; // Fatal error
        }

        mutex_exit(&inst->mutex);
        // This is not a critical error but this packet is lost
        return cleanUpDuringRx(inst, HAL_RADIO_RECEIVE_FAIL);
    } else {
        // Clear the buffer used for receiving data
        if (cBufferClear(rx_buf) < 0) {
            mutex_exit(&inst->mutex);
            return HAL_RADIO_BUFFER_ERROR; // Fatal error
        }

        // Try to Read payload size byte from FIFO, but only if it was not done in the fifo interrupt
        if (!rfm69_read(&inst->rfm, RFM69_REG_FIFO, &inst->current_packet_size, 1)) {
            mutex_exit(&inst->mutex);
            return HAL_RADIO_DRIVER_ERROR; // Fatal error
        }

        // Check if the packet size is valid
        if (inst->current_packet_size == 0 || inst->current_packet_size > cBufferAvailableForWrite(rx_buf)) {
            mutex_exit(&inst->mutex);
            // This is not a critical error but this packet is lost
            return cleanUpDuringRx(inst, HAL_RADIO_RECEIVE_FAIL);
        }
    }

    // Get the current write pointer
    uint8_t * raw_rx_buffer = NULL;
    if ((raw_rx_buffer = cBufferGetWritePointer(rx_buf)) == NULL){
        mutex_exit(&inst->mutex);
        return HAL_RADIO_BUFFER_ERROR; // Fatal error
    }

    // Try to Read the rest of the payload
    if (!rfm69_read(&inst->rfm, RFM69_REG_FIFO, raw_rx_buffer, inst->current_packet_size)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR; // Fatal error
    }

    if (cBufferEmptyWrite(rx_buf, inst->current_packet_size) < C_BUFFER_SUCCESS) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_BUFFER_ERROR; // Fatal error
    }

    inst->current_packet_size = 0;

    // |SIZE|ADDR|PAYLOAD|, size = sizeof(addr) + sizeof(payload)
    // Get the target address for this package, should be this radio .., this call is safe
    inst->active_package.address = cBufferReadByte(rx_buf);

    int32_t cb_res = HAL_RADIO_CB_SUCCESS;
    mutex_exit(&inst->mutex);

    if (state) {
        // Notify about new package
        if (inst->package_callback != NULL && inst->package_callback->package_cb != NULL) {
            cb_res = inst->package_callback->package_cb(inst->package_callback, &inst->active_package);
        }
    } else {
        LOG_DEBUG("CRC FAIL\n");
        if (cBufferClear(rx_buf) != C_BUFFER_SUCCESS) {
            return HAL_RADIO_BUFFER_ERROR; // Fatal error
        }
        // Stay in RX as we where intructed
        cb_res = HAL_RADIO_CB_DO_NOTHING;
    }

    taken = mutex_try_enter(&inst->mutex, NULL);
    if (!taken) {
        // This is not good, the radio will en up in an undefined state
        LOG_DEBUG_BUSY("BUSY ERROR %i\n", 4);
        return cleanUpDuringRx(inst, HAL_RADIO_BUSY);
    }

    // Manage result from callback
    switch(cb_res) {
        case HAL_RADIO_CB_SUCCESS:
            // Try to return radio to default mode
            if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_STDBY)) {
                mutex_exit(&inst->mutex);
                return HAL_RADIO_DRIVER_ERROR; // Fatal error
            }

            // Reset the GPIO callback
            if ((halGpioDisableIrqCb(HAL_RADIO_PIN_DIO0)) != HAL_GPIO_SUCCESS) {
                mutex_exit(&inst->mutex);
                return HAL_RADIO_GPIO_ERROR; // Fatal error
            }

            // Reset the GPIO callback
            if ((halGpioDisableIrqCb(HAL_RADIO_PIN_DIO1)) != HAL_GPIO_SUCCESS) {
                mutex_exit(&inst->mutex);
                return HAL_RADIO_GPIO_ERROR; // Fatal error
            }

            inst->mode = HAL_RADIO_IDLE;
            mutex_exit(&inst->mutex);
            return HAL_RADIO_SUCCESS;
        case HAL_RADIO_CB_DO_NOTHING:
            // THis indicates that the higher layer has called a hal function, or that we should just continue in RX
            // Re-Enable gpio callback for fifo level
            if (halGpioEnableIrqCbRisingEdge(&inst->gpio_dio1, HAL_RADIO_PIN_DIO1) != HAL_GPIO_SUCCESS) {
                mutex_exit(&inst->mutex);
                return HAL_RADIO_GPIO_ERROR; // Fatal error
            }
            break;
        default:
            // Try to return radio to default mode
            if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_STDBY)) {
                mutex_exit(&inst->mutex);
                return HAL_RADIO_DRIVER_ERROR; // Fatal error
            }

            // Reset the GPIO callback
            if ((halGpioDisableIrqCb(HAL_RADIO_PIN_DIO0)) != HAL_GPIO_SUCCESS) {
                mutex_exit(&inst->mutex);
                return HAL_RADIO_GPIO_ERROR; // Fatal error
            }

            inst->mode = HAL_RADIO_IDLE;
            mutex_exit(&inst->mutex);
            return cb_res;
    }

    mutex_exit(&inst->mutex);
    return HAL_RADIO_SUCCESS;
}

// IRQ from gpio module
static void dio0GpioCallback(halGpioInterface_t *interface, halGpioEvents_t event) {
    halRadio_t * inst = CONTAINER_OF(interface, halRadio_t, gpio_dio0);

    // Store Time Of Arrival
    inst->active_package.time = time_us_64();

    inst->gpio_interrupt = HAL_RADIO_PIN_DIO0;
}

// IRQ from gpio module
static void dio1GpioCallback(halGpioInterface_t *interface, halGpioEvents_t event) {
    halRadio_t * inst = CONTAINER_OF(interface, halRadio_t, gpio_dio1);

    // Store Time Of Arrival
    inst->active_package.time = time_us_64();

    inst->gpio_interrupt = HAL_RADIO_PIN_DIO1;
}

static int32_t manageDio0Interrupt(halRadio_t *inst) {
    // Manage the radio mode
    switch(inst->mode) {
        case HAL_RADIO_TX:
           return managePacketSent(inst);
        case HAL_RADIO_RX:
           return managePayloadReady(inst);
        default:
            break;
    }

    // Reset the radio mode
    inst->mode = HAL_RADIO_IDLE;

    // Protect the radio during hal access
    bool taken = mutex_try_enter(&inst->mutex, NULL);
    if (!taken) {
        LOG_DEBUG_BUSY("BUSY ERROR %i\n", 5);
        return HAL_RADIO_BUSY; // Fatal error
    }

    // Try to return radio to default mode
    if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_STDBY)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR; // Fatal error
    }

    mutex_exit(&inst->mutex);

    return HAL_RADIO_SUCCESS;
}

static int32_t manageTXFifoThreshold(halRadio_t *inst) {
    LOG_DEBUG("TX FIFO INTERRUPT\n");

    switch (inst->radio_state) {
        case HAL_RADIO_TX_DATA:

            // Check if there still is more to send than fits in the fifo
            if (inst->current_packet_size > HAL_RADIO_FIFO_THRESHOLD) {
                LOG_DEBUG("Writing %u bytes\n", HAL_RADIO_FIFO_THRESHOLD);
                int32_t res = byteWiseWrite(inst, inst->package_callback->pkt_buffer, HAL_RADIO_FIFO_THRESHOLD);
                if (res != HAL_RADIO_SUCCESS) {
                    return res; // Probably a fatal error
                }

                inst->current_packet_size -= HAL_RADIO_FIFO_THRESHOLD;
            } else {
                if ((halGpioDisableIrqCb(HAL_RADIO_PIN_DIO1)) != HAL_GPIO_SUCCESS) {
                    return HAL_RADIO_GPIO_ERROR; // Fatal error
                }

                LOG_DEBUG("Writing %u bytes\n", inst->current_packet_size);
                // Write the rest of the data
                int32_t res = byteWiseWrite(inst, inst->package_callback->pkt_buffer, inst->current_packet_size);
                if (res != HAL_RADIO_SUCCESS) {
                    return res; // Probably a fatal error
                }

                // Transfer complete
                inst->current_packet_size = 0;

                // Reset the radio state
                inst->radio_state = HAL_RADIO_REC_IDLE;

                bool state = false;

                bool taken = mutex_try_enter(&inst->mutex, NULL);
                if (!taken) {
                    LOG_DEBUG_BUSY("BUSY ERROR %i\n", 6);
                    return HAL_RADIO_BUSY; // Fatal error
                }

                // Check if the payload ready flag has triggered
                if (!rfm69_irq2_flag_state(&inst->rfm, RFM69_IRQ2_FLAG_PACKET_SENT, &state)) {
                    mutex_exit(&inst->mutex);
                    return HAL_RADIO_DRIVER_ERROR; // Fatal error
                }

                mutex_exit(&inst->mutex);

                // If so is the case trigger the sent function directly from here
                if (state) {
                    // Reset the flag, as this would mean that the packedt sent interrupt has fired while we processed the TX fifo
                    inst->gpio_interrupt = 0;
                    return managePacketSent(inst);
                }
            }
        break;
        default:
            break;
    }

    return HAL_RADIO_SUCCESS;
}

static int32_t byteWiseRead(halRadio_t *inst, uint8_t num_bytes) {
    bool taken = mutex_try_enter(&inst->mutex, NULL);
    if (!taken) {
        LOG_DEBUG_BUSY("BUSY ERROR %i\n", 7);
        return HAL_RADIO_BUSY;
    }

    cBuffer_t *rx_buf = inst->package_callback->pkt_buffer;
    uint64_t s_time = time_us_64();
    uint64_t e_time = 0;
    int32_t expected_time = halRadioBitRateToDelayUs(inst, inst->config.bitrate, num_bytes) + halRadioSpiDelayEstimateUs(inst, num_bytes);
    uint8_t read_bytes = 0;
    bool    state = false;

    while (read_bytes < num_bytes && e_time < expected_time) {
        e_time = time_us_64() - s_time;

        if (!rfm69_irq2_flag_state(&inst->rfm, RFM69_IRQ2_FLAG_FIFO_NOT_EMPTY, &state)) {
            mutex_exit(&inst->mutex);
            return HAL_RADIO_DRIVER_ERROR; // Fatal error
        }

        if (state) {
            uint8_t next_byte = 0;
            // Try to Read 1 byte
            if (!rfm69_read(&inst->rfm, RFM69_REG_FIFO, &next_byte, 1)) {
                mutex_exit(&inst->mutex);
                return HAL_RADIO_DRIVER_ERROR; // Fatal error
            }

            if (cBufferAppendByte(rx_buf, next_byte) != 1) {
                // We end up here if the buffer is full
                mutex_exit(&inst->mutex);
                return HAL_RADIO_RECEIVE_FAIL; // Not a critical error but this packet is lost
            }

            read_bytes++;
            LOG_DEBUG("Bytes read %u, total %u, %c\n", read_bytes, inst->current_packet_size, next_byte);
        }
    }

    if (read_bytes != num_bytes) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_RECEIVE_FAIL; // Not a critical error but this packet is lost
    }

    mutex_exit(&inst->mutex);
    return HAL_RADIO_SUCCESS;
}

static int32_t manageRXFifoThreshold(halRadio_t *inst) {
    LOG_DEBUG("RX FIFO INTERRUPT\n");
    int32_t result = HAL_RADIO_SUCCESS;

    // Get the buffer pointer
    cBuffer_t *rx_buf = inst->package_callback->pkt_buffer;

    switch (inst->radio_state) {
        case HAL_RADIO_REC_IDLE:
            // Clear the buffer to make room for a new package
            if (cBufferClear(rx_buf) < 0) {
                return HAL_RADIO_BUFFER_ERROR; // Fatal error
            }

            bool taken = mutex_try_enter(&inst->mutex, NULL);
            if (!taken) {
                LOG_DEBUG_BUSY("BUSY ERROR %i\n", 8);
                return HAL_RADIO_BUSY; // Fatal error
            }

            // Try to Read payload size byte from FIFO
            if (!rfm69_read(&inst->rfm, RFM69_REG_FIFO, &inst->current_packet_size, 1)) {
                mutex_exit(&inst->mutex);
                return HAL_RADIO_DRIVER_ERROR; // Fatal error
            }

            mutex_exit(&inst->mutex);

            // Check if the packet size is valid
            if (inst->current_packet_size == 0 || inst->current_packet_size > cBufferAvailableForWrite(rx_buf)) {
                return cleanUpDuringRx(inst, HAL_RADIO_SUCCESS); // Not a fatal error but this packet is lost
            }

            // Check if a this is a large packet that needs to be read from the fifo in steps
            if (inst->current_packet_size > RFM69_FIFO_SIZE) {
                LOG_DEBUG("Large packet transfer 1st %u\n", inst->current_packet_size);

                // Try to Read remaining data bytes from FIFO, keep in mind that we have allready read 1 byte to get the size
                result = byteWiseRead(inst, HAL_RADIO_FIFO_THRESHOLD - 1);
                if (result == HAL_RADIO_RECEIVE_FAIL) {
                    return cleanUpDuringRx(inst, HAL_RADIO_SUCCESS); // Not a fatal error but this packet is lost
                } else if (result != HAL_RADIO_SUCCESS) {
                    LOG("Bytewise read fail 1 %i\n", result);
                    return result; // Probably a fatal error
                }

                inst->current_packet_size -= (HAL_RADIO_FIFO_THRESHOLD - 1);
                // Check if we will get another interrupt
                if (inst->current_packet_size > HAL_RADIO_FIFO_THRESHOLD) {
                    // There will be another interrupt with data
                    inst->radio_state = HAL_RADIO_REC_DATA;
                } else {
                    // The next interrupt will be a paylodReady
                    LOG_DEBUG("End state 0 remaining %u\n", inst->current_packet_size);
                    inst->radio_state = HAL_RADIO_REC_END;

                    // Reset the GPIO callback, to not trigger when reading data from fifo
                    if ((halGpioDisableIrqCb(HAL_RADIO_PIN_DIO1)) != HAL_GPIO_SUCCESS) {
                        return HAL_RADIO_GPIO_ERROR; // Fatal error
                    }
                }
            } else {
                // This will just trigger a regular Payload Ready
                LOG_DEBUG("End state 1 remaining %u\n", inst->current_packet_size);
                inst->radio_state = HAL_RADIO_REC_END;

                // Reset the GPIO callback, to not trigger when reading data from fifo
                if ((halGpioDisableIrqCb(HAL_RADIO_PIN_DIO1)) != HAL_GPIO_SUCCESS) {
                    return HAL_RADIO_GPIO_ERROR; // Fatal error
                }
            }
        break;
        case HAL_RADIO_REC_DATA:
            LOG_DEBUG("Large packet transfer rest %u\n", inst->current_packet_size);

            // Try to Read data bytes from FIFO
            result = byteWiseRead(inst, HAL_RADIO_FIFO_THRESHOLD);
            if (result == HAL_RADIO_RECEIVE_FAIL) {
                return cleanUpDuringRx(inst, HAL_RADIO_SUCCESS); // Not a fatal error but this packet is lost
            } else if (result != HAL_RADIO_SUCCESS) {
                LOG("Bytewise read fail 2 %i\n", result);
                return result; // Probably a fatal error
            }

            inst->current_packet_size -= HAL_RADIO_FIFO_THRESHOLD;

            // Check if we will get another interrupt
            if (inst->current_packet_size > HAL_RADIO_FIFO_THRESHOLD) {
                // There will be another interrupt with data
                inst->radio_state = HAL_RADIO_REC_DATA;
            } else {
                LOG_DEBUG("End state 2 remaining %u\n", inst->current_packet_size);
                // The next interrupt will be a paylodReady
                inst->radio_state = HAL_RADIO_REC_END;

                // Reset the GPIO callback, to not trigger when reading data from fifo
                if ((halGpioDisableIrqCb(HAL_RADIO_PIN_DIO1)) != HAL_GPIO_SUCCESS) {
                    return HAL_RADIO_GPIO_ERROR; // Fatal error
                }
            }
            break;
        default:
        // What should we do here?
        break;
    }

    bool taken = mutex_try_enter(&inst->mutex, NULL);
    if (!taken) {
        LOG_DEBUG_BUSY("BUSY ERROR %i\n", 9);
        return HAL_RADIO_BUSY; // Fatal error
    }

    bool state = false;
    if (!rfm69_irq2_flag_state(&inst->rfm, RFM69_IRQ2_FLAG_PAYLOAD_READY, &state)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR; // Fatal error
    }

    mutex_exit(&inst->mutex);

    if (state) {
        // Reset the flag, as this would mean that the payload ready interrupt has fired while we processed the RX fifo
        inst->gpio_interrupt = 0;
        return managePayloadReady(inst);
    }

    return HAL_RADIO_SUCCESS;
}

static int32_t manageDio1Interrupt(halRadio_t *inst) {
    // Manage the radio mode
    switch(inst->mode) {
        case HAL_RADIO_TX:
           return manageTXFifoThreshold(inst);
        case HAL_RADIO_RX:
           return manageRXFifoThreshold(inst);
        default:
            break;
    }

    // Reset the radio mode
    inst->mode = HAL_RADIO_IDLE;

    // Protect access to the radio
    bool taken = mutex_try_enter(&inst->mutex, NULL);
    if (!taken) {
        LOG_DEBUG_BUSY("BUSY ERROR %i\n", 10);
        return HAL_RADIO_BUSY; // Fatal error
    }

    // Try to return radio to default mode
    if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_STDBY)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR; // Fatal error
    }

    mutex_exit(&inst->mutex);

    return HAL_RADIO_SUCCESS;
}

int32_t halRadioEventInQueue(halRadio_t *inst) {
    if (inst->gpio_interrupt > 0) {
        return HAL_RADIO_INTERRUPT_IN_QUEUE;
    }

    return HAL_RADIO_SUCCESS;
}

int32_t halRadioProcess(halRadio_t *inst) {
    switch (inst->gpio_interrupt) {
        case 0:
            // Nothing to do
             return HAL_GPIO_SUCCESS;
        case HAL_RADIO_PIN_DIO1: {
            // Reset the interrupt flag
            inst->gpio_interrupt = 0;
            int32_t res = manageDio1Interrupt(inst);
            return res;
            } break;
        case HAL_RADIO_PIN_DIO0: {
            // Reset the interrupt flag
            inst->gpio_interrupt = 0;
            int32_t res = manageDio0Interrupt(inst);
            return res;
        } break;
        default:
            return HAL_RADIO_GPIO_ERROR;
    }
    inst->gpio_interrupt = 0;

    return HAL_GPIO_SUCCESS;
}

int32_t halRadioInit(halRadio_t *inst, halRadioConfig_t hal_config) {
    if (inst == NULL) {
        return HAL_RADIO_NULL_ERROR;
    }

    // Initialize the spi
    spi_init(HAL_RADIO_SPI_INST, HAL_RADIO_SPI_BAUD_RATE);
    gpio_set_function(HAL_RADIO_PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(HAL_RADIO_PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(HAL_RADIO_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_init(HAL_RADIO_PIN_CS);

    // Configure DIO0 pin as input with pull down
    gpio_init(HAL_RADIO_PIN_DIO0);
    gpio_set_dir(HAL_RADIO_PIN_DIO0, GPIO_IN);
    gpio_pull_down(HAL_RADIO_PIN_DIO0);

    // Configure DIO1 pin as input with no pull
    gpio_init(HAL_RADIO_PIN_DIO1);
    gpio_set_dir(HAL_RADIO_PIN_DIO1, GPIO_IN);
    gpio_disable_pulls(HAL_RADIO_PIN_DIO0);

    // Configure the dio callbacks
    inst->gpio_dio0.data_cb = dio0GpioCallback;
    inst->gpio_dio1.data_cb = dio1GpioCallback;
    inst->gpio_interrupt    = 0;

    // Drive CS pin high
    gpio_set_dir(HAL_RADIO_PIN_CS, GPIO_OUT);
    gpio_put(HAL_RADIO_PIN_CS, 1);

    mutex_init(&inst->mutex);

    bool taken = mutex_try_enter(&inst->mutex, NULL);
    if (!taken) {
        LOG_DEBUG_BUSY("BUSY ERROR %i\n", 11);
        return HAL_RADIO_BUSY;
    }

    // Configure the radio
    struct rfm69_config_s config = {
       .spi = HAL_RADIO_SPI_INST,
       .pin_cs = HAL_RADIO_PIN_CS,
       .pin_rst = HAL_RADIO_PIN_RST
    };

    if (!rfm69_init(&inst->rfm, &config)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    RFM69_MODEM_BITRATE rfm69_bitrate = 0;
    uint32_t            freq_dev      = 600;  // This is the absolute minimum
    RFM69_RXBW_MANTISSA bw_mantissa   = 0;
    uint8_t             bw_exponent   = 0;

    // Get the bitrate, and set the other parameters
    inst->config.bitrate = hal_config.bitrate;
    switch(inst->config.bitrate) {
        case HAL_RADIO_BITRATE_12_5:
            // Set the bandwidth to 50kHz
            bw_mantissa   = RFM69_RXBW_MANTISSA_20;
            bw_exponent   = 4;
            freq_dev      = 25000;
            rfm69_bitrate = RFM69_MODEM_BITRATE_12_5;
            break;
        case HAL_RADIO_BITRATE_25:
            // Set the bandwidth to 50kHz
            bw_mantissa   = RFM69_RXBW_MANTISSA_20;
            bw_exponent   = 3;
            freq_dev      = 50000;
            rfm69_bitrate = RFM69_MODEM_BITRATE_25;
            break;
        case HAL_RADIO_BITRATE_50:
            // Set the bandwidth to 100kHz
            bw_mantissa   = RFM69_RXBW_MANTISSA_20;
            bw_exponent   = 2;
            freq_dev      = 70000;
            rfm69_bitrate = RFM69_MODEM_BITRATE_50;
            break;
        case HAL_RADIO_BITRATE_100:
            // Set the bandwidth to 125kHz
            bw_mantissa   = RFM69_RXBW_MANTISSA_16;
            bw_exponent   = 2;
            freq_dev      = 70000;
            rfm69_bitrate = RFM69_MODEM_BITRATE_100;
            break;
        case HAL_RADIO_BITRATE_150:
            // Set the bandwidth to 250kHz
            bw_mantissa   = RFM69_RXBW_MANTISSA_16;
            bw_exponent   = 1;
            freq_dev      = 125000;
            rfm69_bitrate = RFM69_MODEM_BITRATE_150;
            break;
        case HAL_RADIO_BITRATE_200:
            // Set the bandwidth to 400kHz
            bw_mantissa   = RFM69_RXBW_MANTISSA_20;
            bw_exponent   = 0;
            freq_dev      = 200000;
            rfm69_bitrate = RFM69_MODEM_BITRATE_200;
            break;
        case HAL_RADIO_BITRATE_250:
            // Set the bandwidth to the maximum 500kHz
            bw_mantissa   = RFM69_RXBW_MANTISSA_16;
            bw_exponent   = 0;
            freq_dev      = 250000;
            rfm69_bitrate = RFM69_MODEM_BITRATE_250;
            break;
        case HAL_RADIO_BITRATE_300:
            // Set the bandwidth to the maximum 500kHz
            bw_mantissa   = RFM69_RXBW_MANTISSA_16;
            bw_exponent   = 0;
            freq_dev      = 150000;
            rfm69_bitrate = RFM69_MODEM_BITRATE_300;
            break;
        default:
            mutex_exit(&inst->mutex);
            return HAL_RADIO_INVALID_RATE;
    }

    if (!rfm69_bitrate_set(&inst->rfm, rfm69_bitrate)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    // Read and confirm bitrate
    uint16_t bitrate_read = 0;
    if (!rfm69_bitrate_get(&inst->rfm, &bitrate_read)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }
    if (bitrate_read != rfm69_bitrate) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_CONFIG_ERROR;
    }

    // Set the frequency deviation
    if (!rfm69_fdev_set(&inst->rfm, freq_dev)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    // Read and verify fdev
    uint32_t read_fdev;
    if (!rfm69_fdev_get(&inst->rfm, &read_fdev)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }
    if (read_fdev != rfm69_fdev_compute_closest(freq_dev)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_CONFIG_ERROR;
    }

    if (!rfm69_modulation_shaping_set(&inst->rfm, RFM69_FSK_GAUSSIAN_0_5)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    RFM69_MODULATION_SHAPING shape_read = RFM69_NO_SHAPING;
    if (!rfm69_modulation_shaping_get(&inst->rfm, &shape_read)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    if (!rfm69_modulation_afc_beta_set(&inst->rfm, true)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    bool beta_on = false;
    if (!rfm69_modulation_afc_beta_get(&inst->rfm, &beta_on)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    if (!beta_on) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_CONFIG_ERROR;
    }

    if (!rfm69_modulation_afc_set(&inst->rfm, 2)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    uint8_t afc = 0;
    if (!rfm69_modulation_afc_get(&inst->rfm, &afc)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    if (afc != 2) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_CONFIG_ERROR;
    }

    // Write channel frequency
    inst->config.channel = hal_config.channel;
    if (!rfm69_frequency_set(&inst->rfm, inst->config.channel)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    // Read and verify frequency
    uint32_t read_freq = 0;
    if (!rfm69_frequency_get(&inst->rfm, &read_freq)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }
    if (read_freq != rfm69_frequency_compute_closest(inst->config.channel)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_CONFIG_ERROR;
    }

    // RXBW >= fdev + br/2
    if (!rfm69_rxbw_set(&inst->rfm, bw_mantissa, bw_exponent)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    // Read and verify the bandwidth
    uint8_t mantissa = 0;
    uint8_t exponent = 0;
    if (!rfm69_rxbw_get(&inst->rfm, &mantissa, &exponent)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }
    if ((mantissa != bw_mantissa) || (exponent != bw_exponent)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_CONFIG_ERROR;
    }

    if (!rfm69_dcfree_set(&inst->rfm, RFM69_DCFREE_WHITENING)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    // Read and verify dcfree setting
    uint8_t reg_read = 0;
    if (!rfm69_dcfree_get(&inst->rfm, &reg_read)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }
    if (reg_read != RFM69_DCFREE_WHITENING) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_CONFIG_ERROR;
    }

    inst->config.power_dbm = hal_config.power_dbm;
    if (!rfm69_power_level_set(&inst->rfm, inst->config.power_dbm)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    // Store and set our address
    inst->config.rx_address = hal_config.rx_address;
    if (!rfm69_node_address_set(&inst->rfm, inst->config.rx_address)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    // Read and verify the node address
    if (!rfm69_node_address_get(&inst->rfm, &reg_read)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    if (reg_read != inst->config.rx_address) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_CONFIG_ERROR;
    }

    // Store and set the broadcast address
    inst->config.broadcast_address = hal_config.broadcast_address;
    if (!rfm69_broadcast_address_set(&inst->rfm, inst->config.broadcast_address)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    //Read and verify the broadcast address
    if (!rfm69_broadcast_address_get(&inst->rfm, &reg_read)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    if (reg_read != inst->config.broadcast_address) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_CONFIG_ERROR;
    }

    // Set packet format
    if (!rfm69_packet_format_set(&inst->rfm, RFM69_PACKET_VARIABLE)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    // Read and verify packet format
    if (!rfm69_packet_format_get(&inst->rfm, &reg_read)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    if (reg_read != RFM69_PACKET_VARIABLE) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_CONFIG_ERROR;
    }

    if (!rfm69_fifo_threshold_set(&inst->rfm, HAL_RADIO_FIFO_THRESHOLD)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    uint8_t thresh_verification = 0;
    if (!rfm69_fifo_threshold_get(&inst->rfm, &thresh_verification)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    if (thresh_verification != HAL_RADIO_FIFO_THRESHOLD) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_CONFIG_ERROR;
    }

    // We dont want the crc to autoclear becaus we want an interrupt to fire when the packet is done
    // no matter if it is correct or not, crc failed is explicitly managed.
    if (!rfm69_crc_autoclear_set(&inst->rfm, false)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    bool autoclear_set = true;
    if (!rfm69_crc_autoclear_get(&inst->rfm, &autoclear_set)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    if (autoclear_set) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_CONFIG_ERROR;
    }

    if (!rfm69_payload_length_set(&inst->rfm, HAL_RADIO_MAX_BUFFER_SIZE)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    uint8_t payload_len = 0;
    if (!rfm69_payload_length_get(&inst->rfm, &payload_len)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    if (payload_len != HAL_RADIO_MAX_BUFFER_SIZE) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_CONFIG_ERROR;
    }

    // Configure dio1 to create an interrupt when the fifo level exceeds the threshold
    // in either direction
    if (!rfm69_dio1_config_set(&inst->rfm, RFM69_DIO1_PKT_TX_FIFO_LVL)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    // Set the radio in default mode
    if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_STDBY)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    mutex_exit(&inst->mutex);
    return HAL_RADIO_SUCCESS;
}

int32_t halRadioCancelReceive(halRadio_t *inst) {

    uint32_t tst = 3;
    bool taken = mutex_try_enter(&inst->mutex, &tst);
    if (!taken) {
        LOG_DEBUG_BUSY("BUSY ERROR %i\n", 12);
        return HAL_RADIO_BUSY;
    }

    // Put radio back to default
    if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_STDBY)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    // Reset the GPIO callback
    if ((halGpioDisableIrqCb(HAL_RADIO_PIN_DIO0)) != HAL_GPIO_SUCCESS) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_GPIO_ERROR;
    }

    // Reset the GPIO callback
    if ((halGpioDisableIrqCb(HAL_RADIO_PIN_DIO1)) != HAL_GPIO_SUCCESS) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_GPIO_ERROR;
    }

    inst->mode = HAL_RADIO_IDLE;

    mutex_exit(&inst->mutex);
    return HAL_RADIO_SUCCESS;
}

int32_t halRadioReceivePackageNB(halRadio_t *inst, halRadioInterface_t *interface, bool wait_for_rx_mode) {
    if (inst == NULL || interface == NULL || interface->package_cb == NULL || interface->pkt_buffer == NULL) {
        return HAL_RADIO_NULL_ERROR;
    }

    uint32_t tst = 4;
    bool taken = mutex_try_enter(&inst->mutex, &tst);
    if (!taken) {
        LOG_DEBUG_BUSY("BUSY ERROR %i\n", 13);
        return HAL_RADIO_BUSY;
    }
 
    // Check if the radio is busy
    if (inst->mode == HAL_RADIO_TX) {
        LOG("Radio TX interrupted\n");

        // Notify that the package send failed
        int32_t cb_res = HAL_RADIO_SUCCESS;
        if (inst->package_callback != NULL && inst->package_callback->pkg_sent_cb != NULL) {
            cb_res = inst->package_callback->pkg_sent_cb(inst->package_callback, &inst->active_package, HAL_RADIO_SEND_INTERRUPTED);
        } else {
            mutex_exit(&inst->mutex);
            return HAL_RADIO_NULL_ERROR;
        }

        // Check if we shuld proceed or give up
        if (cb_res != HAL_RADIO_SUCCESS) {
            mutex_exit(&inst->mutex);
            return cb_res;
        }

        // Reset the active package address
        inst->active_package.address = 0;
        // Now move on with going to Rx!
    }

    // Check if the radio is already in the correct state
    if (inst->mode == HAL_RADIO_RX) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_SUCCESS;
    }

    // Enable gpio interrupt and set callback
    inst->package_callback = interface;
    if (halGpioEnableIrqCbRisingEdge(&inst->gpio_dio0, HAL_RADIO_PIN_DIO0) != HAL_GPIO_SUCCESS) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_GPIO_ERROR;
    }

    // Enable gpio callbacks
    if (halGpioEnableIrqCbRisingEdge(&inst->gpio_dio1, HAL_RADIO_PIN_DIO1) != HAL_GPIO_SUCCESS) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_GPIO_ERROR;
    }

    // Configure dio0 to create an interrupt on payload ready
    if (!rfm69_dio0_config_set(&inst->rfm, RFM69_DIO0_PKT_RX_PAYLOAD_READY)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    if (wait_for_rx_mode) {
        // Try to set RX mode, and wait to see if it is successfull
        if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_RX)) {
            mutex_exit(&inst->mutex);
            return HAL_RADIO_DRIVER_ERROR;
        }
    } else {
        // Switch to RX mode but dont wait for the mode transition
        if (!rfm69_write_masked(&inst->rfm, RFM69_REG_OP_MODE, RFM69_OP_MODE_RX, RFM69_OP_MODE_MASK)) {
            mutex_exit(&inst->mutex);
            return HAL_RADIO_DRIVER_ERROR;
        }
        inst->rfm.op_mode = RFM69_OP_MODE_RX;
    }

    inst->mode = HAL_RADIO_RX;

    mutex_exit(&inst->mutex);
    return HAL_RADIO_SUCCESS;
}

int32_t halRadioReceivePackageBlockingInterface(halRadio_t *inst, halRadioInterface_t *interface, uint32_t time_ms) {
    if (inst == NULL) {
        return HAL_RADIO_NULL_ERROR;
    }

    // Get the buffer pointer
    cBuffer_t *rx_buf = interface->pkt_buffer;
    // Enable gpio interrupt and set callback
    inst->package_callback = interface;

    // Receive blocking
    int32_t res = halRadioReceivePackageBlocking(inst, rx_buf, time_ms);
    if (res != HAL_RADIO_SUCCESS) {
        return res;
    }

    // |SIZE|ADDR|PAYLOAD|, size = sizeof(addr) + sizeof(payload)
    // Get the target address for this package, should be this radio ..
    inst->active_package.address = cBufferReadByte(rx_buf);

    int32_t cb_res = HAL_RADIO_CB_SUCCESS;
    // Notify about new package
    if (inst->package_callback != NULL && inst->package_callback->package_cb != NULL) {
        cb_res = inst->package_callback->package_cb(inst->package_callback, &inst->active_package);
    }

    // Manage result from callback
    switch(cb_res) {
        case HAL_RADIO_CB_SUCCESS:
            // Try to return radio to default mode
            if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_STDBY)) {
                return HAL_RADIO_DRIVER_ERROR;
            }

            // Reset the GPIO callback
            if ((halGpioDisableIrqCb(HAL_RADIO_PIN_DIO0)) != HAL_GPIO_SUCCESS) {
                return HAL_RADIO_GPIO_ERROR;
            }

            // Reset the GPIO callback
            if ((halGpioDisableIrqCb(HAL_RADIO_PIN_DIO1)) != HAL_GPIO_SUCCESS) {
                return HAL_RADIO_GPIO_ERROR;
            }

            inst->mode = HAL_RADIO_IDLE;
            return HAL_RADIO_SUCCESS;
        case HAL_RADIO_CB_DO_NOTHING:
            // THis indicates that the higher layer has called a hal function
            // Re-Enable gpio callback for fifo level
            if (halGpioEnableIrqCbRisingEdge(&inst->gpio_dio1, HAL_RADIO_PIN_DIO1) != HAL_GPIO_SUCCESS) {
                return HAL_RADIO_GPIO_ERROR;
            }
            break;
        default:
            // Try to return radio to default mode
            if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_STDBY)) {
                return HAL_RADIO_DRIVER_ERROR;
            }

            // Reset the GPIO callback
            if ((halGpioDisableIrqCb(HAL_RADIO_PIN_DIO0)) != HAL_GPIO_SUCCESS) {
                return HAL_RADIO_GPIO_ERROR;
            }

            inst->mode = HAL_RADIO_IDLE;
            return cb_res;
    }

    return HAL_RADIO_SUCCESS;
}

int32_t halRadioReceivePackageBlocking(halRadio_t *inst, cBuffer_t *rx_buf, uint32_t time_ms) {
    if (inst == NULL || rx_buf == NULL || rx_buf->data == NULL) {
        return HAL_RADIO_NULL_ERROR;
    }

    // Try to set RX mode
    if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_RX)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    // Reset the IRQ registers
    uint8_t buf[2] = {0xFF,0xFF};
    if (!rfm69_write(&inst->rfm, RFM69_REG_IRQ_FLAGS_1, buf, 2)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    bool state = false;
    bool fifo_state = false;
    uint32_t count = 0;
    uint8_t read_bytes = 0;

    if (!rfm69_irq2_flag_state(&inst->rfm, RFM69_IRQ2_FLAG_FIFO_NOT_EMPTY, &state)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    // Make sure to empty the fifo
    while (state)
    {
        uint8_t tmp = 0;
        // Try to Read payload size byte from FIFO
        if (!rfm69_read(&inst->rfm, RFM69_REG_FIFO, &tmp, 1)) {
            return HAL_RADIO_DRIVER_ERROR;
        }

        if (!rfm69_irq2_flag_state(&inst->rfm, RFM69_IRQ2_FLAG_FIFO_NOT_EMPTY, &state)) {
            return HAL_RADIO_DRIVER_ERROR;
        }
    }

    // Wait for fifo threshold, or payload ready
    while ((!state && !fifo_state) && count < time_ms) {
        if (!rfm69_irq2_flag_state(&inst->rfm, RFM69_IRQ2_FLAG_PAYLOAD_READY, &state)) {
            return HAL_RADIO_DRIVER_ERROR;
        }

        if (!rfm69_irq2_flag_state(&inst->rfm, RFM69_IRQ2_FLAG_FIFO_LEVEL, &fifo_state)) {
            return HAL_RADIO_DRIVER_ERROR;
        }

        sleep_us(PACKET_RX_POLL_TIME_MS);
        count += PACKET_RX_POLL_TIME_MS;
    }

    // Check if any package was received
    if (!state && !fifo_state) {
        // Try to return to default mode
        if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_STDBY)) {
            return HAL_RADIO_DRIVER_ERROR;
        }

        return HAL_RADIO_RECEIVE_FAIL;
    }

    // Clear the buffer to make room for a new package
    if (cBufferClear(rx_buf) < 0) {
        return HAL_RADIO_BUFFER_ERROR;
    }

    // Try to Read payload size byte from FIFO
    if (!rfm69_read(&inst->rfm, RFM69_REG_FIFO, &inst->current_packet_size, 1)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    // Check if the packet size is valid
    if (inst->current_packet_size == 0 || inst->current_packet_size > cBufferAvailableForWrite(rx_buf)) {
        return HAL_RADIO_INVALID_SIZE;
    }

    int32_t expected_time = halRadioBitRateToDelayUs(inst, inst->config.bitrate, inst->current_packet_size) + halRadioSpiDelayEstimateUs(inst, inst->current_packet_size);
    uint64_t s_time = time_us_64();
    uint64_t e_time = 0;

    // Check if a this is a large packet that needs to be read from the fifo in steps
    if (inst->current_packet_size > RFM69_FIFO_SIZE) {
        LOG_DEBUG("Large packet transfer 1st %u\n", inst->current_packet_size);

        while (read_bytes < inst->current_packet_size && e_time < expected_time) {
            e_time = time_us_64() - s_time;
            // Get the current write pointer
            uint8_t * raw_rx_buffer = NULL;
            if ((raw_rx_buffer = cBufferGetWritePointer(rx_buf)) == NULL){
                return HAL_RADIO_BUFFER_ERROR;
            }

            if (!rfm69_irq2_flag_state(&inst->rfm, RFM69_IRQ2_FLAG_FIFO_NOT_EMPTY, &state)) {
                    return HAL_RADIO_DRIVER_ERROR;
            }

            if (state) {
                // Try to Read 1 byte
                if (!rfm69_read(&inst->rfm, RFM69_REG_FIFO, raw_rx_buffer, 1)) {
                    return HAL_RADIO_DRIVER_ERROR;
                }

                read_bytes++;
                LOG_DEBUG("Bytes read %u, total %u, %c\n", read_bytes, inst->current_packet_size, *raw_rx_buffer);

                // Make sure that the buffer knows how many bytes is in the data array
                if (cBufferEmptyWrite(rx_buf, 1) < 0) {
                    return HAL_RADIO_BUFFER_ERROR;
                }
            } else {
                if (!rfm69_irq2_flag_state(&inst->rfm, RFM69_IRQ2_FLAG_PAYLOAD_READY, &state)) {
                    return HAL_RADIO_DRIVER_ERROR;
                }

                if (state) {
                    // Try to Read 1 byte
                    if (!rfm69_read(&inst->rfm, RFM69_REG_FIFO, raw_rx_buffer, 1)) {
                        return HAL_RADIO_DRIVER_ERROR;
                    }

                    read_bytes++;
                    LOG_DEBUG("Bytes read %u, total %u, %c\n", read_bytes, inst->current_packet_size, *raw_rx_buffer);

                    // Make sure that the buffer knows how many bytes is in the data array
                    if (cBufferEmptyWrite(rx_buf, 1) < 0) {
                        return HAL_RADIO_BUFFER_ERROR;
                    }
                }
            }
        }

        if (read_bytes != inst->current_packet_size) {
            LOG("Packet timeout received %u\n", read_bytes);
            // Try to return to default mode
            if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_STDBY)) {
                return HAL_RADIO_DRIVER_ERROR;
            }
            return HAL_RADIO_RECEIVE_FAIL;
        }
    } else {
        // Check if the complete payload has arrived
        if (state) {
            // Get the current write pointer
            uint8_t * raw_rx_buffer = NULL;
            if ((raw_rx_buffer = cBufferGetWritePointer(rx_buf)) == NULL){
                return HAL_RADIO_BUFFER_ERROR;
            }

            // Try to Read remaining data bytes from FIFO
            if (!rfm69_read(&inst->rfm, RFM69_REG_FIFO, raw_rx_buffer, inst->current_packet_size)) {
                return HAL_RADIO_DRIVER_ERROR;
            }

            // Make sure that the buffer knows how many bytes is in the data array
            if (cBufferEmptyWrite(rx_buf, inst->current_packet_size) < 0) {
                return HAL_RADIO_BUFFER_ERROR;
            }
        } else {

            // Wait for payload ready
            while (!state && e_time < expected_time) {
                e_time = time_us_64() - s_time;
                if (!rfm69_irq2_flag_state(&inst->rfm, RFM69_IRQ2_FLAG_PAYLOAD_READY, &state)) {
                    return HAL_RADIO_DRIVER_ERROR;
                }

                sleep_us(100);
            }

            // Check if any package was received
            if (!state) {
                // Try to return to default mode
                if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_STDBY)) {
                    return HAL_RADIO_DRIVER_ERROR;
                }

                return HAL_RADIO_RECEIVE_FAIL;
            }
        }
    }

    return HAL_RADIO_SUCCESS;
}

int32_t halRadioCancelTransmit(halRadio_t *inst) {
    uint32_t tst = 5;
    bool taken = mutex_try_enter(&inst->mutex, &tst);
    if (!taken) {
        LOG_DEBUG_BUSY("BUSY ERROR %i\n", 15);
        return HAL_RADIO_BUSY;
    }

    // Put radio back to default
    if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_STDBY)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    // Reset the GPIO callback
    if ((halGpioDisableIrqCb(HAL_RADIO_PIN_DIO0)) != HAL_GPIO_SUCCESS) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_GPIO_ERROR;
    }

    // Reset the GPIO callback
    if ((halGpioDisableIrqCb(HAL_RADIO_PIN_DIO1)) != HAL_GPIO_SUCCESS) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_GPIO_ERROR;
    }

    inst->mode = HAL_RADIO_IDLE;

    mutex_exit(&inst->mutex);

    return HAL_RADIO_SUCCESS;
}

static int32_t byteWiseWrite(halRadio_t *inst, cBuffer_t *tx_buf, uint8_t num_bytes) {
    bool taken = mutex_try_enter(&inst->mutex, NULL);
    if (!taken) {
        LOG_DEBUG_BUSY("BUSY ERROR %i\n", 16);
        return HAL_RADIO_BUSY;
    }

    uint64_t s_time = time_us_64();
    uint64_t e_time = 0;
    int32_t expected_time = halRadioBitRateToDelayUs(inst, inst->config.bitrate, num_bytes) + halRadioSpiDelayEstimateUs(inst, num_bytes);
    uint8_t writen_bytes = 0;
    bool    state = false;

    while (writen_bytes < num_bytes && e_time < expected_time) {
        e_time = time_us_64() - s_time;

        // Check if the buffer is full
        if (!rfm69_irq2_flag_state(&inst->rfm, RFM69_IRQ2_FLAG_FIFO_FULL, &state)) {
            mutex_exit(&inst->mutex);
            return HAL_RADIO_DRIVER_ERROR; // Fatal error
        }

        if (!state) {
            uint8_t next_byte = cBufferReadByte(tx_buf);
            // Try to write 1 byte
            if (!rfm69_write(&inst->rfm, RFM69_REG_FIFO, &next_byte, 1)) {
                mutex_exit(&inst->mutex);
                return HAL_RADIO_DRIVER_ERROR; // Fatal error
            }

            writen_bytes++;
            LOG_DEBUG("Bytes written %u, total %u, %c\n", writen_bytes, inst->current_packet_size, *raw_rx_buffer);
        }
    }

    if (writen_bytes != num_bytes) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_SEND_FAIL;
    }

    mutex_exit(&inst->mutex);
    return HAL_RADIO_SUCCESS;
}

static int32_t writeDataAndEnableTx(halRadio_t *inst, cBuffer_t *pkt_buffer, uint8_t address, bool not_blocking) {
	// Variable packet format pg.52 of datasheet, the size of size byte should not be included in size

    // Get the current size of the buffer contents
    int32_t result = 0;
    if ((result = cBufferAvailableForRead(pkt_buffer)) <= 0 || result > 253) {
        LOG("Hal Radio buffer insufficent %i\n", result);
        return HAL_RADIO_BUFFER_ERROR;
    }

    uint8_t pkt_size = (uint8_t)result;

    // Prepend the address of the packet
    if ((result = cBufferPrepend(pkt_buffer, &address, HAL_RADIO_PACKET_ADDR_SIZE)) != HAL_RADIO_PACKET_ADDR_SIZE) {
        return HAL_RADIO_BUFFER_ERROR;
    }

    // Prepend the size of the packet, including the address byte
    pkt_size += HAL_RADIO_PACKET_ADDR_SIZE;
    if ((result = cBufferPrepend(pkt_buffer, &pkt_size, HAL_RADIO_PACKET_SIZE_SIZE)) != HAL_RADIO_PACKET_SIZE_SIZE) {
        return HAL_RADIO_BUFFER_ERROR;
    }

    // Make sure the the contents in the pkt_buffer is in continous memeory
    if ((result = cBufferContiguate(pkt_buffer)) != C_BUFFER_SUCCESS) {
        return HAL_RADIO_BUFFER_ERROR;
    }

    // Get the pointer to the packet to send
    uint8_t *tx_buffer = NULL;
    if ((tx_buffer = cBufferGetReadPointer(pkt_buffer)) == NULL) {
        return HAL_RADIO_BUFFER_ERROR;
    }

    // Set the current packet size
    inst->current_packet_size = pkt_size + HAL_RADIO_PACKET_SIZE_SIZE;
    // When it is an outgoing packet the address is my address
    inst->active_package.address = inst->config.rx_address;

    // Set the tx power level
    if (!rfm69_power_level_set(&inst->rfm, inst->config.power_dbm)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    // Reset the IRQ registers
    uint8_t buf[2] = {0xFF,0xFF};
    if (!rfm69_write(&inst->rfm, RFM69_REG_IRQ_FLAGS_1, buf, 2)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    if (inst->current_packet_size > RFM69_FIFO_SIZE) {
        // Write the package to the radio
        if (!rfm69_write(&inst->rfm, RFM69_REG_FIFO, tx_buffer, (RFM69_FIFO_SIZE))) {
            return HAL_RADIO_DRIVER_ERROR;
        }

       LOG_DEBUG("Writing %u bytes\n", RFM69_FIFO_SIZE);
       // Update the buffer
       if (cBufferEmptyRead(pkt_buffer, RFM69_FIFO_SIZE) != RFM69_FIFO_SIZE) {
           return HAL_RADIO_BUFFER_ERROR;
       }

        // Check if we are allready in tx state
        if (inst->mode != HAL_RADIO_TX_IDLE) {
            // Switch to TX mode to send buffer
            if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_TX)) {
                return HAL_RADIO_DRIVER_ERROR;
            }
        }

        inst->current_packet_size -= RFM69_FIFO_SIZE;

        if (not_blocking) {
            // Enable gpio fifo callbacks
            if (halGpioEnableIrqCbFallingEdge(&inst->gpio_dio1, HAL_RADIO_PIN_DIO1) != HAL_GPIO_SUCCESS) {
                return HAL_RADIO_GPIO_ERROR;
            }

            // The rest is managed in the interrupt handler
            inst->radio_state = HAL_RADIO_TX_DATA;
            return HAL_RADIO_SUCCESS;
        }

        while (inst->current_packet_size > 0) {
            bool state = true;
            // Wait for the fifo level interrupt going under the threshold
            while (state) {
                if (!rfm69_irq2_flag_state(&inst->rfm, RFM69_IRQ2_FLAG_FIFO_LEVEL, &state)) {
                    return HAL_RADIO_DRIVER_ERROR;
                }
            }

            // Check if there still is more to send than fits in the fifo
            if (inst->current_packet_size > HAL_RADIO_FIFO_THRESHOLD) {
                LOG_DEBUG("Writing %u bytes\n", HAL_RADIO_FIFO_THRESHOLD);
                if (byteWiseWrite(inst, pkt_buffer, HAL_RADIO_FIFO_THRESHOLD) != HAL_RADIO_SUCCESS) {
                    return HAL_RADIO_SEND_FAIL;
                }

                inst->current_packet_size -= HAL_RADIO_FIFO_THRESHOLD;
            } else {
                LOG_DEBUG("Writing %u bytes\n", inst->current_packet_size);
                // Write the rest of the data
                if (byteWiseWrite(inst, pkt_buffer, inst->current_packet_size) != HAL_RADIO_SUCCESS) {
                    return HAL_RADIO_SEND_FAIL;
                }

                // Transfer complete
                inst->current_packet_size = 0;

                if (cBufferClear(pkt_buffer) != C_BUFFER_SUCCESS) {
                    return HAL_RADIO_BUFFER_ERROR;
                }
            }
        }
    } else {
        // Write the package to the radio
        if (!rfm69_write(&inst->rfm, RFM69_REG_FIFO, tx_buffer, inst->current_packet_size)) {
            return HAL_RADIO_DRIVER_ERROR;
        }

        if (cBufferClear(pkt_buffer) != C_BUFFER_SUCCESS) {
            return HAL_RADIO_BUFFER_ERROR;
        }

        if (inst->mode == HAL_RADIO_TX_IDLE) {
            return HAL_RADIO_SUCCESS;
        }

        // Switch to TX mode to send buffer
        if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_TX)) {
            return HAL_RADIO_DRIVER_ERROR;
        }
    }

    return HAL_RADIO_SUCCESS;
}

int32_t halRadioEnterTX(halRadio_t *inst) {
    if (inst == NULL) {
        return HAL_RADIO_NULL_ERROR;
    }

    bool taken = mutex_try_enter(&inst->mutex, NULL);
    if (!taken) {
        LOG_DEBUG_BUSY("BUSY ERROR %i\n", 17);
        return HAL_RADIO_BUSY;
    }

    // Set the tx power level
    if (!rfm69_power_level_set(&inst->rfm, inst->config.power_dbm)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    // Switch to TX mode to send buffer
    if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_TX)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    inst->mode = HAL_RADIO_IDLE;

    mutex_exit(&inst->mutex);
    return HAL_RADIO_SUCCESS;
}

int32_t halRadioSendPackageNB(halRadio_t *inst, halRadioInterface_t *interface, uint8_t address) {
    if (inst == NULL || interface == NULL || interface->pkg_sent_cb == NULL || interface->pkt_buffer == NULL) {
        return HAL_RADIO_NULL_ERROR;
    }

    uint32_t tst = 6;
    bool taken = mutex_try_enter(&inst->mutex, &tst);
    if (!taken) {
        LOG_DEBUG_BUSY("BUSY ERROR %i\n", 18);
        return HAL_RADIO_BUSY;
    }

    // Check if the radio is ready for transmitt
    if (inst->mode == HAL_RADIO_TX) {
        mutex_exit(&inst->mutex);
        LOG_DEBUG_BUSY("BUSY ERROR %i\n", 19);
        return HAL_RADIO_BUSY;
    }

    // Check package size validity
    int32_t free_space = cBufferAvailableForWrite(interface->pkt_buffer);
    if (free_space < 0 || free_space < HAL_RADIO_PACKET_OVERHEAD) {
        LOG("Invalid size %u\n", free_space);
        mutex_exit(&inst->mutex);
        return HAL_RADIO_INVALID_SIZE;
    }

    // Enable gpio callbacks
    inst->package_callback = interface;
    if (halGpioEnableIrqCbRisingEdge(&inst->gpio_dio0, HAL_RADIO_PIN_DIO0) != HAL_GPIO_SUCCESS) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_GPIO_ERROR;
    }

    // Configure dio0 to trigger packet sent interrupt
    if (!rfm69_dio0_config_set(&inst->rfm, RFM69_DIO0_PKT_TX_PACKET_SENT)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    int32_t res;
    if ((res = writeDataAndEnableTx(inst, interface->pkt_buffer, address, true)) != HAL_RADIO_SUCCESS) {
        mutex_exit(&inst->mutex);
        return res;
    }

    inst->mode = HAL_RADIO_TX;

    mutex_exit(&inst->mutex);

    return HAL_RADIO_SUCCESS;
}

int32_t halRadioQueueSend(halRadio_t *inst, bool wait_for_tx_mode) {
    if (inst == NULL) {
        return HAL_RADIO_NULL_ERROR;
    }

    uint32_t tst = 7;
    bool taken = mutex_try_enter(&inst->mutex, &tst);
    if (!taken) {
        LOG_DEBUG_BUSY("BUSY ERROR %i\n", 20);
        return HAL_RADIO_BUSY;
    }

    // Check if the radio is ready for transmitt
    if (inst->mode != HAL_RADIO_TX_QUEUE) {
        // Nothing to send. Which is fine
        mutex_exit(&inst->mutex);
        return HAL_RADIO_SUCCESS;
    }

    if (wait_for_tx_mode) {
        // Switch to TX mode to send FIFO contents, and wait for the radio to enter TX
        if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_TX)) {
            mutex_exit(&inst->mutex);
            return HAL_RADIO_DRIVER_ERROR;
        }
    } else {
        // Switch to TX mode to send FIFO contents, but do not wait for the radio to enter TX
        if (!rfm69_write_masked(&inst->rfm, RFM69_REG_OP_MODE, RFM69_OP_MODE_TX, RFM69_OP_MODE_MASK)) {
            mutex_exit(&inst->mutex);
            return HAL_RADIO_DRIVER_ERROR;
        }
        inst->rfm.op_mode = RFM69_OP_MODE_TX;
    }

    // Radio is in TX state
    inst->mode = HAL_RADIO_TX;
    mutex_exit(&inst->mutex);

    return HAL_RADIO_SUCCESS;
}

int32_t halRadioQueuePackage(halRadio_t *inst, halRadioInterface_t *interface, uint8_t address) {
    if (inst == NULL || interface == NULL || interface->pkg_sent_cb == NULL) {
        return HAL_RADIO_NULL_ERROR;
    }

    uint32_t tst = 8;
    bool taken = mutex_try_enter(&inst->mutex, &tst);
    if (!taken) {
        LOG_DEBUG_BUSY("BUSY ERROR %i\n", 21);
        return HAL_RADIO_BUSY;
    }

    // Check if the radio is ready for transmitt
    if (inst->mode != HAL_RADIO_IDLE) {
        mutex_exit(&inst->mutex);
        LOG_DEBUG_BUSY("BUSY ERROR %i\n", 22);
        return HAL_RADIO_BUSY;
    }

    // Check package size validity
    int32_t free_space = cBufferAvailableForWrite(interface->pkt_buffer);
    if (free_space < 0 || free_space < HAL_RADIO_PACKET_OVERHEAD) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_INVALID_SIZE;
    }

    // Get the current size of the buffer contents
    int32_t result = 0;
    if ((result = cBufferAvailableForRead(interface->pkt_buffer)) <= 0 || result > 253) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_BUFFER_ERROR;
    }

    uint8_t pkt_size = (uint8_t)result;

    // Prepend the address of the packet
    if ((result = cBufferPrepend(interface->pkt_buffer, &address, HAL_RADIO_PACKET_ADDR_SIZE)) != HAL_RADIO_PACKET_ADDR_SIZE) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_BUFFER_ERROR;
    }

    // Prepend the size of the packet, including the address byte
    pkt_size += HAL_RADIO_PACKET_ADDR_SIZE;
    if ((result = cBufferPrepend(interface->pkt_buffer, &pkt_size, HAL_RADIO_PACKET_SIZE_SIZE)) != HAL_RADIO_PACKET_SIZE_SIZE) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_BUFFER_ERROR;
    }

    // Make sure the the contents in the pkt_buffer is in continous memeory
    if ((result = cBufferContiguate(interface->pkt_buffer)) != C_BUFFER_SUCCESS) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_BUFFER_ERROR;
    }

    // Get the pointer to the packet to send
    uint8_t *tx_buffer = NULL;
    if ((tx_buffer = cBufferGetReadPointer(interface->pkt_buffer)) == NULL) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_BUFFER_ERROR;
    }

    // Set the current packet size
    inst->current_packet_size = pkt_size + HAL_RADIO_PACKET_SIZE_SIZE;

    // Write the package to the radio
    if (!rfm69_write(&inst->rfm, RFM69_REG_FIFO, tx_buffer, inst->current_packet_size)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    // Enable gpio callbacks
    inst->package_callback = interface;
    if (cBufferClear(inst->package_callback->pkt_buffer) != C_BUFFER_SUCCESS) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_BUFFER_ERROR;
    }

    // Configure dio0 to trigger packet sent interrupt
    if (!rfm69_dio0_config_set(&inst->rfm, RFM69_DIO0_PKT_TX_PACKET_SENT)) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_DRIVER_ERROR;
    }

    if (halGpioEnableIrqCbRisingEdge(&inst->gpio_dio0, HAL_RADIO_PIN_DIO0) != HAL_GPIO_SUCCESS) {
        mutex_exit(&inst->mutex);
        return HAL_RADIO_GPIO_ERROR;
    }

    // We are now in fifo queued state
    inst->mode = HAL_RADIO_TX_QUEUE;
    mutex_exit(&inst->mutex);

    return HAL_RADIO_SUCCESS;
}

int32_t halRadioSendPackageBlocking(halRadio_t *inst, cBuffer_t *pkt_buffer, uint8_t address) {
    if (inst == NULL || pkt_buffer == NULL) {
        return HAL_RADIO_NULL_ERROR;
    }

    // Check package size validity
    int32_t free_space = cBufferAvailableForWrite(pkt_buffer);
    if (free_space < 0 || free_space < HAL_RADIO_PACKET_OVERHEAD) {
        return HAL_RADIO_INVALID_SIZE;
    }

    // Get the number of bytes to send
    int32_t to_send = cBufferAvailableForRead(pkt_buffer);

    int32_t res;
    if ((res = writeDataAndEnableTx(inst, pkt_buffer, address, false)) != HAL_RADIO_SUCCESS) {
        return res;
    }

    inst->mode = HAL_RADIO_TX;

	uint8_t buf; // Buffer to hold read result from radio
    // Wait for packet sent flag
    bool state = false;
    uint32_t counter = 0; // Set a maximum timout for sending
    while (!state && counter < 10) {
        if (!rfm69_read(&inst->rfm, RFM69_REG_IRQ_FLAGS_2, &buf, 1)) {
            return HAL_RADIO_DRIVER_ERROR;
        }

        LOG_DEBUG("IRQ2: %02X\n", buf);

        if (!rfm69_read(&inst->rfm, RFM69_REG_IRQ_FLAGS_1, &buf, 1)) {
            return HAL_RADIO_DRIVER_ERROR;
        }

        LOG_DEBUG("IRQ1: %02X\n", buf);
        if (!rfm69_irq2_flag_state(&inst->rfm, RFM69_IRQ2_FLAG_PACKET_SENT, &state)) {
            return HAL_RADIO_DRIVER_ERROR;
        }

        sleep_ms(1);
        counter++;
    }

    inst->mode = HAL_RADIO_IDLE;

    // Put radio back to default mode
    if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_STDBY)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    LOG_DEBUG("%u bytes sent!\n", to_send);

    if (state == 0) {
        return HAL_RADIO_SEND_FAIL;
    }

    return HAL_RADIO_SUCCESS;
}

int32_t halRadioGetMode(halRadio_t *inst) {
    if (inst == NULL) {
        return HAL_RADIO_NULL_ERROR;
    }
    return inst->mode;
}

int32_t halRadioBitRateToDelayUs(halRadio_t *inst, halRadioBitrate_t bitrate, uint8_t num_bytes) {
    if (inst == NULL || num_bytes == 0) {
        return HAL_RADIO_NULL_ERROR;
    }

    // Add the overhead created by the radio and hal layer
    num_bytes += RFM69_DEFAULT_SYNC_WORD_LEN + RFM69_DEFAULT_PREAMBLE_LEN + HAL_RADIO_PACKET_OVERHEAD;

    int32_t time_us = 0;

    switch(bitrate) {
        case HAL_RADIO_BITRATE_12_5: {
            // Calculate the time i takes to send a single bit, round up
            int32_t bit_time_us = 1000000/12500 + 1;
            time_us = bit_time_us * num_bytes * 8;
        } break;
        case HAL_RADIO_BITRATE_25: {
            int32_t bit_time_us = 1000000/25000 + 1;
            time_us = bit_time_us * num_bytes * 8;
        } break;
        case HAL_RADIO_BITRATE_50: {
            int32_t bit_time_us = 1000000/50000 + 1;
            time_us = bit_time_us * num_bytes * 8;
        } break;
        case HAL_RADIO_BITRATE_100: {
            // 51us interrupt delay
            int32_t bit_time_us = 1000000/100000 + 1;
            time_us = bit_time_us * num_bytes * 8;
        } break;
        case HAL_RADIO_BITRATE_150: {
            // 29us interrupt delay
            int32_t bit_time_us = 1000000/150000 + 1;
            time_us = bit_time_us * num_bytes * 8;
        } break;
        case HAL_RADIO_BITRATE_200: {
            // 21us interrupt delay
            int32_t bit_time_us = 1000000/200000 + 1;
            time_us = bit_time_us * num_bytes * 8;
        } break;
        case HAL_RADIO_BITRATE_250: {
            int32_t bit_time_us = 1000000/250000 + 1;
            time_us = bit_time_us * num_bytes * 8;
        } break;
        case HAL_RADIO_BITRATE_300: {
            int32_t bit_time_us = 1000000/300000 + 1;
            time_us = bit_time_us * num_bytes * 8;
        } break;
        default:
            return HAL_RADIO_INVALID_RATE;
    }

    return time_us;
}

int32_t halRadioSpiDelayEstimateUs(halRadio_t *inst, uint8_t num_bytes) {
    if (inst == NULL || num_bytes == 0) {
        return HAL_RADIO_NULL_ERROR;
    }

    // Verify that the value of the num_bytes is withing the maximum value
    if (num_bytes > (255 - (RFM69_DEFAULT_SYNC_WORD_LEN + RFM69_DEFAULT_PREAMBLE_LEN + HAL_RADIO_PACKET_OVERHEAD))) {
        return HAL_RADIO_INVALID_SIZE;
    }

    // Add the overhead created by the radio and hal layer
    num_bytes += RFM69_DEFAULT_SYNC_WORD_LEN + RFM69_DEFAULT_PREAMBLE_LEN + HAL_RADIO_PACKET_OVERHEAD;

    // The maximum number of bytes to ever be consecutively read/written is RFM69_FIFO_SIZE
    // If the packet send or received is larger than RFM69_FIFO_SIZE the read/write will be done in the background
    // during the packet transmission. This function aims to estimate how much time in between packets is needed
    // due to SPI tranfers.
    num_bytes = MIN(num_bytes, RFM69_FIFO_SIZE);

    int32_t time_us = (int32_t)num_bytes*kHalRadioSpiUsPerByte;

    return time_us;
}

int32_t halRadioCheckBusy(halRadio_t *inst) {
    uint32_t tst = 9;
    bool taken = mutex_try_enter(&inst->mutex, &tst);
    if (!taken) {
        LOG_DEBUG_BUSY("BUSY ERROR %i\n", 23);
        return HAL_RADIO_BUSY;
    }

    mutex_exit(&inst->mutex);
    return HAL_RADIO_SUCCESS;
}