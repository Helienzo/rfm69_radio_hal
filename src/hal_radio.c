/**
 * @file:       hal_radio.c
 * @author:     Lucas Wennerholm <lucas.wennerholm@gmail.com>
 * @brief:      Implementation of radio HAL layer
 *
 * @license: MIT License
 *
 * Copyright (c) 2024 Lucas Wennerholm
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

#include "hal_radio.h"

#ifndef LOG
#define LOG(f_, ...) printf((f_), ##__VA_ARGS__)
#endif

#define PACKET_RX_POLL_TIME_MS 10

// Constant for calculating time used by the spi to send X bytes
static const float kHalRadioSpiUsPerByte = (8.0f * 1000000.0f)/HAL_RADIO_SPI_BAUD_RATE;

static int32_t managePacketSent(halRadio_t *inst) {
    // Read the packet sent flag
    bool state = false;
    int32_t cb_res = HAL_RADIO_CB_SUCCESS;
    rfm69_irq2_flag_state(&inst->rfm, RFM69_IRQ2_FLAG_PACKET_SENT, &state);

    // If the package was not sent this is an invalid interrupt, bad ..
    if (!state) {
        // Try to return to default mode
        if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_DEFAULT)) {
            return HAL_RADIO_DRIVER_ERROR;
        }

        // Reset the GPIO callback
        if ((halGpioDisableIrqCb(HAL_RADIO_PIN_DIO0)) != HAL_GPIO_SUCCESS) {
            return HAL_RADIO_GPIO_ERROR;
        }

        // Reset the radio mode
        inst->mode = HAL_RADIO_IDLE;

        // Notify that the package send failed
        if (inst->package_callback != NULL && inst->package_callback->pkg_sent_cb != NULL) {
            cb_res = inst->package_callback->pkg_sent_cb(inst->package_callback, &inst->active_package, HAL_RADIO_SEND_FAIL);
        }
    } else {
        inst->mode = HAL_RADIO_TX_IDLE;

        // Notify that the package was successfully sent
        if (inst->package_callback != NULL && inst->package_callback->pkg_sent_cb != NULL) {
            cb_res = inst->package_callback->pkg_sent_cb(inst->package_callback, &inst->active_package, HAL_RADIO_SUCCESS);
        }
    }

    // Manage result from callback
    switch(cb_res) {
        case HAL_RADIO_CB_SUCCESS:
            // Try to return radio to default mode
            if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_DEFAULT)) {
                return HAL_RADIO_DRIVER_ERROR;
            }

            // Reset the GPIO callback
            if ((halGpioDisableIrqCb(HAL_RADIO_PIN_DIO0)) != HAL_GPIO_SUCCESS) {
                return HAL_RADIO_GPIO_ERROR;
            }

            inst->mode = HAL_RADIO_IDLE;
            return HAL_RADIO_SUCCESS;
        case HAL_RADIO_CB_DO_NOTHING:
            // Do nothing, indicates that the higher layer has called a hal function
            break;
        default:
            // Try to return radio to default mode
            if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_DEFAULT)) {
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

static int32_t managePayloadReady(halRadio_t *inst) {
    bool state = false;
    rfm69_irq2_flag_state(&inst->rfm, RFM69_IRQ2_FLAG_PAYLOAD_READY, &state);

    // Check if any package was received, and that the interrupt was valid
    if (!state) {
        // Try to return to default mode
        if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_DEFAULT)) {
            return HAL_RADIO_DRIVER_ERROR;
        }

        // Reset the radio mode
        inst->mode = HAL_RADIO_IDLE;
        return HAL_RADIO_RECEIVE_FAIL;
    }

    // Clear the buffer used for receiving data
    cBuffer_t *rx_buf = inst->package_callback->pkt_buffer;
    if (cBufferClear(rx_buf) < 0) {
        return HAL_RADIO_BUFFER_ERROR;
    }

    // Try to Read payload size byte from FIFO
    uint8_t num_bytes;
    if (!rfm69_read(&inst->rfm, RFM69_REG_FIFO, &num_bytes, 1)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    // Check if the packet size is valid
    if (num_bytes == 0 || num_bytes > cBufferAvailableForWrite(rx_buf)) {
        return HAL_RADIO_INVALID_SIZE;
    }

    // Get the current write pointer
    uint8_t * raw_rx_buffer = NULL;
    if ((raw_rx_buffer = cBufferGetWritePointer(rx_buf)) == NULL){
        return HAL_RADIO_BUFFER_ERROR;
    }

    // Try to Read remaining data bytes from FIFO
    if (!rfm69_read(&inst->rfm, RFM69_REG_FIFO, raw_rx_buffer, num_bytes)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    // Make sure that the buffer knows how many bytes is in the data array
    if (cBufferEmptyWrite(rx_buf, num_bytes) < 0) {
        return HAL_RADIO_BUFFER_ERROR;
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
            if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_DEFAULT)) {
                return HAL_RADIO_DRIVER_ERROR;
            }

            // Reset the GPIO callback
            if ((halGpioDisableIrqCb(HAL_RADIO_PIN_DIO0)) != HAL_GPIO_SUCCESS) {
                return HAL_RADIO_GPIO_ERROR;
            }

            inst->mode = HAL_RADIO_IDLE;
            return HAL_RADIO_SUCCESS;
        case HAL_RADIO_CB_DO_NOTHING:
            // Do nothing, indicates that the higher layer has called a hal function
            break;
        default:
            // Try to return radio to default mode
            if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_DEFAULT)) {
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

// IRQ from gpio module
static void dio0GpioCallback(halGpioInterface_t *interface, halGpioEvents_t event) {
    halRadio_t * inst = CONTAINER_OF(interface, halRadio_t, gpio_dio0);

    // Store Time Of Arrival
    inst->active_package.time = time_us_64();

    inst->gpio_interrupt = true;
}

int32_t halRadioProcess(halRadio_t *inst) {
    if (!inst->gpio_interrupt) {
        return HAL_GPIO_SUCCESS;
    }

    // Reset the interrupt flag
    inst->gpio_interrupt = false;

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

    // Try to return radio to default mode
    if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_DEFAULT)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

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

    // Configure the dio0 callback
    inst->gpio_dio0.data_cb = dio0GpioCallback;

    // Drive CS pin high
    gpio_set_dir(HAL_RADIO_PIN_CS, GPIO_OUT);
    gpio_put(HAL_RADIO_PIN_CS, 1);

    // Configure the radio
    struct rfm69_config_s config = {
       .spi = HAL_RADIO_SPI_INST,
       .pin_cs = HAL_RADIO_PIN_CS,
       .pin_rst = HAL_RADIO_PIN_RST
    };

    if (!rfm69_init(&inst->rfm, &config)) {
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
            freq_dev      = 250000;
            rfm69_bitrate = RFM69_MODEM_BITRATE_300;
            break;
        default:
            return HAL_RADIO_INVALID_RATE;
    }

    if (!rfm69_bitrate_set(&inst->rfm, rfm69_bitrate)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    // Read and confirm bitrate
    uint16_t bitrate_read = 0;
    if (!rfm69_bitrate_get(&inst->rfm, &bitrate_read)) {
        return HAL_RADIO_DRIVER_ERROR;
    }
    if (bitrate_read != rfm69_bitrate) {
        return HAL_RADIO_CONFIG_ERROR;
    }

    // Set the frequency deviation
    if (!rfm69_fdev_set(&inst->rfm, freq_dev)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    // Read and verify fdev
    uint32_t read_fdev;
    if (!rfm69_fdev_get(&inst->rfm, &read_fdev)) {
        return HAL_RADIO_DRIVER_ERROR;
    }
    if (read_fdev != rfm69_fdev_compute_closest(freq_dev)) {
        return HAL_RADIO_CONFIG_ERROR;
    }

    // Write channel frequency
    inst->config.channel = hal_config.channel;
    if (!rfm69_frequency_set(&inst->rfm, inst->config.channel)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    // Read and verify frequency
    uint32_t read_freq = 0;
    if (!rfm69_frequency_get(&inst->rfm, &read_freq)) {
        return HAL_RADIO_DRIVER_ERROR;
    }
    if (read_freq != rfm69_frequency_compute_closest(inst->config.channel)) {
        return HAL_RADIO_CONFIG_ERROR;
    }

    // RXBW >= fdev + br/2
    if (!rfm69_rxbw_set(&inst->rfm, bw_mantissa, bw_exponent)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    // Read and verify the bandwidth
    uint8_t mantissa = 0;
    uint8_t exponent = 0;
    if (!rfm69_rxbw_get(&inst->rfm, &mantissa, &exponent)) {
        return HAL_RADIO_DRIVER_ERROR;
    }
    if ((mantissa != bw_mantissa) || (exponent != bw_exponent)) {
        return HAL_RADIO_CONFIG_ERROR;
    }

    if (!rfm69_dcfree_set(&inst->rfm, RFM69_DCFREE_WHITENING)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    // Read and verify dcfree setting
    uint8_t reg_read = 0;
    if (!rfm69_dcfree_get(&inst->rfm, &reg_read)) {
        return HAL_RADIO_DRIVER_ERROR;
    }
    if (reg_read != RFM69_DCFREE_WHITENING) {
        return HAL_RADIO_CONFIG_ERROR;
    }

    inst->config.power_dbm = hal_config.power_dbm;
    if (!rfm69_power_level_set(&inst->rfm, inst->config.power_dbm)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    // Store and set our address
    inst->config.rx_address = hal_config.rx_address;
    if (!rfm69_node_address_set(&inst->rfm, inst->config.rx_address)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    // Read and verify the node address
    if (!rfm69_node_address_get(&inst->rfm, &reg_read)) {
        return HAL_RADIO_DRIVER_ERROR;
    }
    if (reg_read != inst->config.rx_address) {
        return HAL_RADIO_CONFIG_ERROR;
    }

    // Store and set the broadcast address
    inst->config.broadcast_address = hal_config.broadcast_address;
    if (!rfm69_broadcast_address_set(&inst->rfm, inst->config.broadcast_address)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    //Read and verify the broadcast address
    if (!rfm69_broadcast_address_get(&inst->rfm, &reg_read)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    if (reg_read != inst->config.broadcast_address) {
        return HAL_RADIO_CONFIG_ERROR;
    }

    // Set packet format
    if (!rfm69_packet_format_set(&inst->rfm, RFM69_PACKET_VARIABLE)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    // Read and verify packet format
    if (!rfm69_packet_format_get(&inst->rfm, &reg_read)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    if (reg_read != RFM69_PACKET_VARIABLE) {
        return HAL_RADIO_CONFIG_ERROR;
    }

    // Set the radio in default mode
    if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_DEFAULT)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    return HAL_RADIO_SUCCESS;
}

int32_t halRadioCancelReceive(halRadio_t *inst) {
    // Put radio back to default
    if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_DEFAULT)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    inst->mode = HAL_RADIO_IDLE;

    return HAL_RADIO_SUCCESS;
}

int32_t halRadioReceivePackageNB(halRadio_t *inst, halRadioInterface_t *interface) {
    if (inst == NULL || interface == NULL || interface->package_cb == NULL || interface->pkt_buffer == NULL) {
        return HAL_RADIO_NULL_ERROR;
    }

    // Check if the radio is busy
    if (inst->mode == HAL_RADIO_TX) {
        return HAL_RADIO_BUSY;
    }

    // Check if the radio is already in the correct state
    if (inst->mode == HAL_RADIO_RX) {
        return HAL_RADIO_SUCCESS;
    }

    // Enable gpio interrupt and set callback
    inst->package_callback = interface;
    if (halGpioEnableIrqCb(&inst->gpio_dio0, HAL_RADIO_PIN_DIO0) != HAL_GPIO_SUCCESS) {
        return HAL_RADIO_GPIO_ERROR;
    }

    // Configure dio0 to create an interrupt on payload ready
    if (!rfm69_dio0_rx_mode_config_set(&inst->rfm, RFM69_DIO0_PKT_RX_PAYLOAD_READY)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    // Try to set RX mode
	if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_RX)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    inst->mode = HAL_RADIO_RX;

    return HAL_RADIO_SUCCESS;
}

int32_t halRadioReceivePackageBlocking(halRadio_t *inst, uint32_t time_ms, uint8_t *data, size_t *size) {
    if (inst == NULL || data == NULL) {
        return HAL_RADIO_NULL_ERROR;
    }

    // Try to set RX mode
	if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_RX)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    bool state = false;
    uint32_t count = 0;
    while (!state && count < time_ms) {
        rfm69_irq2_flag_state(&inst->rfm, RFM69_IRQ2_FLAG_PAYLOAD_READY, &state);
        sleep_ms(PACKET_RX_POLL_TIME_MS);
        count += PACKET_RX_POLL_TIME_MS;
    }

    // Check if any package was received
    if (!state) {
        // Try to return to default mode
        if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_DEFAULT)) {
            return HAL_RADIO_DRIVER_ERROR;
        }

        return HAL_RADIO_RECEIVE_FAIL;
    }

    int32_t result = HAL_RADIO_SUCCESS;

    // Try to Read payload size byte from FIFO
    uint8_t num_bytes;
    if (!rfm69_read(&inst->rfm, RFM69_REG_FIFO, &num_bytes, 1)) {
        result = HAL_RADIO_DRIVER_ERROR;
    }

    // Protect from packets bigger then the buffer
    if (num_bytes > *size) {
        num_bytes = *size;
        result = HAL_RADIO_INVALID_SIZE;
    }

    // Try to Read remaining data bytes from FIFO
    if (!rfm69_read(&inst->rfm, RFM69_REG_FIFO, data, num_bytes)) {
        result = HAL_RADIO_DRIVER_ERROR;
    }

    // Try to return radio to default mode
    if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_DEFAULT)) {
        result = HAL_RADIO_DRIVER_ERROR;
    }

    // Populate parameters
    *size = num_bytes;

    return result;
}

int32_t halRadioCancelTransmit(halRadio_t *inst) {
    // Put radio back to default
    if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_DEFAULT)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    inst->mode = HAL_RADIO_IDLE;

    return HAL_RADIO_SUCCESS;
}

static int32_t writeDataAndEnableTx(halRadio_t *inst, cBuffer_t *pkt_buffer, uint8_t address) {
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
    inst->current_tx_size = pkt_size + HAL_RADIO_PACKET_SIZE_SIZE;
    // When it is an outgoing packet the address is my address
    inst->active_package.address = inst->config.rx_address;

    // Write the package to the radio
    if (!rfm69_write(&inst->rfm, RFM69_REG_FIFO, tx_buffer, inst->current_tx_size)) {
        return HAL_RADIO_DRIVER_ERROR;
    }
	
    if (cBufferClear(inst->package_callback->pkt_buffer) != C_BUFFER_SUCCESS) {
        return HAL_RADIO_BUFFER_ERROR;
    }

    if (inst->mode == HAL_RADIO_TX_IDLE) {
        return HAL_RADIO_SUCCESS;
    }

    // Set the tx power level
    if (!rfm69_power_level_set(&inst->rfm, inst->config.power_dbm)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    // Switch to TX mode to send buffer
    if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_TX)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    return HAL_RADIO_SUCCESS;
}

int32_t halRadioEnterTX(halRadio_t *inst) {
    if (inst == NULL) {
        return HAL_RADIO_NULL_ERROR;
    }

    // Set the tx power level
    if (!rfm69_power_level_set(&inst->rfm, inst->config.power_dbm)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    // Switch to TX mode to send buffer
    if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_TX)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    inst->mode = HAL_RADIO_IDLE;

    return HAL_RADIO_SUCCESS;
}

int32_t halRadioSendPackageNB(halRadio_t *inst, halRadioInterface_t *interface, uint8_t address) {
    if (inst == NULL || interface == NULL || interface->pkg_sent_cb == NULL || interface->pkt_buffer == NULL) {
        return HAL_RADIO_NULL_ERROR;
    }

    // Check if the radio is ready for transmitt
    if (inst->mode == HAL_RADIO_TX) {
        return HAL_RADIO_BUSY;
    }

    // Check package size validity
    int32_t free_space = cBufferAvailableForWrite(interface->pkt_buffer);
    if (free_space < 0 || free_space < HAL_RADIO_PACKET_OVERHEAD) {
        return HAL_RADIO_INVALID_SIZE;
    }

    // Enable gpio callbacks
    inst->package_callback = interface;
    if (halGpioEnableIrqCb(&inst->gpio_dio0, HAL_RADIO_PIN_DIO0) != HAL_GPIO_SUCCESS) {
        return HAL_RADIO_GPIO_ERROR;
    }

    // Configure dio0 to trigger packet sent interrupt
    if (!rfm69_dio0_tx_mode_config_set(&inst->rfm, RFM69_DIO0_PKT_TX_PACKET_SENT)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    int32_t res;
    if ((res = writeDataAndEnableTx(inst, interface->pkt_buffer, address)) != HAL_RADIO_SUCCESS) {
        return res;
    }

    inst->mode = HAL_RADIO_TX;

    return HAL_RADIO_SUCCESS;
}

int32_t halRadioQueueSend(halRadio_t *inst) {
    if (inst == NULL) {
        return HAL_RADIO_NULL_ERROR;
    }

    // Check if the radio is ready for transmitt
    if (inst->mode != HAL_RADIO_TX_QUEUE) {
        // Nothing to send. Which is fine
        return HAL_RADIO_SUCCESS;
    }

	// Switch to TX mode to send FIFO contents
	if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_TX)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    // Radio is in TX state
    inst->mode = HAL_RADIO_TX;

    return HAL_RADIO_SUCCESS;
}

int32_t halRadioQueuePackage(halRadio_t *inst, halRadioInterface_t *interface, uint8_t address) {
    if (inst == NULL || interface == NULL || interface->pkg_sent_cb == NULL) {
        return HAL_RADIO_NULL_ERROR;
    }

    // Check if the radio is ready for transmitt
    if (inst->mode != HAL_RADIO_IDLE) {
        return HAL_RADIO_BUSY;
    }

    // Check package size validity
    int32_t free_space = cBufferAvailableForWrite(interface->pkt_buffer);
    if (free_space < 0 || free_space < HAL_RADIO_PACKET_OVERHEAD) {
        return HAL_RADIO_INVALID_SIZE;
    }

    // Get the current size of the buffer contents
    int32_t result = 0;
    if ((result = cBufferAvailableForRead(interface->pkt_buffer)) <= 0 || result > 253) {
        return HAL_RADIO_BUFFER_ERROR;
    }

    uint8_t pkt_size = (uint8_t)result;

    // Prepend the address of the packet
    if ((result = cBufferPrepend(interface->pkt_buffer, &address, HAL_RADIO_PACKET_ADDR_SIZE)) != HAL_RADIO_PACKET_ADDR_SIZE) {
        return HAL_RADIO_BUFFER_ERROR;
    }

    // Prepend the size of the packet, including the address byte
    pkt_size += HAL_RADIO_PACKET_ADDR_SIZE;
    if ((result = cBufferPrepend(interface->pkt_buffer, &pkt_size, HAL_RADIO_PACKET_SIZE_SIZE)) != HAL_RADIO_PACKET_SIZE_SIZE) {
        return HAL_RADIO_BUFFER_ERROR;
    }

    // Make sure the the contents in the pkt_buffer is in continous memeory
    if ((result = cBufferContiguate(interface->pkt_buffer)) != C_BUFFER_SUCCESS) {
        return HAL_RADIO_BUFFER_ERROR;
    }

    // Get the pointer to the packet to send
    uint8_t *tx_buffer = NULL;
    if ((tx_buffer = cBufferGetReadPointer(interface->pkt_buffer)) == NULL) {
        return HAL_RADIO_BUFFER_ERROR;
    }

    // Set the current packet size
    inst->current_tx_size = pkt_size + HAL_RADIO_PACKET_SIZE_SIZE;

    // Write the package to the radio
    if (!rfm69_write(&inst->rfm, RFM69_REG_FIFO, tx_buffer, inst->current_tx_size)) {
        return HAL_RADIO_DRIVER_ERROR;
    }
	
    if (cBufferClear(inst->package_callback->pkt_buffer) != C_BUFFER_SUCCESS) {
        return HAL_RADIO_BUFFER_ERROR;
    }

    // Configure dio0 to trigger packet sent interrupt
    if (!rfm69_dio0_tx_mode_config_set(&inst->rfm, RFM69_DIO0_PKT_TX_PACKET_SENT)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

	// Enable gpio callbacks
    inst->package_callback = interface;
    if (halGpioEnableIrqCb(&inst->gpio_dio0, HAL_RADIO_PIN_DIO0) != HAL_GPIO_SUCCESS) {
        return HAL_RADIO_GPIO_ERROR;
    }

    // We are now in fifo queued state
    inst->mode = HAL_RADIO_TX_QUEUE;

    return HAL_RADIO_SUCCESS;
}

int32_t halRadioSendPackageBlocking(halRadio_t *inst, cBuffer_t *pkt_buffer, uint8_t address) {
    if (inst == NULL || pkt_buffer == NULL) {
        return HAL_RADIO_NULL_ERROR;
    }

    // Check package size validity
    int32_t free_space = cBufferAvailableForWrite(pkt_buffer);
    if (free_space < 0 || HAL_RADIO_PACKET_OVERHEAD < free_space) {
        return HAL_RADIO_INVALID_SIZE;
    }

    int32_t res;
    if ((res = writeDataAndEnableTx(inst, pkt_buffer, address)) != HAL_RADIO_SUCCESS) {
        return res;
    }

    inst->mode = HAL_RADIO_TX;

	uint8_t buf; // Buffer to hold read result from radio
    // Wait for packet sent flag
    bool state = false;
    uint32_t counter = 0; // Set a maximum timout for sending
    while (!state && counter < 10) {
        rfm69_read(&inst->rfm, RFM69_REG_IRQ_FLAGS_2, &buf, 1);
        LOG("IRQ2: %02X\n", buf);
        rfm69_read(&inst->rfm, RFM69_REG_IRQ_FLAGS_1, &buf, 1);
        LOG("IRQ1: %02X\n", buf);
        rfm69_irq2_flag_state(&inst->rfm, RFM69_IRQ2_FLAG_PACKET_SENT, &state);
        counter++;
    }

    inst->mode = HAL_RADIO_IDLE;

    // Put radio back to default mode
    if (!rfm69_mode_set(&inst->rfm, RFM69_OP_MODE_DEFAULT)) {
        return HAL_RADIO_DRIVER_ERROR;
    }

    LOG("%u bytes sent!\n", free_space);

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

    int32_t time_us = (int32_t)num_bytes*kHalRadioSpiUsPerByte;

    return time_us;
}