#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hal_radio.h"
#include "pico_bootsel_button.h"


/*
 This example can be flashed to a PICO's with a RFM69 radio.
 The radios will be in RX mode waiting for a packets.

 Note that the example uses the broadcast address to enable flashing without changing addresses.
*/

// Radio configuration defines
#define RADIO_MY_ADDR         (0x02) // Change this to target specific radios
#define RADIO_TARGET_ADDR     (0x01) // Change this to target specific radios
#define RADIO_BROADCAST_ADDR  (0xFF)
#define RADIO_DEFAULT_CHANNEL (868)
#define RADIO_RX_BUFFER_SIZE  (128 + C_BUFFER_ARRAY_OVERHEAD)
#define RADIO_TX_BUFFER_SIZE  (128 + C_BUFFER_ARRAY_OVERHEAD) 
#define RADIO_TX_POWER_DBM    (0)

#ifndef LOG
#define LOG(f_, ...) printf((f_), ##__VA_ARGS__)
#endif

static void device_error();

// HalRadio
typedef struct {
    halRadio_t          hal_radio_inst;
    uint8_t             rx_byte_array[RADIO_RX_BUFFER_SIZE];
    cBuffer_t           rx_buffer;

    // LED management
    bool test_led_state;
} myInstance_t;

static myInstance_t my_instance = {0};

// Perform initialisation
int pico_led_init(void) {
    // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
    // so we can use normal GPIO functionality to turn the led on and off
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
}

// Turn the led on or off
void pico_set_led(bool led_on) {
    // Just set the GPIO on or off
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
}

static int32_t halRadioPackageCb(myInstance_t *inst, cBuffer_t *buf) {
    inst->test_led_state = !inst->test_led_state;
    pico_set_led(inst->test_led_state);

    int32_t result = cBufferAvailableForRead(buf);

    if (result < 0) {
        LOG("Invalid packet received %i.\n", result);
        return result;
    }

    // Get the address
    result -= 1; // And remove the address byte from the length
    uint8_t addr = cBufferReadByte(buf);
    LOG("%i bytes received on addr %u.\n", result, addr);

    // Print out payload
    LOG("Payload: ");
    for (int32_t i = 0; i < result; i++) {
        LOG("%c", cBufferReadByte(buf));
    }
    LOG("\n\n");

    // Inform halRadio to remain in RX
    return HAL_RADIO_SUCCESS;
}

int main()
{
    stdio_init_all();
    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);

    // Init the halRadio and set the receiver address
    halRadioConfig_t hal_config = {
        .bitrate           = HAL_RADIO_BITRATE_150,
        .broadcast_address = RADIO_BROADCAST_ADDR,
        .rx_address        = RADIO_MY_ADDR,
        .channel           = RADIO_DEFAULT_CHANNEL,
        .power_dbm         = RADIO_TX_POWER_DBM,
    };

    int32_t res = halRadioInit(&my_instance.hal_radio_inst, hal_config);

    if (res != HAL_RADIO_SUCCESS) {
        return res;
    }

    // Init the RX buffer
    if(cBufferInit(&my_instance.rx_buffer, my_instance.rx_byte_array, RADIO_RX_BUFFER_SIZE) != C_BUFFER_SUCCESS) {
        return 1;
    }

    /* Blocking Receive */
    while (true) {
        res = halRadioReceivePackageBlocking(&my_instance.hal_radio_inst, &my_instance.rx_buffer, 0xFFFFFFFF);
        LOG("Receive Result %i\n", res);

        if (res != HAL_RADIO_SUCCESS) {
            LOG("RADIO PROCESS FAILED! %i\n", res);
            device_error();
        }

        res = halRadioPackageCb(&my_instance, &my_instance.rx_buffer);

        if (res != HAL_RADIO_SUCCESS) {
            LOG("RADIO PROCESS FAILED! %i\n", res);
            device_error();
        }
    }
}

static void device_error() {
    // Forever blink fast
    while (true) {
        pico_set_led(true);
        sleep_ms(100);
        pico_set_led(false);
        sleep_ms(100);
    }
}

