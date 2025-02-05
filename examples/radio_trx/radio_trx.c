#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hal_radio.h"
#include "pico_bootsel_button.h"


/*
 This example can be flashed to two PICO's with a RFM69 radio.
 Both radios will be in RX mode waiting for a packets.
 Send a packet by pressing the pico bootsel button. This switches the radio to TX mode, sends the
 packet and then returns to RX mode.

 The radio is configured in interrupt mode to notify about send complete, and packet available.
 how ever, the callbacks are not in ISR context, they are called through the proccess function.

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

uint8_t msg[] = {'H', 'e', 'l', 'l', 'o', '!'};

// HalRadio
typedef struct {
    halRadio_t          hal_radio_inst;
    halRadioInterface_t hal_interface;
    uint8_t             rx_byte_array[RADIO_RX_BUFFER_SIZE];
    cBuffer_t           rx_buffer;
    uint8_t             tx_byte_array[RADIO_TX_BUFFER_SIZE];
    cBuffer_t           tx_buffer;
    // Button management
    picoBootSelButton_t          boot_button;
    picoBootSelButtonInterface_t btn_interface;
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

void buttonEventCb(picoBootSelButtonInterface_t *interface, picoBootSelButtonEvent_t event) {
    myInstance_t * inst = CONTAINER_OF(interface, myInstance_t, btn_interface);

    int32_t res = cBufferPrepend(&inst->tx_buffer, msg, sizeof(msg));
    if (res != sizeof(msg)) {
        LOG("RADIO SEND FAILED! %i\n", res);
        device_error();
    }

    res = halRadioCancelReceive(&inst->hal_radio_inst);
    if (res != HAL_RADIO_SUCCESS) {
        LOG("RADIO CANCEL RECEIVE FAILED! %i\n", res);
        device_error();
    }

    // Set the interface buffer to the tx_buffer
    inst->hal_interface.pkt_buffer = &inst->tx_buffer;
    res = halRadioSendPackageNB(&inst->hal_radio_inst, &inst->hal_interface, RADIO_BROADCAST_ADDR); // Use TARGET_ADDR to target specific radios

    if (res != HAL_RADIO_SUCCESS) {
        LOG("RADIO SEND FAILED! %i\n", res);
        device_error();
    }
}

static int32_t halRadioPackageCb(halRadioInterface_t *interface, halRadioPackage_t* hal_packet) {
    myInstance_t * inst = CONTAINER_OF(interface, myInstance_t, hal_interface);

    inst->test_led_state = !inst->test_led_state;
    pico_set_led(inst->test_led_state);

    int32_t result = cBufferAvailableForRead(interface->pkt_buffer);

    if (result < 0) {
        LOG("Invalid packet received %i.\n", result);
        return result;
    }

    LOG("%i bytes received.\n", result);

    // Print out payload
    LOG("Payload: ");
    for (int32_t i = 0; i < result; i++) {
        LOG("%c", cBufferReadByte(interface->pkt_buffer));
    }
    LOG("\n\n");

    // Inform halRadio to remain in RX
    return HAL_RADIO_CB_DO_NOTHING;
}

static int32_t halRadioSentCb(halRadioInterface_t *interface, halRadioPackage_t* hal_packet, halRadioErr_t result) {
    myInstance_t * inst = CONTAINER_OF(interface, myInstance_t, hal_interface);

    // Set the radio to RX
    inst->hal_interface.pkt_buffer = &inst->rx_buffer;
    int32_t res = halRadioReceivePackageNB(&inst->hal_radio_inst, &inst->hal_interface);

    if (res != HAL_RADIO_SUCCESS) {
        LOG("RADIO RX FAILED! %i\n", res);
        return res;
    }

    // Inform the halRadio to stay in the state configured here
    return HAL_RADIO_CB_DO_NOTHING;
}

int main()
{
    stdio_init_all();
    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);

    // Prepare bootsel button
    my_instance.btn_interface.event_cb = buttonEventCb;
    int32_t res = picoBootSelButtonInit(&my_instance.boot_button, &my_instance.btn_interface);
    if (res != PICO_BOOTSEL_BTN_SUCCESS) {
        LOG("BUTTON INIT FAILED!\n");
        device_error();
    }


    // Init the halRadio and set the receiver address
    halRadioConfig_t hal_config = {
        .bitrate           = HAL_RADIO_BITRATE_150,
        .broadcast_address = RADIO_BROADCAST_ADDR,
        .rx_address        = RADIO_MY_ADDR,
        .channel           = RADIO_DEFAULT_CHANNEL,
        .power_dbm         = RADIO_TX_POWER_DBM,
    };

    res = halRadioInit(&my_instance.hal_radio_inst, hal_config);

    if (res != HAL_RADIO_SUCCESS) {
        return res;
    }


    // Init the RX buffer
    if(cBufferInit(&my_instance.rx_buffer, my_instance.rx_byte_array, RADIO_RX_BUFFER_SIZE) != C_BUFFER_SUCCESS) {
        return 1;
    }

    // Init the TX buffer
    if(cBufferInit(&my_instance.tx_buffer, my_instance.tx_byte_array, RADIO_RX_BUFFER_SIZE) != C_BUFFER_SUCCESS) {
        return 1;
    }

    my_instance.hal_interface.package_cb  = halRadioPackageCb;
    my_instance.hal_interface.pkg_sent_cb = halRadioSentCb;
    my_instance.hal_interface.pkt_buffer = &my_instance.rx_buffer;

    // Set the radio to RX
    my_instance.hal_interface.pkt_buffer = &my_instance.rx_buffer;
    res = halRadioReceivePackageNB(&my_instance.hal_radio_inst, &my_instance.hal_interface);

    if (res != HAL_RADIO_SUCCESS) {
        LOG("RADIO RX FAILED! %i\n", res);
        device_error();
    }

    while (true) {
        res = halRadioProcess(&my_instance.hal_radio_inst);
        if (res != HAL_RADIO_SUCCESS) {
            LOG("RADIO PROCESS FAILED! %i\n", res);
            device_error();
        }

        // Process the button
        res = picoBootSelButtonProcess(&my_instance.boot_button);
        if (res != PICO_BOOTSEL_BTN_SUCCESS) {
            LOG("BUTTON PROCESS FAILED!\n");
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

