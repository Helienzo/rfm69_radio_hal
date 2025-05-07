#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hal_radio.h"
#include "pico_bootsel_button.h"


/*
 This example can be flashed to two PICO's with a RFM69 radio.
 Send a packet by pressing the pico bootsel button. This switches the radio to TX mode, sends the
 packet and then returns to idle mode.

 Note that the example uses the broadcast address to enable flashing without changing addresses.
*/

// Radio configuration defines
#define RADIO_MY_ADDR         (0x02) // Change this to target specific radios
#define RADIO_TARGET_ADDR     (0x01) // Change this to target specific radios
#define RADIO_BROADCAST_ADDR  (0xFF)
#define RADIO_DEFAULT_CHANNEL (868000000)
#define RADIO_RX_BUFFER_SIZE  (128 + C_BUFFER_ARRAY_OVERHEAD)
#define RADIO_TX_BUFFER_SIZE  (128 + C_BUFFER_ARRAY_OVERHEAD) 
#define RADIO_TX_POWER_DBM    (0)

#ifndef LOG
#define LOG(f_, ...) printf((f_), ##__VA_ARGS__)
#endif

static void device_error();

/* Small package that can be sent on a single fifo write */
/*
uint8_t msg[] = {'H','e', 'l', 'l', 'o',' ', 'W', 'o', 'r', 'l', 'd', '!', ' ', 'H','e', 'l', 'l', 'o',' ', 'W', 'o', 'r', 'l', 'd', '!'};
*/

/* Large packet that requires on the fly fifo fill */
static uint8_t msg[] = {
    '1', ',', ' ', '2', ',', ' ', '3', ',', ' ', '4', ',', ' ', '5', ',', ' ', 
    '6', ',', ' ', '7', ',', ' ', '8', ',', ' ', '9', ',', ' ', '1', '0', ',', ' ', 
    '1', '1', ',', ' ', '1', '2', ',', ' ', '1', '3', ',', ' ', '1', '4', ',', ' ', 
    '1', '5', ',', ' ', '1', '6', ',', ' ', '1', '7', ',', ' ', '1', '8', ',', ' ', 
    '1', '9', ',', ' ', '2', '0', ',', ' ', '2', '1', ',', ' ', '2', '2', ',', ' ', 
    '2', '3', ',', ' ', '2', '4', ',', ' ', '2', '5', ',', ' ', '2', '6', ',', ' ', 
    '2', '7', ',', ' ', '2', '8', ',', ' ', '2', '9', ',', ' ', '3', '0'};

// HalRadio
typedef struct {
    halRadio_t          hal_radio_inst;
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

    /* Blocking Write */
    res = halRadioSendPackageBlocking(&inst->hal_radio_inst, &inst->tx_buffer, RADIO_BROADCAST_ADDR);
    if (res != HAL_RADIO_SUCCESS) {
        LOG("RADIO SEND FAILED! %i\n", res);
        device_error();
    }
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

    // Init the TX buffer
    if(cBufferInit(&my_instance.tx_buffer, my_instance.tx_byte_array, RADIO_TX_BUFFER_SIZE) != C_BUFFER_SUCCESS) {
        return 1;
    }

    while (true) {
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

