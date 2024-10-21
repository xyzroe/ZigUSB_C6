#ifndef ZIGUSB_H
#define ZIGUSB_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "esp_zigbee_core.h"

void int_led_blink();
void ext_led_action(int mode);

typedef struct
{
    bool USB_state;
    bool ext_led_mode;
    bool int_led_mode;
    bool alarm_state;
    int start_up_on_off;
} DataStructure;

// #define TEST_MODE

extern float led_hz;
extern char strftime_buf[64];
extern DataStructure data;

extern uint16_t manuf_id;
extern char manufacturer[16];
extern char model[16];
extern char firmware_version[16];
extern char firmware_date[16];
extern bool time_updated;
extern bool connected;

extern uint16_t power;
extern uint16_t voltage;
extern uint16_t current;
extern uint16_t CPU_temp;
extern uint16_t uptime;
extern float ina_bus_voltage;
extern float ina_shunt_voltage;
extern float ina_current;
extern float ina_power;

extern TaskHandle_t ledTaskHandle;

typedef enum
{
    ATTRIBUTE_ALL,
    ATTRIBUTE_TEMP,
    ATTRIBUTE_ELECTRO
} attribute_t;

void app_main(void);

#endif // ZIGUSB_H
