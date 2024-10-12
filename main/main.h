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

void setup_NVS();
bool int_to_bool(int32_t value);
int read_NVS(const char *nvs_key);
bool write_NVS(const char *nvs_key, int value);

// #define TEST_MODE

#define ESP_ZB_ZR_CONFIG()                        \
    {                                             \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER, \
        .install_code_policy = false,             \
        .nwk_cfg.zczr_cfg = {                     \
            .max_children = 10,                   \
        },                                        \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()       \
    {                                       \
        .radio_mode = ZB_RADIO_MODE_NATIVE, \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                          \
    {                                                         \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE, \
    }

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

typedef enum
{
    ATTRIBUTE_ALL,
    ATTRIBUTE_TEMP,
    ATTRIBUTE_ELECTRO
} attribute_t;

void update_attributes(attribute_t attribute);

void send_bin_cfg_option(int endpoint, bool value);

void send_alarm_state(bool alarm);
void setup_NVS();

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask);
void update_attributes();
void force_update();
static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message);
static esp_err_t zb_read_attr_resp_handler(const esp_zb_zcl_cmd_read_attr_resp_message_t *message);
static esp_err_t zb_ota_upgrade_status_handler(esp_zb_zcl_ota_upgrade_value_message_t message);
static esp_err_t zb_ota_upgrade_query_image_resp_handler(esp_zb_zcl_ota_upgrade_query_image_resp_message_t message);
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message);
void read_server_time();
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct);
static void set_zcl_string(char *buffer, char *value);
static void esp_zb_task(void *pvParameters);
void app_main(void);

#endif // ZIGUSB_H
