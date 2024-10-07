#ifndef ZIGUSB_H
#define ZIGUSB_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "esp_zigbee_core.h"

#define BTN_GPIO_1 5
#define BTN_GPIO_2 9
#define ALARM_GPIO 10
#define EXT_LED_GPIO 7
#define INT_LED_GPIO 8
#define USB_GPIO 3
#define I2C_SDA_GPIO 0
#define I2C_SCL_GPIO 1
#define I2C_PORT 0
#define I2C_ADDR INA219_ADDR_GND_GND

#define LED_ON_STATE 0
#define LED_OFF_STATE 1

#define SHUNT_RESISTOR_MILLI_OHM 100

#define INA219_INTERVAL 1000
#define CPU_TEMP_INTERVAL 1000
#define IDENTITY_INTERVAL 10000
#define LONG_PRESS_TIME 5000
#define SHORT_PRESS_TIME 150
#define UPDATE_ATTRIBUTE_INTERVAL 60000

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

/* Zigbee configuration */
#define MAX_CHILDREN 10                 /* the max amount of connected devices */
#define INSTALLCODE_POLICY_ENABLE false /* enable the install code policy for security */
#define SENSOR_ENDPOINT 1
#define INT_LED_ENDPOINT 2
#define EXT_LED_ENDPOINT 3
#define ESP_ZB_PRIMARY_CHANNEL_MASK ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK /* Zigbee primary channel mask use in the example */

#define OTA_UPGRADE_MANUFACTURER 0x1111 /* The attribute indicates the value for the manufacturer of the device */
#define OTA_UPGRADE_IMAGE_TYPE 0x1011   /* The attribute indicates the type of the image */
#define OTA_UPGRADE_HW_VERSION 0x0101   /* The parameter indicates the version of hardware */
#define OTA_UPGRADE_MAX_DATA_SIZE 64    /* The parameter indicates the maximum data size of query block image */

#define HW_MANUFACTURER "xyzroe" /* The parameter indicates the manufacturer of the device */
#define HW_MODEL "ZigUSB_C6"     /* The parameter indicates the model of the device */

#define OTA_FW_VERSION 0x0000025F /* The attribute indicates the version of the firmware */
#define FW_BUILD_DATE "20241008"  /* The parameter indicates the build date of the firmware */

#define ESP_ZB_ZR_CONFIG()                                \
    {                                                     \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,         \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE, \
        .nwk_cfg.zczr_cfg = {                             \
            .max_children = MAX_CHILDREN,                 \
        },                                                \
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

extern char manufacturer[16];
extern char model[16];
extern char firmware_version[16];
extern char firmware_date[16];
extern bool time_updated;
extern bool connected;
extern bool usb_pwr_state;

extern uint16_t power;
extern uint16_t voltage;
extern uint16_t current;
extern uint16_t CPU_temp;
extern uint16_t uptime;
extern float ina_bus_voltage;
extern float ina_shunt_voltage;
extern float ina_current;
extern float ina_power;

void usb_driver_set_power(bool state);
void send_bin_cfg_option(int endpoint, bool value);

void send_alarm_state(bool alarm);
void setup_NVS();
void init_pullup_i2c_pins();
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask);
void send_update_attribute_and_ias();
void update_attribute();
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
