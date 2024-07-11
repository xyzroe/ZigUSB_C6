#include "esp_zigbee_core.h"

#define BTN_GPIO 5
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
#define UPDATE_ATTRIBUTE_INTERVAL 10000

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

#define OTA_UPGRADE_MANUFACTURER 0x1001 /* The attribute indicates the value for the manufacturer of the device */
#define OTA_UPGRADE_IMAGE_TYPE 0x1011   /* The attribute indicates the type of the image */
#define OTA_UPGRADE_HW_VERSION 0x0101   /* The parameter indicates the version of hardware */
#define OTA_UPGRADE_MAX_DATA_SIZE 64    /* The parameter indicates the maximum data size of query block image */

#define HW_MANUFACTURER "xyzroe" /* The parameter indicates the manufacturer of the device */
#define HW_MODEL "ZigUSB_C6"     /* The parameter indicates the model of the device */

#define OTA_FW_VERSION 0x00000262 /* The attribute indicates the version of the firmware */
#define FW_BUILD_DATE "20240711"  /* The parameter indicates the build date of the firmware */

#define ESP_ZB_ZR_CONFIG()                                \
    {                                                     \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,         \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE, \
        .nwk_cfg.zczr_cfg = {                             \
            .max_children = MAX_CHILDREN,                 \
        },                                                \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()    \
    {                                    \
        .radio_mode = ZB_RADIO_MODE_NATIVE, \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                       \
    {                                                      \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE, \
    }
