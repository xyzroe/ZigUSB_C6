
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "i2cdev.h"
#include "ina219.h"
#include <time.h>
#include <sys/time.h>

#include "ha/esp_zigbee_ha_standard.h"
#include "esp_timer.h"
#include "esp_ota_ops.h"
#include "zboss_api.h"
#include "zcl/esp_zigbee_zcl_command.h"
#include "zcl/zb_zcl_common.h"

#include "main.h"
#include "const.h"
#include "tools.h"
#include "perf.h"
#include "zigbee.h"

/*------ Global definitions -----------*/
float led_hz = 0;
char strftime_buf[64];
DataStructure data;

uint16_t manuf_id = OTA_UPGRADE_MANUFACTURER;
char manufacturer[16];
char model[16];
char firmware_version[16];
char firmware_date[16];
bool time_updated = false;
bool connected = false;

uint16_t power = 0;
uint16_t voltage = 0;
uint16_t current = 0;
uint16_t CPU_temp = 0;
uint16_t uptime = 0;
float ina_bus_voltage = 0;
float ina_shunt_voltage = 0;
float ina_current = 0;
float ina_power = 0;

TaskHandle_t ledTaskHandle = NULL;

void app_main(void)
{

    ESP_LOGW(__func__, "FW verison 0x%x (%d) date: %s", OTA_FW_VERSION, OTA_FW_VERSION, FW_BUILD_DATE);

    setup_NVS();

    print_chip_info();

    register_button(BTN_GPIO_1); /* Button from v0.3 */
    register_button(BTN_GPIO_2); /* Button till v0.2 */
    register_alarm_input();

    data.int_led_mode = read_NVS("int_led_mode");
    data.ext_led_mode = read_NVS("ext_led_mode");
    data.USB_state = read_NVS("USB_state");
    data.start_up_on_off = read_NVS("start_up_on_off");

    init_outputs();

    ESP_ERROR_CHECK(i2cdev_init());

    zigbee_setup();

    xTaskCreate(led_task, "led_task", 4096, NULL, 5, &ledTaskHandle);
    xTaskCreate(force_update, "force_update_task", 4096, NULL, 4, NULL);
    xTaskCreate(int_temp_task, "int_temp_task", 4096, NULL, 3, NULL);
    xTaskCreate(ina219_task, "ina219_task", 4096, NULL, 2, NULL);

    // xTaskCreate(test_task, "test_task", 4096, NULL, 2, NULL);
    xTaskCreate(debug_task, "debug_task", 4096, NULL, 3, NULL);
}
