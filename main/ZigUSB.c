#include "ZigUSB.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "ina219.h"
#include "iot_button.h"
#include <time.h>
#include <sys/time.h>
#include "driver/temperature_sensor.h"
#include "ha/esp_zigbee_ha_standard.h"

/*------ Clobal definitions -----------*/
static char manufacturer[16], model[16], firmware_version[16], firmware_date[16];
bool time_updated = false, connected = false;

uint16_t power = 0, voltage = 0, current = 0, CPU_temp = 0, uptime = 0;           // ZB values
float ina_bus_voltage = 0, ina_shunt_voltage = 0, ina_current = 0, ina_power = 0; // INA219 values

float led_hz = 0; // LED blinks every 1000/led_hz millis
char strftime_buf[64];

static const char *TAG = HW_MODEL;

static const char *ZIG_TAG = "ZB";

static bool usb_pwr_state = false;

DataStructure data;

#if !defined CONFIG_ZB_ZCZR
#error Define ZB_ZCZR in idf.py menuconfig to compile light (Router) source code.
#endif

TaskHandle_t ledTaskHandle = NULL;

/*static switch_func_pair_t button_func_pair[] = {
    {GPIO_INPUT_IO_TOGGLE_SWITCH, SWITCH_ONOFF_TOGGLE_CONTROL}
};

static void esp_zb_buttons_handler(switch_func_pair_t *button_func_pair)
{
    if (button_func_pair->func == SWITCH_ONOFF_TOGGLE_CONTROL) {
        // implemented light switch toggle functionality
        esp_zb_zcl_on_off_cmd_t cmd_req;
        cmd_req.zcl_basic_cmd.src_endpoint = HA_ONOFF_SWITCH_ENDPOINT;
        cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
        cmd_req.on_off_cmd_id = ESP_ZB_ZCL_CMD_ON_OFF_TOGGLE_ID;
        esp_zb_lock_acquire(portMAX_DELAY);
        esp_zb_zcl_on_off_cmd_req(&cmd_req);
        esp_zb_lock_release();
        ESP_EARLY_LOGI(TAG, "Send 'on_off toggle' command");
    }
}
*/

void usb_driver_set_power(bool state)
{
    gpio_set_level(USB_GPIO, state);
    ext_led_action(state);
    ESP_LOGI("usb_driver_set_power", "Setting USB power to %d", state);
    usb_pwr_state = state;
    data.USB_state = state;
    if (data.start_up_on_off > 1)
    {
        write_NVS("USB_state", state);
    }
}

void send_bin_cfg_option(int endpoint, bool value)
{
    esp_zb_zcl_status_t state_tmp = esp_zb_zcl_set_attribute_val(endpoint,
                                                                 ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
                                                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                                                 ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                                                                 &value,
                                                                 false);

    if (state_tmp != ESP_ZB_ZCL_STATUS_SUCCESS)
    {
        ESP_LOGE(TAG, "Setting cfg option On/Off attribute failed! error %d", state_tmp);
    }
}

static void button_single_click_cb(void *arg, void *usr_data)
{
    ESP_LOGI("Button boot", "Single click");

    bool new_state = !(usb_pwr_state);
    usb_driver_set_power(new_state);

    /*  esp_zb_zcl_status_t state_tmp = esp_zb_zcl_set_attribute_val(1,
                                                                   ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
                                                                   ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                                                   ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                                                                   &new_state,
                                                                   false);

      if (state_tmp != ESP_ZB_ZCL_STATUS_SUCCESS)
      {
          ESP_LOGE(TAG, "Setting On/Off attribute failed! error %d", state_tmp);
      }*/
    send_bin_cfg_option(1, new_state);
}

static void button_long_press_cb(void *arg, void *usr_data)
{
    ESP_LOGI("Button boot", "Long press, leave & reset");
    data.ext_led_mode = 1; // Force turn LED ON
    for (int i = 0; i < 5; i++)
    {
        ext_led_action(3);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    esp_zb_factory_reset();
}

void register_button()
{
    // create gpio button
    button_config_t gpio_btn_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = LONG_PRESS_TIME,
        .short_press_time = SHORT_PRESS_TIME,
        .gpio_button_config = {
            .gpio_num = BTN_GPIO,
            .active_level = 0,
        },
    };

    button_handle_t gpio_btn = iot_button_create(&gpio_btn_cfg);
    if (NULL == gpio_btn)
    {
        ESP_LOGE("Button boot", "Button create failed");
    }

    iot_button_register_cb(gpio_btn, BUTTON_SINGLE_CLICK, button_single_click_cb, NULL);
    iot_button_register_cb(gpio_btn, BUTTON_LONG_PRESS_START, button_long_press_cb, NULL);
}

static void send_alarm_state(bool alarm)
{

    uint16_t new_zone_status = 0x0001;
    if (alarm == 0)
    {
        new_zone_status = 0x0000;
    }
    if (connected)
    {
        /*
        // Set zone_status to a different value in another part of the application
        // esp_zb_ias_zone_cluster_add_attr(SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_IAS_ZONE_ZONE_STATUS_ALARM1, &new_zone_status);
        esp_zb_zcl_status_t state_zone = esp_zb_zcl_set_attribute_val(SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_IAS_ZONE, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_IAS_ZONE_ZONE_STATUS_ALARM1, &new_zone_status, false);
        // Check for error
        if (state_zone != ESP_ZB_ZCL_STATUS_SUCCESS)
        {
            ESP_LOGE(TAG, "Setting zone attribute failed!");
        }
        */
        esp_zb_zcl_ias_zone_status_change_notif_cmd_t cmd = {
            .zcl_basic_cmd = {
                .dst_addr_u.addr_short = 0x0000,
                .dst_endpoint = 1,
                .src_endpoint = 1,
            },
            .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
            .zone_status = new_zone_status,
            .zone_id = 0,
            .delay = 0,
        };
        esp_zb_zcl_status_t state_tmp = esp_zb_zcl_ias_zone_status_change_notif_cmd_req(&cmd);

        /*if (state_tmp != ESP_ZB_ZCL_STATUS_SUCCESS)
        {
            ESP_LOGE(TAG, "ias_zone_status_change_notif_cmd failed! error %d", state_tmp);
        }*/
    }
    else
    {
        ESP_LOGE(TAG, "Alarm! but not connected to coordinator");
    }
}

static void alarm_input_active_cb(void *arg, void *usr_data)
{
    ESP_LOGE("Alarm input", "active");
    data.alarm_state = 1;
    send_alarm_state(data.alarm_state);
}

static void alarm_input_deactive_cb(void *arg, void *usr_data)
{
    ESP_LOGE("Alarm input", "deactive");
    data.alarm_state = 0;
    send_alarm_state(data.alarm_state);
}

void register_alarm_input()
{
    button_config_t gpio_alarm_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config = {
            .gpio_num = ALARM_GPIO,
            .active_level = 0,
        },
    };

    button_handle_t gpio_alarm = iot_button_create(&gpio_alarm_cfg);
    if (NULL == gpio_alarm)
    {
        ESP_LOGE("Alarm boot", "Alarm input create failed");
    }

    iot_button_register_cb(gpio_alarm, BUTTON_PRESS_UP, alarm_input_deactive_cb, NULL);
    iot_button_register_cb(gpio_alarm, BUTTON_PRESS_DOWN, alarm_input_active_cb, NULL);
}

void setup_NVS()
{
    static const char *local_tag = "setup_NVS";
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    nvs_handle_t my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    ESP_LOGI(local_tag, "Opening Non-Volatile Storage (NVS) handle... %s", (err != ESP_OK) ? "Failed!" : "Done");
    if (err == ESP_OK)
    {
   
        // Read
        int32_t restart_counter = 0; // value will default to 0, if not set yet in NVS
        err = nvs_get_i32(my_handle, "restart_counter", &restart_counter);
        ESP_LOGI(local_tag, "Reading restart counter from NVS ... %s", (err != ESP_OK) ? "Failed!" : "Done");
        switch (err)
        {
        case ESP_OK:
            ESP_LOGI(local_tag, "Restart counter = %" PRIu32, restart_counter);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGI(local_tag, "The value is not initialized yet!");
            break;
        default:
            ESP_LOGI(local_tag, "Error (%s) reading!", esp_err_to_name(err));
        }

        // Write
        restart_counter++;
        err = nvs_set_i32(my_handle, "restart_counter", restart_counter);
        ESP_LOGI(local_tag, "Updating restart counter in NVS ... %s", (err != ESP_OK) ? "Failed!" : "Done");

        // Commit written value.
        // After setting any values, nvs_commit() must be called to ensure changes are written
        // to flash storage. Implementations may write to storage at other times,
        // but this is not guaranteed.
        err = nvs_commit(my_handle);
        ESP_LOGI(local_tag, "Committing updates in NVS ... %s", (err != ESP_OK) ? "Failed!" : "Done");

        // Close
        nvs_close(my_handle);
    }
}

bool int_to_bool(int32_t value)
{
    return (value != 0);
}

int read_NVS(const char *nvs_key)
{
    static const char *local_tag = "read_NVS";
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);

    // Read
    // ESP_LOGI(TAG, "Reading restart counter from NVS ... ");
    // int32_t restart_counter = 0; // value will default to 0, if not set yet in NVS
    int32_t value = 0;
    err = nvs_get_i32(my_handle, nvs_key, &value);
    switch (err)
    {
    case ESP_OK:
        ESP_LOGI(local_tag, "%s is %ld ", nvs_key, value);
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        ESP_LOGE(local_tag, "The value is not initialized yet!");
        int value = 0;

        char *substring = "_led_mode";
        if (strstr(nvs_key, substring) != NULL)
        {
            value = 1;
        }
        err = nvs_set_i32(my_handle, nvs_key, value);
        ESP_LOGW(local_tag, "Updating %s in NVS ... %s", nvs_key, (err != ESP_OK) ? "Failed!" : "Done");
        break;
    default:
        ESP_LOGE(local_tag, "Error (%s) reading!", esp_err_to_name(err));
    }
    // Close
    nvs_close(my_handle);
    if (err != ESP_OK)
    {
        return false;
    }
    return value;
}

bool write_NVS(const char *nvs_key, int value)
{
    static const char *local_tag = "write_NVS";
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    err = nvs_set_i32(my_handle, nvs_key, value);
    ESP_LOGI(local_tag, "Write value... %s", (err != ESP_OK) ? "Failed!" : "Done");

    // Commit written value.
    // After setting any values, nvs_commit() must be called to ensure changes are written
    // to flash storage. Implementations may write to storage at other times,
    // but this is not guaranteed.
    err = nvs_commit(my_handle);
    ESP_LOGI(local_tag, "Commit updates... %s", (err != ESP_OK) ? "Failed!" : "Done");

    // Close
    nvs_close(my_handle);

    if (err != ESP_OK)
    {
        return false;
    }
    return true;
}

void init_outputs()
{
    ESP_LOGI("init_outputs", "setup LEDs gpios");
    gpio_reset_pin(EXT_LED_GPIO);
    gpio_set_direction(EXT_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_pulldown_en(EXT_LED_GPIO);
    gpio_set_level(EXT_LED_GPIO, LED_OFF_STATE);

    gpio_reset_pin(INT_LED_GPIO);
    gpio_set_direction(INT_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_pulldown_en(INT_LED_GPIO);
    gpio_set_level(INT_LED_GPIO, LED_OFF_STATE);

    ESP_LOGI("init_outputs", "setup USB gpio");
    gpio_reset_pin(USB_GPIO);
    gpio_set_direction(USB_GPIO, GPIO_MODE_OUTPUT);
    gpio_pullup_en(USB_GPIO);
    switch (data.start_up_on_off)
    {
    case 0:
        usb_driver_set_power(0);
        break;
    case 1:
        usb_driver_set_power(1);
        break;
    case 2:
        usb_driver_set_power(!data.USB_state);
        break;
    case 255:
        usb_driver_set_power(data.USB_state);
        break;
    default:
        break;
    }
}

void init_pullup_i2c_pins()
{
    ESP_LOGI("init_pullup_i2c_pins", "setup I2C_SDA_GPIO");
    gpio_reset_pin(I2C_SDA_GPIO);
    gpio_pullup_en(I2C_SDA_GPIO);
    ESP_LOGI("init_pullup_i2c_pins", "setup I2C_SCL_GPIO");
    gpio_reset_pin(I2C_SCL_GPIO);
    gpio_pullup_en(I2C_SCL_GPIO);
}

/* --------- User task section -----------------*/
static void get_rtc_time()
{
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%a %H:%M:%S", &timeinfo);
}

static void ina219_task(void *pvParameters)
{
    ina219_t dev;
    memset(&dev, 0, sizeof(ina219_t));

    assert(SHUNT_RESISTOR_MILLI_OHM > 0);
    ESP_ERROR_CHECK(ina219_init_desc(&dev, I2C_ADDR, I2C_PORT, I2C_SDA_GPIO, I2C_SCL_GPIO));
    ESP_LOGI(TAG, "Initializing INA219");
    ESP_ERROR_CHECK(ina219_init(&dev));

    ESP_LOGI(TAG, "Configuring INA219");
    ESP_ERROR_CHECK(ina219_configure(&dev, INA219_BUS_RANGE_16V, INA219_GAIN_0_125,
                                     INA219_RES_12BIT_1S, INA219_RES_12BIT_1S, INA219_MODE_CONT_SHUNT_BUS));

    ESP_LOGI(TAG, "Calibrating INA219");

    ESP_ERROR_CHECK(ina219_calibrate(&dev, (float)SHUNT_RESISTOR_MILLI_OHM / 1000.0f));

    ESP_LOGI(TAG, "Starting the loop ina219_task");
    while (1)
    {
        ESP_ERROR_CHECK(ina219_get_bus_voltage(&dev, &ina_bus_voltage));
        ESP_ERROR_CHECK(ina219_get_shunt_voltage(&dev, &ina_shunt_voltage));
        ESP_ERROR_CHECK(ina219_get_current(&dev, &ina_current));
        ESP_ERROR_CHECK(ina219_get_power(&dev, &ina_power));
        /* Using float in printf() requires non-default configuration in
         * sdkconfig. See sdkconfig.defaults.esp32 and
         * sdkconfig.defaults.esp8266  */
        // printf("VBUS: %.04f V, VSHUNT: %.04f mV, IBUS: %.04f mA, PBUS: %.04f mW\n",
        //      ina_bus_voltage, ina_shunt_voltage * 1000, ina_current * 1000, ina_power * 1000);

        // ZCL must be V,A,W = uint16. But we need more accuaracy - so use *100.

        voltage = (float)(ina_bus_voltage * 100);
        current = (float)(ina_current * 100);
        power = (float)(ina_power * 100);
        vTaskDelay(pdMS_TO_TICKS(INA219_INTERVAL));
    }
}

static void CPUtemp_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Initializing temperature sensor");
    temperature_sensor_handle_t temp_handle = NULL;
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 60);
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_handle));

    ESP_LOGI(TAG, "Enable temperature sensor");
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_handle));

    ESP_LOGI(TAG, "Starting the loop CPUtemp_task");
    while (1)
    {
        // Enable temperature sensor
        // ESP_ERROR_CHECK(temperature_sensor_enable(temp_handle));

        // Get converted sensor data
        float tsens_out;
        ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_handle, &tsens_out));
        // printf("Temperature in %f °C\n", tsens_out);
        CPU_temp = (float)(tsens_out * 100);

        // Disable the temperature sensor if it is not needed and save the power
        // ESP_ERROR_CHECK(temperature_sensor_disable(temp_handle));
        vTaskDelay(pdMS_TO_TICKS(CPU_TEMP_INTERVAL));

#ifdef TEST_MODE
        get_rtc_time();
        ESP_LOGI(TAG, "time %s", strftime_buf);
#endif
    }
}

void int_led_blink()
{
    if (data.int_led_mode)
    {
        gpio_set_level(INT_LED_GPIO, LED_ON_STATE);
        vTaskDelay(pdMS_TO_TICKS(150));
        gpio_set_level(INT_LED_GPIO, LED_OFF_STATE);
    }
}

void ext_led_action(int mode)
{
    // ESP_LOGW(TAG, "ext_led_action: mode %d, ex_led_m: %d", mode, data.ext_led_mode);
    if (data.ext_led_mode)
    {
        if (mode == 1)
        {
            // ESP_LOGW(TAG, "ext_led_action: LED ON");
            gpio_set_level(EXT_LED_GPIO, 0);
        }
        else if (mode == 0)
        {
            // ESP_LOGW(TAG, "ext_led_action: LED OFF");
            gpio_set_level(EXT_LED_GPIO, 1);
        }
        else if (mode == 1)
        {
            // ESP_LOGW(TAG, "ext_led_action: LED invert");
            int level = gpio_get_level(EXT_LED_GPIO);
            gpio_set_level(EXT_LED_GPIO, !level);
        }
        else if (mode == 2)
        {
            // ESP_LOGW(TAG, "ext_led_action: LED blink");
            int level = gpio_get_level(EXT_LED_GPIO);
            gpio_set_level(EXT_LED_GPIO, !level);
            vTaskDelay(pdMS_TO_TICKS(250));
            gpio_set_level(EXT_LED_GPIO, level);
        }
    }
}

static void led_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting LED");
    // led_hz = 1;
    while (1)
    {
        if (led_hz > 0)
        {
            int_led_blink();
            vTaskDelay(pdMS_TO_TICKS(1000 / led_hz));
        }
        else
        {
            // LED disabled

            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}
/*----------------------------------------*/

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

/* Manual reporting atribute to coordinator */
static void reportAttribute(uint8_t endpoint, uint16_t clusterID, uint16_t attributeID, void *value, uint8_t value_length)
{
    esp_zb_zcl_report_attr_cmd_t cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = 0x0000,
            .dst_endpoint = endpoint,
            .src_endpoint = endpoint,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = clusterID,
        .attributeID = attributeID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    };
    esp_zb_zcl_attr_t *value_r = esp_zb_zcl_get_attribute(endpoint, clusterID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, attributeID);
    memcpy(value_r->data_p, value, value_length);
    esp_zb_zcl_report_attr_cmd_req(&cmd);
}

/* Task for update attribute value */
void update_attribute()
{
    static const char *local_tag = "update_attribute";
    while (1)
    {
        if (connected)
        {
            /* Send current alarm state value */
            send_alarm_state(data.alarm_state);

            /* Write new temperature value */
            esp_zb_zcl_status_t state_tmp = esp_zb_zcl_set_attribute_val(SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &CPU_temp, false);
            /* Check for error */
            if (state_tmp != ESP_ZB_ZCL_STATUS_SUCCESS)
            {
                ESP_LOGE(local_tag, "Setting temperature attribute failed!");
            }

            // DC clusters
            /* Write new current value */
            esp_zb_zcl_status_t state_current = esp_zb_zcl_set_attribute_val(SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_ID, &current, false);
            /* Check for error */
            if (state_current != ESP_ZB_ZCL_STATUS_SUCCESS)
            {
                ESP_LOGE(local_tag, "Setting current attribute failed!");
            }

            /* Write new voltage value */
            esp_zb_zcl_status_t state_voltage = esp_zb_zcl_set_attribute_val(SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_ID, &voltage, false);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            /* Check for error */
            if (state_voltage != ESP_ZB_ZCL_STATUS_SUCCESS)
            {
                ESP_LOGE(local_tag, "Setting voltage attribute failed!");
            }

            /* Write new power value */
            esp_zb_zcl_status_t state_power = esp_zb_zcl_set_attribute_val(SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DCPOWER_ID, &power, false);
            /* Check for error */
            if (state_power != ESP_ZB_ZCL_STATUS_SUCCESS)
            {
                ESP_LOGE(local_tag, "Setting power attribute failed!");
            }

            // AC clusters z2m
            // Write new current value
            state_current = esp_zb_zcl_set_attribute_val(SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMSCURRENT_ID, &current, false);
            // Check for error
            if (state_current != ESP_ZB_ZCL_STATUS_SUCCESS)
            {
                ESP_LOGE(local_tag, "Setting current attribute failed!");
            }

            // Write new voltage value
            state_voltage = esp_zb_zcl_set_attribute_val(SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMSVOLTAGE_ID, &voltage, false);
            // Check for error
            if (state_voltage != ESP_ZB_ZCL_STATUS_SUCCESS)
            {
                ESP_LOGE(local_tag, "Setting voltage attribute failed!");
            }

            // Write new power value
            state_power = esp_zb_zcl_set_attribute_val(SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACTIVE_POWER_ID, &power, false);
            // Check for error
            if (state_power != ESP_ZB_ZCL_STATUS_SUCCESS)
            {
                ESP_LOGE(local_tag, "Setting power attribute failed!");
            }

            int_led_blink();
            ESP_LOGI(local_tag, "temperature = %d, current = %d, voltage = %d, power = %d", CPU_temp, current, voltage, power);
        }

        vTaskDelay(UPDATE_ATTRIBUTE_INTERVAL / portTICK_PERIOD_MS);
    }
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, ZIG_TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, ZIG_TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(ZIG_TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);
    if (message->info.dst_endpoint == SENSOR_ENDPOINT)
    {
        switch (message->info.cluster)
        {
        case ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY:
            ESP_LOGI(ZIG_TAG, "Identify pressed");
            // esp_zb_zcl_identify_cmd_req()
            float old_led_hz = led_hz;
            bool old_int_led_mode = data.int_led_mode;
            data.int_led_mode = 1;
            led_hz = 4;

            if (ledTaskHandle != NULL)
            {
                vTaskDelete(ledTaskHandle);
                ledTaskHandle = NULL;
            }
            xTaskCreate(led_task, "led_task", 4096, NULL, 3, &ledTaskHandle);

            vTaskDelay(IDENTITY_INTERVAL / portTICK_PERIOD_MS);

            // led_hz = 0;
            if (ledTaskHandle != NULL)
            {
                vTaskDelete(ledTaskHandle);
                ledTaskHandle = NULL;
            }
            led_hz = old_led_hz;
            int_led_blink();
            data.int_led_mode = old_int_led_mode;
            // xTaskCreate(led_task, "led_task", 4096, NULL, 3, &ledTaskHandle);
            ESP_LOGI(ZIG_TAG, "Identify exit");
            break;
        case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
            int value = *(int *)message->attribute.data.value;
            ESP_LOGW(ZIG_TAG, "Power-on behavior %d", value);
            data.start_up_on_off = value;
            write_NVS("start_up_on_off", data.start_up_on_off);
            break;
        default:
            ESP_LOGI(ZIG_TAG, "Message data: cluster(0x%x), attribute(0x%x)  ", message->info.cluster, message->attribute.id);
        }
    }
    bool usb_state = 0;
    /*if (message->info.dst_endpoint == HA_ONOFF_SWITCH_ENDPOINT)
    {
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF)
        {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL)
            {
                usb_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : usb_state;
                ESP_LOGI(TAG, "USB sets to %s", usb_state ? "On" : "Off");
                usb_driver_set_power(usb_state);
            }
        }
    }*/
    if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF)
    {

        if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL)
        {
            if (message->info.dst_endpoint == SENSOR_ENDPOINT)
            {
                bool usb_state =
                    message->attribute.data.value ? *(bool *)message->attribute.data.value : 0;
                ESP_LOGI(ZIG_TAG, "USB sets to %s", usb_state ? "On" : "Off");
                usb_driver_set_power(usb_state);
            }
            else
            {
                bool switch_state =
                    message->attribute.data.value ? *(bool *)message->attribute.data.value : 0;
                ESP_LOGI(ZIG_TAG, "Endpoint %d, sets to %s", message->info.dst_endpoint, switch_state ? "On" : "Off");
                if (message->info.dst_endpoint == INT_LED_ENDPOINT)
                {
                    if (switch_state == 0)
                    {
                        gpio_set_level(INT_LED_GPIO, LED_OFF_STATE);
                    }
                    data.int_led_mode = switch_state;
                    write_NVS("int_led_mode", switch_state);
                }
                else if (message->info.dst_endpoint == EXT_LED_ENDPOINT)
                {
                    if (switch_state == 0)
                    {
                        ESP_LOGI(ZIG_TAG, "cmd is OFF, so EXT_LED_GPIO (1)");
                        gpio_set_level(EXT_LED_GPIO, 1);
                    }
                    else if (data.USB_state == 1)
                    {
                        ESP_LOGI(ZIG_TAG, "USB is ON, so EXT_LED_GPIO (0)");
                        gpio_set_level(EXT_LED_GPIO, 0);
                    }
                    data.ext_led_mode = switch_state;
                    write_NVS("ext_led_mode", switch_state);
                }
            }
        }
    }
    return ret;
}

static esp_err_t zb_read_attr_resp_handler(const esp_zb_zcl_cmd_read_attr_resp_message_t *message)
{

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, ZIG_TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, ZIG_TAG, "Received message: error status(%d)",
                        message->info.status);

    esp_zb_zcl_read_attr_resp_variable_t *variable = message->variables;
    while (variable)
    {
        ESP_LOGI(ZIG_TAG, "Read attribute response: status(%d), cluster(0x%x), attribute(0x%x), type(0x%x), value(%d)", variable->status,
                 message->info.cluster, variable->attribute.id, variable->attribute.data.type,
                 variable->attribute.data.value ? *(uint8_t *)variable->attribute.data.value : 0);

        if (message->info.dst_endpoint == SENSOR_ENDPOINT)
        {
            switch (message->info.cluster)
            {
            case ESP_ZB_ZCL_CLUSTER_ID_TIME:
                ESP_LOGI(ZIG_TAG, "Server time recieved %lu", *(uint32_t *)variable->attribute.data.value);
                struct timeval tv;
                tv.tv_sec = *(uint32_t *)variable->attribute.data.value + 946684800 - 1080; // after adding OTA cluster time shifted to 1080 sec... strange issue ...
                settimeofday(&tv, NULL);
                time_updated = true;

                uint32_t boot_time = *(uint32_t *)variable->attribute.data.value;
                ESP_LOGI(ZIG_TAG, "Write new boot time %lu", boot_time);
                esp_zb_zcl_status_t state_boot_time = esp_zb_zcl_set_attribute_val(SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TIME, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TIME_LAST_SET_TIME_ID, &boot_time, false);

                if (state_boot_time != ESP_ZB_ZCL_STATUS_SUCCESS)
                {
                    ESP_LOGE(ZIG_TAG, "Setting boot time attribute failed!");
                }

                break;
            default:
                ESP_LOGI(ZIG_TAG, "Message data: cluster(0x%x), attribute(0x%x)  ", message->info.cluster, variable->attribute.id);
            }
        }

        variable = variable->next;
    }

    return ESP_OK;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id)
    {
    /*case ESP_ZB_CORE_IDENTIFY_EFFECT_CB_ID:
        ret = esp_zb_zcl_identify_cmd_req((esp_zb_zcl_identify_cmd_t *)message);
        ESP_LOGW(TAG, "ESP_ZB_CORE_IDENTIFY_EFFECT_CB_ID");
        break;*/
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
        ret = zb_read_attr_resp_handler((esp_zb_zcl_cmd_read_attr_resp_message_t *)message);
        break;
    /*case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID:
        ret = (esp_zb_zcl_cmd_default_resp_message_t *)message;
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback, %s", callback_id, ret);
        break;*/
    default:
        ESP_LOGD(ZIG_TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

void read_server_time()
{
    esp_zb_zcl_read_attr_cmd_t read_req;
    read_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    uint16_t attributes[] = {ESP_ZB_ZCL_ATTR_TIME_LOCAL_TIME_ID};

    read_req.attr_number = sizeof(attributes) / sizeof(uint16_t);
    read_req.attr_field = attributes;

    read_req.clusterID = ESP_ZB_ZCL_CLUSTER_ID_TIME;

    read_req.zcl_basic_cmd.dst_endpoint = 1;
    read_req.zcl_basic_cmd.src_endpoint = 1;
    read_req.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
    esp_zb_zcl_read_attr_cmd_req(&read_req);
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    esp_zb_zdo_signal_leave_params_t *leave_params = NULL;
    switch (sig_type)
    {
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status != ESP_OK)
        {
            connected = false;
            led_hz = 2;
            ESP_LOGW(ZIG_TAG, "Stack %s failure with %s status, steering", esp_zb_zdo_signal_to_string(sig_type), esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        else
        {
            /* device auto start successfully and on a formed network */
            connected = true;
            led_hz = 0;
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(ZIG_TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel());
            read_server_time();
            send_bin_cfg_option(1, data.USB_state);
            send_bin_cfg_option(2, data.int_led_mode);
            send_bin_cfg_option(3, data.ext_led_mode);
            send_alarm_state(data.alarm_state);
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        leave_params = (esp_zb_zdo_signal_leave_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        if (leave_params->leave_type == ESP_ZB_NWK_LEAVE_TYPE_RESET)
        {
            ESP_LOGI(ZIG_TAG, "Reset device");
            esp_zb_factory_reset();
        }
        break;
    default:
        ESP_LOGI(ZIG_TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

static void set_zcl_string(char *buffer, char *value)
{
    buffer[0] = (char)strlen(value);
    memcpy(buffer + 1, value, buffer[0]);
}

static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    uint16_t undefined_value;
    undefined_value = 0x8000;

    uint16_t hundred_value;
    hundred_value = 100;

    uint16_t one_value;
    one_value = 1;

    float undefined_float = 0;
    float float_zero = 0;
    float float_one = 1;

    /* basic cluster create with fully customized */
    set_zcl_string(manufacturer, HW_MANUFACTURER);
    set_zcl_string(model, HW_MODEL);
    char ota_upgrade_file_version[10];
    sprintf(ota_upgrade_file_version, "%d", OTA_FW_VERSION);
    set_zcl_string(firmware_version, ota_upgrade_file_version);
    set_zcl_string(firmware_date, FW_BUILD_DATE);
    uint8_t dc_power_source;
    dc_power_source = 4;
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, manufacturer);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, model);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &dc_power_source); /**< DC source. */

    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID, firmware_version);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID, firmware_date);

    /* identify cluster create with fully customized */
    uint8_t identyfi_id;
    identyfi_id = 0;
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    esp_zb_identify_cluster_add_attr(esp_zb_identify_cluster, ESP_ZB_ZCL_CMD_IDENTIFY_IDENTIFY_ID, &identyfi_id);

    /* Electrical cluster */
    esp_zb_attribute_list_t *esp_zb_electrical_meas_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT);

    /* DC Electrical attributes */
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_ID, &undefined_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_MAX_ID, &undefined_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_MIN_ID, &undefined_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DCPOWER_ID, &undefined_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_POWER_MAX_ID, &undefined_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_POWER_MIN_ID, &undefined_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_ID, &undefined_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_MAX_ID, &undefined_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_MIN_ID, &undefined_value);

    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_DIVISOR_ID, &hundred_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_POWER_DIVISOR_ID, &hundred_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_DIVISOR_ID, &hundred_value);

    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_MULTIPLIER_ID, &one_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_POWER_MULTIPLIER_ID, &one_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_MULTIPLIER_ID, &one_value);

    /* AC Electrical attributes */
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMSCURRENT_ID, &undefined_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMS_CURRENT_MAX_ID, &undefined_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMS_CURRENT_MIN_ID, &undefined_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACTIVE_POWER_ID, &undefined_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACTIVE_POWER_MAX_ID, &undefined_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACTIVE_POWER_MIN_ID, &undefined_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMSVOLTAGE_ID, &undefined_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMS_VOLTAGE_MAX_ID, &undefined_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMS_VOLTAGE_MIN_ID, &undefined_value);

    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACCURRENT_DIVISOR_ID, &hundred_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACPOWER_DIVISOR_ID, &hundred_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACVOLTAGE_DIVISOR_ID, &hundred_value);

    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACCURRENT_MULTIPLIER_ID, &one_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACPOWER_MULTIPLIER_ID, &one_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACVOLTAGE_MULTIPLIER_ID, &one_value);

    /* Temperature cluster */
    esp_zb_attribute_list_t *esp_zb_temperature_meas_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);
    esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &undefined_value);
    esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_ID, &undefined_value);
    esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_ID, &undefined_value);

    /*
        esp_zb_attribute_list_t *esp_zb_ias_zone_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IAS_ZONE);
        esp_zb_ias_zone_cluster_add_attr(esp_zb_ias_zone_cluster, ESP_ZB_ZCL_ATTR_IAS_ZONE_ZONESTATE_ID, &undefined_value);
        esp_zb_ias_zone_cluster_add_attr(esp_zb_ias_zone_cluster, ESP_ZB_ZCL_ATTR_IAS_ZONE_ZONESTATUS_ID, &undefined_value);
        esp_zb_ias_zone_cluster_add_attr(esp_zb_ias_zone_cluster, ESP_ZB_ZCL_ATTR_IAS_ZONE_ZONETYPE_ID, &undefined_value);
    */

    /*
        esp_zb_attribute_list_t *esp_zb_on_off_switch_config_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ON_OFF_SWITCH_CONFIG);

        esp_zb_on_off_switch_config_cluster_add_attr(esp_zb_on_off_switch_config_cluster, esp_zb_on_off_switch_config_cluster
        esp_zb_on_off_switch_config_cluster_add_attr(esp_zb_on_off_switch_config_cluster, ESP_ZB_ZCL_ATTR_IAS_ZONE_ZONESTATUS_ID, &undefined_value);
        esp_zb_on_off_switch_config_cluster_add_attr(esp_zb_on_off_switch_config_cluster, ESP_ZB_ZCL_ATTR_IAS_ZONE_ZONETYPE_ID, &undefined_value);

    esp_zb_cluster_list_add_on_off_switch_config_cluster
    */

    /* Time cluster */
    esp_zb_attribute_list_t *esp_zb_server_time_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TIME);

    esp_zb_attribute_list_t *esp_zb_client_time_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TIME);
    esp_zb_time_cluster_add_attr(esp_zb_client_time_cluster, ESP_ZB_ZCL_ATTR_TIME_LAST_SET_TIME_ID, &undefined_value);

    /* IAS cluster */
    esp_zb_ias_zone_cluster_cfg_t ias_cluster_cfg = {
        .zone_type = ESP_ZB_ZCL_IAS_ZONE_ZONETYPE_STANDARD_CIE,
        .zone_state = ESP_ZB_ZCL_IAS_ZONE_ZONESTATE_NOT_ENROLLED,
        .zone_status = ESP_ZB_ZCL_IAS_ZONE_ZONE_STATUS_ALARM1,
        .ias_cie_addr = ESP_ZB_ZCL_ZONE_IAS_CIE_ADDR_DEFAULT,
        .zone_id = 0,
    };
    esp_zb_attribute_list_t *esp_zb_ias_zone_cluster = esp_zb_ias_zone_cluster_create(&ias_cluster_cfg);

    /** Create ota client cluster with attributes.
     *  Manufacturer code, image type and file version should match with configured values for server.
     *  If the client values do not match with configured values then it shall discard the command and
     *  no further processing shall continue.
     */
    esp_zb_ota_cluster_cfg_t ota_cluster_cfg = {
        //.ota_upgrade_downloaded_file_ver = OTA_FW_VERSION,
        //.ota_upgrade_manufacturer = OTA_UPGRADE_MANUFACTURER,
        //.ota_upgrade_image_type = OTA_UPGRADE_IMAGE_TYPE,
        .ota_upgrade_file_version = OTA_FW_VERSION,        // OTA_UPGRADE_RUNNING_FILE_VERSION,
        .ota_upgrade_downloaded_file_ver = OTA_FW_VERSION, // OTA_UPGRADE_DOWNLOADED_FILE_VERSION,
        .ota_upgrade_manufacturer = OTA_UPGRADE_MANUFACTURER,
        .ota_upgrade_image_type = OTA_UPGRADE_IMAGE_TYPE,
    };
    esp_zb_attribute_list_t *esp_zb_ota_client_cluster = esp_zb_ota_cluster_create(&ota_cluster_cfg);
    /** add client parameters to ota client cluster */
    esp_zb_zcl_ota_upgrade_client_variable_t variable_config = {
        .timer_query = ESP_ZB_ZCL_OTA_UPGRADE_QUERY_TIMER_COUNT_DEF,
        .hw_version = OTA_UPGRADE_HW_VERSION,
        .max_data_size = OTA_UPGRADE_MAX_DATA_SIZE,
    };

    esp_zb_ota_cluster_add_attr(esp_zb_ota_client_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_CLIENT_DATA_ID, (void *)&variable_config);

    /* create cluster list with ota cluster */

    esp_zb_on_off_cluster_cfg_t on_off_cfg = {};
    esp_zb_attribute_list_t *esp_zb_on_off_cluster = esp_zb_on_off_cluster_create(&on_off_cfg);

    // uint8_t default_value = 0xff;
    esp_zb_on_off_cluster_add_attr(esp_zb_on_off_cluster, ESP_ZB_ZCL_ATTR_ON_OFF_START_UP_ON_OFF, &data.start_up_on_off);
    // esp_zb_on_off_cluster_cfg_t
    // esp_zb_on_off_cluster_add_attr(&on_off_cfg,

    /* Create full cluster list enabled on device */
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_electrical_meas_cluster(esp_zb_cluster_list, esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_temperature_meas_cluster(esp_zb_cluster_list, esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_time_cluster(esp_zb_cluster_list, esp_zb_server_time_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_cluster_list_add_time_cluster(esp_zb_cluster_list, esp_zb_client_time_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_ota_cluster(esp_zb_cluster_list, esp_zb_ota_client_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    // esp_zb_cluster_list_add_groups_cluster(esp_zb_cluster_list,
    esp_zb_cluster_list_add_ias_zone_cluster(esp_zb_cluster_list, esp_zb_ias_zone_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // esp_zb_cluster_list_add_on_off_switch_config_cluster(esp_zb_cluster_list,
    // esp_zb_cluster_list_add_power_config_cluster(esp_zb_cluster_list,
    // esp_zb_cluster_list_add_scenes_cluster(esp_zb_cluster_list,

    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();

    // esp_zb_mains_power_outlet_cfg_t switch_cfg = ESP_ZB_DEFAULT_MAINS_POWER_OUTLET_CONFIG();

    // esp_zb_on_off_switch_cfg_t switch_cfg = ESP_ZB_DEFAULT_ON_OFF_SWITCH_CONFIG();

    // esp_zb_ep_list_t *esp_zb_on_off_switch_ep = esp_zb_on_off_switch_ep_create(HA_ONOFF_SWITCH_ENDPOINT, &switch_cfg);

    // esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_mains_power_outlet_ep_create(4, &switch_cfg);

    // esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_on_off_switch_cfg_cluster_create(switch_cfg)
    // esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_on_off_switch_ep_create(4, &switch_cfg);

    esp_zb_on_off_cluster_cfg_t on_off_cfg2 = {};
    esp_zb_attribute_list_t *esp_zb_on_off_cluster2 = esp_zb_on_off_cluster_create(&on_off_cfg2);
    esp_zb_cluster_list_t *esp_zb_cluster_list2 = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list2, esp_zb_on_off_cluster2, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_endpoint_config_t endpoint_config1 = {
        .endpoint = SENSOR_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_OUTPUT_DEVICE_ID,
        .app_device_version = 0};
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, endpoint_config1);

    esp_zb_endpoint_config_t endpoint_config2 = {
        .endpoint = INT_LED_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID,
        .app_device_version = 0};
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list2, endpoint_config2);

    esp_zb_endpoint_config_t endpoint_config3 = {
        .endpoint = EXT_LED_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID,
        .app_device_version = 0};
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list2, endpoint_config3);

    /* END */
    esp_zb_device_register(esp_zb_ep_list);
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(true));
    esp_zb_main_loop_iteration();
}

void app_main(void)
{

    setup_NVS();

    register_button();
    register_alarm_input();

    data.int_led_mode = read_NVS("int_led_mode");
    data.ext_led_mode = read_NVS("ext_led_mode");
    data.USB_state = read_NVS("USB_state");
    data.start_up_on_off = read_NVS("start_up_on_off");

    init_outputs();
    // switch_driver_init(button_func_pair, PAIR_SIZE(button_func_pair), esp_zb_buttons_handler);
    // init_pullup_i2c_pins();

    ESP_ERROR_CHECK(i2cdev_init());

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    // ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    xTaskCreate(led_task, "led_task", 4096, NULL, 3, &ledTaskHandle);
    xTaskCreate(ina219_task, "ina219_task", 4096, NULL, 3, NULL);
    xTaskCreate(CPUtemp_task, "CPUtemp_task", 4096, NULL, 3, NULL);

    xTaskCreate(update_attribute, "Update_attribute_value", 4096, NULL, 2, NULL);
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 1, NULL);
}
