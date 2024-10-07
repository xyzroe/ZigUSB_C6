#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/temperature_sensor.h"
#include "iot_button.h"
#include "ina219.h"
#include "string.h"

#include "ZigUSB.h"
#include "perf.h"

static const char *TAG = "perf";

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

void led_task(void *pvParameters)
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

void register_button_1()
{
    // create gpio button
    button_config_t gpio_btn_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = LONG_PRESS_TIME,
        .short_press_time = SHORT_PRESS_TIME,
        .gpio_button_config = {
            .gpio_num = BTN_GPIO_1,
            .active_level = 0,
        },
    };

    button_handle_t gpio_btn = iot_button_create(&gpio_btn_cfg);
    if (NULL == gpio_btn)
    {
        ESP_LOGE("Button 1 boot", "Button create failed");
    }

    iot_button_register_cb(gpio_btn, BUTTON_SINGLE_CLICK, button_single_click_cb, NULL);
    iot_button_register_cb(gpio_btn, BUTTON_LONG_PRESS_START, button_long_press_cb, NULL);
}

void register_button_2()
{
    // create gpio button
    button_config_t gpio_btn_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = LONG_PRESS_TIME,
        .short_press_time = SHORT_PRESS_TIME,
        .gpio_button_config = {
            .gpio_num = BTN_GPIO_2,
            .active_level = 0,
        },
    };

    button_handle_t gpio_btn = iot_button_create(&gpio_btn_cfg);
    if (NULL == gpio_btn)
    {
        ESP_LOGE("Button 2 boot", "Button create failed");
    }

    iot_button_register_cb(gpio_btn, BUTTON_SINGLE_CLICK, button_single_click_cb, NULL);
    iot_button_register_cb(gpio_btn, BUTTON_LONG_PRESS_START, button_long_press_cb, NULL);
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

void CPUtemp_task(void *pvParameters)
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

void ina219_task(void *pvParameters)
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
