
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "ina219.h"
#include <time.h>
#include <sys/time.h>

#include "ha/esp_zigbee_ha_standard.h"
#include "esp_timer.h"
#include "esp_ota_ops.h"

#include "ZigUSB.h"
#include "tools.h"
#include "perf.h"

/*------ Clobal definitions -----------*/
/*------ Global definitions -----------*/
float led_hz = 0;
char strftime_buf[64];
DataStructure data;

char manufacturer[16];
char model[16];
char firmware_version[16];
char firmware_date[16];
bool time_updated = false;
bool connected = false;
bool usb_pwr_state = false;

uint16_t power = 0;
uint16_t voltage = 0;
uint16_t current = 0;
uint16_t CPU_temp = 0;
uint16_t uptime = 0;
float ina_bus_voltage = 0;
float ina_shunt_voltage = 0;
float ina_current = 0;
float ina_power = 0;

static const char *TAG = HW_MODEL;

static const char *ZIG_TAG = "ZB";

static const esp_partition_t *s_ota_partition = NULL;
static esp_ota_handle_t s_ota_handle = 0;

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

void send_bin_cfg_option(int endpoint, bool value)
{
    if (connected)
    {
        ESP_LOGW("send_bin_cfg_option", "attribute to %d", value);
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
    else
    {
        ESP_LOGW("send_bin_cfg_option", "no connection! attribute to %d", value);
    }
}

void send_alarm_state(bool alarm)
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

/* --------- User task section -----------------*/

/*----------------------------------------*/

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

/* Manual reporting atribute to coordinator */
/*static void reportAttribute(uint8_t endpoint, uint16_t clusterID, uint16_t attributeID, void *value, uint8_t value_length)
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
}*/

void send_update_attribute_and_ias()
{
    if (connected)
    {
        static const char *local_tag = "z -> ias+attr";

        int_led_blink();

        // Send current alarm state value
        send_alarm_state(data.alarm_state);

        // Write new temperature value
        esp_zb_zcl_status_t state_tmp = esp_zb_zcl_set_attribute_val(SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &CPU_temp, false);
        // Check for error
        if (state_tmp != ESP_ZB_ZCL_STATUS_SUCCESS)
        {
            ESP_LOGE(local_tag, "Setting temperature attribute failed!");
        }

        // DC clusters
        // Write new current value
        esp_zb_zcl_status_t state_current = esp_zb_zcl_set_attribute_val(SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_ID, &current, false);
        // Check for error
        if (state_current != ESP_ZB_ZCL_STATUS_SUCCESS)
        {
            ESP_LOGE(local_tag, "Setting current attribute failed!");
        }

        // Write new voltage value
        esp_zb_zcl_status_t state_voltage = esp_zb_zcl_set_attribute_val(SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_ID, &voltage, false);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        // Check for error
        if (state_voltage != ESP_ZB_ZCL_STATUS_SUCCESS)
        {
            ESP_LOGE(local_tag, "Setting voltage attribute failed!");
        }

        // Write new power value
        esp_zb_zcl_status_t state_power = esp_zb_zcl_set_attribute_val(SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DCPOWER_ID, &power, false);
        // Check for error
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

        ESP_LOGI(local_tag, "temperature = %d, current = %d, voltage = %d, power = %d", CPU_temp, current, voltage, power);
    }
}

/* Task for update attribute value */
void update_attribute()
{
    static const char *local_tag = "update_attribute";
    while (1)
    {

        send_update_attribute_and_ias();

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
        /*case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
            int value = *(int *)message->attribute.data.value;
            ESP_LOGW(ZIG_TAG, "Power-on behavior %d", value);
            data.start_up_on_off = value;
            write_NVS("start_up_on_off", data.start_up_on_off);
            break;*/
        default:
            ESP_LOGI(ZIG_TAG, "Message data: cluster(0x%x), attribute(0x%x)  ", message->info.cluster, message->attribute.id);
        }
    }
    // bool usb_state = 0;
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

size_t ota_data_len_;
size_t ota_header_len_;
bool ota_upgrade_subelement_;
uint8_t ota_header_[6];

static esp_err_t zb_ota_upgrade_status_handler(esp_zb_zcl_ota_upgrade_value_message_t message)
{
    static uint32_t total_size = 0;
    static uint32_t offset = 0;
    static int64_t start_time = 0;
    esp_err_t ret = ESP_OK;

    if (message.info.status == ESP_ZB_ZCL_STATUS_SUCCESS)
    {
        switch (message.upgrade_status)
        {
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_START:
            ESP_LOGI(TAG, "-- OTA upgrade start");
            start_time = esp_timer_get_time();
            s_ota_partition = esp_ota_get_next_update_partition(NULL);
            assert(s_ota_partition);
            ret = esp_ota_begin(s_ota_partition, 0, &s_ota_handle);
            ESP_RETURN_ON_ERROR(ret, TAG, "Failed to begin OTA partition, status: %s", esp_err_to_name(ret));
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_RECEIVE:
            size_t payload_size = message.payload_size;
            const uint8_t *payload = message.payload;

            total_size = message.ota_header.image_size;
            offset += payload_size;

            ESP_LOGI(TAG, "-- OTA Client receives data: progress [%ld/%ld]", offset, total_size);

            /* Read and process the first sub-element, ignoring everything else */
            while (ota_header_len_ < 6 && payload_size > 0)
            {
                ota_header_[ota_header_len_] = payload[0];
                ota_header_len_++;
                payload++;
                payload_size--;
            }

            if (!ota_upgrade_subelement_ && ota_header_len_ == 6)
            {
                if (ota_header_[0] == 0 && ota_header_[1] == 0)
                {
                    ota_upgrade_subelement_ = true;
                    ota_data_len_ =
                        (((int)ota_header_[5] & 0xFF) << 24) | (((int)ota_header_[4] & 0xFF) << 16) | (((int)ota_header_[3] & 0xFF) << 8) | ((int)ota_header_[2] & 0xFF);
                    ESP_LOGD(TAG, "OTA sub-element size %zu", ota_data_len_);
                }
                else
                {
                    ESP_LOGE(TAG, "OTA sub-element type %02x%02x not supported", ota_header_[0], ota_header_[1]);
                    return ESP_FAIL;
                }
            }

            if (ota_data_len_)
            {
                payload_size = fmin(ota_data_len_, payload_size);
                ota_data_len_ -= payload_size;

                if (message.payload_size && message.payload)
                {
                    ret = esp_ota_write(s_ota_handle, payload, payload_size);
                    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to write OTA data to partition, status: %s", esp_err_to_name(ret));
                }
            }
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_APPLY:
            ESP_LOGI(TAG, "-- OTA upgrade apply");
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_CHECK:
            ret = offset == total_size ? ESP_OK : ESP_FAIL;
            ESP_LOGI(TAG, "-- OTA upgrade check status: %s", esp_err_to_name(ret));
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_FINISH:
            ESP_LOGI(TAG, "-- OTA Finish");
            ESP_LOGI(TAG, "-- OTA Information: version: 0x%lx, manufacturer code: 0x%x, image type: 0x%x, total size: %ld bytes, cost time: %lld ms,",
                     message.ota_header.file_version, message.ota_header.manufacturer_code, message.ota_header.image_type,
                     message.ota_header.image_size, (esp_timer_get_time() - start_time) / 1000);
            ret = esp_ota_end(s_ota_handle);
            ESP_RETURN_ON_ERROR(ret, TAG, "Failed to end OTA partition, status: %s", esp_err_to_name(ret));
            ret = esp_ota_set_boot_partition(s_ota_partition);
            ESP_RETURN_ON_ERROR(ret, TAG, "Failed to set OTA boot partition, status: %s", esp_err_to_name(ret));
            ESP_LOGW(TAG, "Prepare to restart system");
            esp_restart();
            break;
        default:
            ESP_LOGI(TAG, "OTA status: %d", message.upgrade_status);
            break;
        }
    }
    return ret;
}

static esp_err_t zb_ota_upgrade_query_image_resp_handler(esp_zb_zcl_ota_upgrade_query_image_resp_message_t message)
{
    esp_err_t ret = ESP_OK;
    if (message.info.status == ESP_ZB_ZCL_STATUS_SUCCESS)
    {
        ESP_LOGI(TAG, "Queried OTA image from address: 0x%04hx, endpoint: %d", message.server_addr.u.short_addr, message.server_endpoint);
        ESP_LOGI(TAG, "Image version: 0x%lx, manufacturer code: 0x%x, image size: %ld", message.file_version, message.manufacturer_code,
                 message.image_size);
    }
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Approving OTA image upgrade");
    }
    else
    {
        ESP_LOGI(TAG, "Rejecting OTA image upgrade, status: %s", esp_err_to_name(ret));
    }
    return ret;
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
    case ESP_ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
        uint16_t manuf_id = OTA_UPGRADE_MANUFACTURER;
        esp_zb_set_node_descriptor_manufacturer_code(manuf_id);
        break;
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        ESP_LOGW(ZIG_TAG, "CORE_SET_ATTR_VALUE_CB action(0x%x) callback", callback_id);
        break;
    case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
        ret = zb_read_attr_resp_handler((esp_zb_zcl_cmd_read_attr_resp_message_t *)message);
        ESP_LOGW(ZIG_TAG, "CORE_CMD_READ_ATTR_RESP_CB action(0x%x) callback", callback_id);
        break;
    case ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID:
        ret = zb_ota_upgrade_status_handler(*(esp_zb_zcl_ota_upgrade_value_message_t *)message);
        break;
    case ESP_ZB_CORE_OTA_UPGRADE_SRV_QUERY_IMAGE_CB_ID:
        ret = zb_ota_upgrade_query_image_resp_handler(*(esp_zb_zcl_ota_upgrade_query_image_resp_message_t *)message);
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
            send_update_attribute_and_ias();
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

static void zigbee_config_endpoints_attributes()
{
    uint16_t undefined_value;
    undefined_value = 0x8000;

    uint16_t hundred_value;
    hundred_value = 100;

    uint16_t one_value;
    one_value = 1;

    // basic cluster create with fully customized
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

    // identify cluster create with fully customized
    uint8_t identyfi_id;
    identyfi_id = 0;
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    esp_zb_identify_cluster_add_attr(esp_zb_identify_cluster, ESP_ZB_ZCL_CMD_IDENTIFY_IDENTIFY_ID, &identyfi_id);

    // Electrical cluster
    esp_zb_attribute_list_t *esp_zb_electrical_meas_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT);

    // DC Electrical attributes
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_ID, &undefined_value);
    // esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_MAX_ID, &undefined_value);
    // esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_MIN_ID, &undefined_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DCPOWER_ID, &undefined_value);
    // esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_POWER_MAX_ID, &undefined_value);
    // esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_POWER_MIN_ID, &undefined_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_ID, &undefined_value);
    // esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_MAX_ID, &undefined_value);
    // esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_MIN_ID, &undefined_value);

    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_DIVISOR_ID, &hundred_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_POWER_DIVISOR_ID, &hundred_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_DIVISOR_ID, &hundred_value);

    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_MULTIPLIER_ID, &one_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_POWER_MULTIPLIER_ID, &one_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_MULTIPLIER_ID, &one_value);

    // AC Electrical attributes
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMSCURRENT_ID, &undefined_value);
    // esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMS_CURRENT_MAX_ID, &undefined_value);
    // esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMS_CURRENT_MIN_ID, &undefined_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACTIVE_POWER_ID, &undefined_value);
    // esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACTIVE_POWER_MAX_ID, &undefined_value);
    // esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACTIVE_POWER_MIN_ID, &undefined_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMSVOLTAGE_ID, &undefined_value);
    // esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMS_VOLTAGE_MAX_ID, &undefined_value);
    // esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMS_VOLTAGE_MIN_ID, &undefined_value);

    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACCURRENT_DIVISOR_ID, &hundred_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACPOWER_DIVISOR_ID, &hundred_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACVOLTAGE_DIVISOR_ID, &hundred_value);

    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACCURRENT_MULTIPLIER_ID, &one_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACPOWER_MULTIPLIER_ID, &one_value);
    esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACVOLTAGE_MULTIPLIER_ID, &one_value);

    // Temperature cluster
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

    // IAS cluster
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
    uint16_t manuf_id = OTA_UPGRADE_MANUFACTURER;
    esp_zb_ota_cluster_cfg_t ota_cluster_cfg = {
        //.ota_upgrade_downloaded_file_ver = OTA_FW_VERSION,
        //.ota_upgrade_manufacturer = OTA_UPGRADE_MANUFACTURER,
        //.ota_upgrade_image_type = OTA_UPGRADE_IMAGE_TYPE,
        .ota_upgrade_file_version = OTA_FW_VERSION,        // OTA_UPGRADE_RUNNING_FILE_VERSION,
        .ota_upgrade_downloaded_file_ver = OTA_FW_VERSION, // OTA_UPGRADE_DOWNLOADED_FILE_VERSION,
        .ota_upgrade_manufacturer = manuf_id,
        .ota_upgrade_image_type = OTA_UPGRADE_IMAGE_TYPE,
    };
    esp_zb_attribute_list_t *esp_zb_ota_client_cluster = esp_zb_ota_cluster_create(&ota_cluster_cfg);
    /** add client parameters to ota client cluster */
    esp_zb_zcl_ota_upgrade_client_variable_t variable_config = {
        .timer_query = ESP_ZB_ZCL_OTA_UPGRADE_QUERY_TIMER_COUNT_DEF,
        .hw_version = OTA_UPGRADE_HW_VERSION,
        .max_data_size = OTA_UPGRADE_MAX_DATA_SIZE,
    };

    uint16_t ota_upgrade_server_addr = 0xffff;
    uint8_t ota_upgrade_server_ep = 0xff;
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = 1,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_TEST_DEVICE_ID,
        .app_device_version = 0,
    };

    esp_zb_ota_cluster_add_attr(esp_zb_ota_client_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_CLIENT_DATA_ID, (void *)&variable_config);
    esp_zb_ota_cluster_add_attr(esp_zb_ota_client_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ADDR_ID, (void *)&ota_upgrade_server_addr);
    esp_zb_ota_cluster_add_attr(esp_zb_ota_client_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ENDPOINT_ID, (void *)&ota_upgrade_server_ep);

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

    esp_zb_on_off_cluster_cfg_t on_off_cfg3 = {};
    esp_zb_attribute_list_t *esp_zb_on_off_cluster3 = esp_zb_on_off_cluster_create(&on_off_cfg3);
    esp_zb_cluster_list_t *esp_zb_cluster_list3 = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list3, esp_zb_on_off_cluster3, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

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
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list3, endpoint_config3);

    /* END */
    esp_zb_device_register(esp_zb_ep_list);
}

static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    // float undefined_float = 0;
    // float float_zero = 0;
    // float float_one = 1;

    zigbee_config_endpoints_attributes();

    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

    ESP_ERROR_CHECK(esp_zb_start(true));
    esp_zb_main_loop_iteration();
}

void app_main(void)
{

    ESP_LOGW(TAG, "FW verison 0x%x (%d) date: %s", OTA_FW_VERSION, OTA_FW_VERSION, FW_BUILD_DATE);

    setup_NVS();

    register_button_1();
    register_button_2();
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

    xTaskCreate(led_task, "led_task", 4096, NULL, 5, &ledTaskHandle);
    xTaskCreate(CPUtemp_task, "CPUtemp_task", 4096, NULL, 4, NULL);
    xTaskCreate(ina219_task, "ina219_task", 4096, NULL, 3, NULL);
    xTaskCreate(update_attribute, "update_attribute_task", 4096, NULL, 2, NULL);
    xTaskCreate(esp_zb_task, "zigbee_main_task", 4096, NULL, 1, NULL);
}
