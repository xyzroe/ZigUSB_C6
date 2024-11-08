#ifndef ZIGBEE_H
#define ZIGBEE_H

#include "esp_zigbee_core.h"
#include "const.h"

#ifdef __cplusplus
extern "C"
{
#endif

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

#if !defined CONFIG_ZB_ZCZR
#error Define ZB_ZCZR in idf.py menuconfig to compile light (Router) source code.
#endif

    extern char strftime_buf[64];
    extern uint16_t manuf_id;
    extern char manufacturer[16];
    extern char model[16];
    extern char firmware_version[16];
    extern char firmware_date[16];
    extern bool time_updated;
    extern bool connected;

    void zigbee_setup();

    void update_attributes(attribute_t attribute);
    void send_bin_cfg_option(int endpoint, bool value);
    void send_alarm_state(bool alarm);
    void force_update();
    void read_server_time();

    static void esp_zb_task(void *pvParameters);
    static void update_attribute_value(uint8_t endpoint, uint16_t cluster_id, uint8_t role, uint16_t attr_id, void *value, const char *attr_name);

    void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct);
    static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask);

    static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message);
    static esp_err_t zb_read_attr_resp_handler(const esp_zb_zcl_cmd_read_attr_resp_message_t *message);
    //static esp_err_t zb_ota_upgrade_status_handler(esp_zb_zcl_ota_upgrade_value_message_t message);
    static esp_err_t zb_ota_upgrade_query_image_resp_handler(esp_zb_zcl_ota_upgrade_query_image_resp_message_t message);
    static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message);

#ifdef __cplusplus
}
#endif

#endif // ZIGBEE_H