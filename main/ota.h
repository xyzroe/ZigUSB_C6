#pragma once

#include <ha/esp_zigbee_ha_standard.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t zb_ota_upgrade_status_handler(esp_zb_zcl_ota_upgrade_value_message_t message);

#ifdef __cplusplus
} // extern "C"
#endif