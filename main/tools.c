
#include "nvs_flash.h"
#include "esp_log.h"
#include <time.h>
#include "string.h"

#include "ZigUSB.h"
#include "tools.h"

static void get_rtc_time()
{
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%a %H:%M:%S", &timeinfo);
}

bool int_to_bool(int32_t value)
{
    return (value != 0);
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
