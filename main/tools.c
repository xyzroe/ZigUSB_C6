
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_mac.h"
#include <time.h>
#include "string.h"

#include "main.h"
#include "tools.h"
#include "const.h"
#include "zigbee.h"

void get_rtc_time()
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
    ESP_LOGI(__func__, "Opening Non-Volatile Storage (NVS) handle... %s", (err != ESP_OK) ? T_STATUS_FAILED : T_STATUS_DONE);
    if (err == ESP_OK)
    {

        // Read
        int32_t restart_counter = 0; // value will default to 0, if not set yet in NVS
        err = nvs_get_i32(my_handle, "restart_counter", &restart_counter);
        ESP_LOGI(__func__, "Reading restart counter from NVS ... %s", (err != ESP_OK) ? T_STATUS_FAILED : T_STATUS_DONE);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGI(__func__, "Restart counter = %" PRIu32, restart_counter);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGI(__func__, "The value is not initialized yet!");
            break;
        default:
            ESP_LOGI(__func__, "Error (%s) reading!", esp_err_to_name(err));
        }

        // Write
        restart_counter++;
        err = nvs_set_i32(my_handle, "restart_counter", restart_counter);
        ESP_LOGI(__func__, "Updating restart counter in NVS ... %s", (err != ESP_OK) ? T_STATUS_FAILED : T_STATUS_DONE);

        // Commit written value.
        // After setting any values, nvs_commit() must be called to ensure changes are written
        // to flash storage. Implementations may write to storage at other times,
        // but this is not guaranteed.
        err = nvs_commit(my_handle);
        ESP_LOGI(__func__, "Committing updates in NVS ... %s", (err != ESP_OK) ? T_STATUS_FAILED : T_STATUS_DONE);

        // Close
        nvs_close(my_handle);
    }
}

int read_NVS(const char *nvs_key)
{
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);

    // Read
    // ESP_LOGI(__func__, "Reading restart counter from NVS ... ");
    // int32_t restart_counter = 0; // value will default to 0, if not set yet in NVS
    int32_t value = 0;
    err = nvs_get_i32(my_handle, nvs_key, &value);
    switch (err)
    {
    case ESP_OK:
        ESP_LOGI(__func__, "%s is %ld ", nvs_key, value);
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        ESP_LOGE(__func__, "The value is not initialized yet!");
        int value = 0;

        char *substring = "_led_mode";
        if (strstr(nvs_key, substring) != NULL)
        {
            value = 1;
        }
        err = nvs_set_i32(my_handle, nvs_key, value);
        ESP_LOGW(__func__, "Updating %s in NVS ... %s", nvs_key, (err != ESP_OK) ? T_STATUS_FAILED : T_STATUS_DONE);
        break;
    default:
        ESP_LOGE(__func__, "Error (%s) reading!", esp_err_to_name(err));
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
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    err = nvs_set_i32(my_handle, nvs_key, value);
    ESP_LOGI(__func__, "Write value... %s", (err != ESP_OK) ? T_STATUS_FAILED : T_STATUS_DONE);

    // Commit written value.
    // After setting any values, nvs_commit() must be called to ensure changes are written
    // to flash storage. Implementations may write to storage at other times,
    // but this is not guaranteed.
    err = nvs_commit(my_handle);
    ESP_LOGI(__func__, "Commit updates... %s", (err != ESP_OK) ? T_STATUS_FAILED : T_STATUS_DONE);

    // Close
    nvs_close(my_handle);

    if (err != ESP_OK)
    {
        return false;
    }
    return true;
}

const char *get_endpoint_name(int endpoint)
{
    switch (endpoint)
    {
    case SENSOR_ENDPOINT:
        return "SENSOR";
    case INT_LED_ENDPOINT:
        return "INT_LED";
    case EXT_LED_ENDPOINT:
        return "EXT_LED";
    default:
        return "Unknown";
    }
}

float random_float(float min, float max)
{
    return min + (max - min) * ((float)rand() / RAND_MAX);
}

float round_to_4_decimals(float value)
{
    return roundf(value * 10000) / 10000;
}

void set_zcl_string(char *buffer, char *value)
{
    buffer[0] = (char)strlen(value);
    memcpy(buffer + 1, value, buffer[0]);
}

void print_chip_info()
{
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    uint8_t mac[6];

    esp_chip_info(&chip_info);

    ESP_LOGW(__func__, "This is %s chip with %d CPU core(s), %s%s%s%s, ",
             CONFIG_IDF_TARGET,
             chip_info.cores,
             (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
             (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
             (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    ESP_LOGW(__func__, "Silicon revision v%d.%d", major_rev, minor_rev);

    if (esp_flash_get_size(NULL, &flash_size) != ESP_OK)
    {
        ESP_LOGE(__func__, "Get flash size failed");
        return;
    }

    ESP_LOGW(__func__, "%" PRIu32 "MB %s flash",
             flash_size / (uint32_t)(1024 * 1024),
             (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    ESP_LOGW(__func__, "Minimum free heap size: %" PRIu32 " bytes", esp_get_minimum_free_heap_size());

    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK)
    {
        ESP_LOGW(__func__, "Base MAC (WiFi STA): %02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    else if (esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP) == ESP_OK)
    {
        ESP_LOGW(__func__, "Base MAC (WiFi SoftAP): %02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    else if (esp_read_mac(mac, ESP_MAC_BT) == ESP_OK)
    {
        ESP_LOGW(__func__, "Base MAC (Bluetooth): %02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    else if (esp_read_mac(mac, ESP_MAC_ETH) == ESP_OK)
    {
        ESP_LOGW(__func__, "Base MAC (Ethernet): %02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    else
    {
        ESP_LOGE(__func__, "Failed to get any MAC address");
    }
}

void heap_stats()
{
    multi_heap_info_t heap_info;
    heap_caps_get_info(&heap_info, MALLOC_CAP_8BIT);

    size_t total_heap_size = heap_caps_get_total_size(MALLOC_CAP_8BIT);
    size_t free_heap_size = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    size_t largest_free_block = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    size_t minimum_free_size = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);

    float frag_heap_percentage = 0.0;
    if (free_heap_size > 0)
    {
        frag_heap_percentage = (1.0 - ((float)largest_free_block / (float)free_heap_size)) * 100.0;
    }

    float free_heap_percentage = ((float)free_heap_size / (float)total_heap_size) * 100.0;

    ESP_LOGI(__func__, "total: %d, free: %d, largest free block: %d, minimum free size: %d",
             total_heap_size,
             free_heap_size,
             largest_free_block,
             minimum_free_size);

    ESP_LOGW(__func__, "free: %.2f%% (%d bytes), fragmentation: %.2f%%",
             free_heap_percentage,
             free_heap_size,
             frag_heap_percentage);
}

void debug_task(void *pvParameters)
{
    ESP_LOGW(__func__, "started");
    while (1)
    {
        heap_stats();
        if (connected)
        {
            if (!time_updated)
            {
                ESP_LOGE(__func__, "Time not updated yet");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(DEBUG_TASK_INTERVAL));
    }
}