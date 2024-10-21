#ifndef TOOLS_H
#define TOOLS_H

#include <stdint.h>
#include <stdbool.h>

void get_rtc_time();
bool int_to_bool(int32_t value);

void setup_NVS();
int read_NVS(const char *nvs_key);
bool write_NVS(const char *nvs_key, int value);

const char *get_endpoint_name(int endpoint);
float random_float(float min, float max);
float round_to_4_decimals(float value);

void set_zcl_string(char *buffer, char *value);

void print_chip_info();
void heap_stats();

void debug_task(void *pvParameters);

#endif // TOOLS_H