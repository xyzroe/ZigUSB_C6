#ifndef TOOLS_H
#define TOOLS_H

#include <stdint.h>
#include <stdbool.h>

static void get_rtc_time();
bool int_to_bool(int32_t value);

void setup_NVS();
int read_NVS(const char *nvs_key);
bool write_NVS(const char *nvs_key, int value);

#endif // TOOLS_H