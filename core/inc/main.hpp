#pragma once

#define LED GPIO_NUM_14
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <JC4827W543R.hpp>
#include "rom/ets_sys.h" 
#include <esp_log.h>
#include <eeprom.h>
#include "esp_mac.h"
#include <lvgl.h>
#include <esp_task_wdt.h>
#include "nvs_flash.h"
#include "nvs.h"
#include <sys/lock.h>
#include <esp_timer.h>
#include <esp_err.h>
#include <esp_task_wdt.h>
#include <esp_private/esp_clk.h>
#include <driver/ledc.h>
#include <driver/i2c_master.h>

void setup(void);
void loop(void);
#define pdSECOND pdMS_TO_TICKS(1000)


