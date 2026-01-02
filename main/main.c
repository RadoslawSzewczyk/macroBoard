#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "System Initialized. Starting Scheduler...");
    
    while (1) {
        ESP_LOGI(TAG, "Free Memory: %lu bytes", esp_get_free_heap_size());
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}