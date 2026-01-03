#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "MACRO_BOARD";

#define GPIO_ROTARY_A    18
#define GPIO_ROTARY_B    19
#define GPIO_BUTTON      21

typedef struct {
    int cpu_usage;
    int ram_usage;
} pc_stats_t;

pc_stats_t global_stats;

QueueHandle_t xRotaryQueue;
SemaphoreHandle_t xStatsMutex;

void task_input_monitor(void *pvParameters);
void task_network_ssh(void *pvParameters);
void task_display_ui(void *pvParameters);
static void IRAM_ATTR gpio_isr_handler(void* arg);


void setup_rotary_gpio()
{
    gpio_config_t io_conf = {};

    io_conf.pin_bit_mask = (1ULL << GPIO_ROTARY_A) | (1ULL << GPIO_ROTARY_B);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    
    io_conf.intr_type = GPIO_INTR_ANYEDGE; 
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << GPIO_BUTTON);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);

    gpio_isr_handler_add(GPIO_ROTARY_A, gpio_isr_handler, (void*) GPIO_ROTARY_A);
    gpio_isr_handler_add(GPIO_ROTARY_B, gpio_isr_handler, (void*) GPIO_ROTARY_B);
    gpio_isr_handler_add(GPIO_BUTTON, gpio_isr_handler, (void*) GPIO_BUTTON);
}

// Valid movement lookup table
// Index = (PrevState << 2) | CurrentState
static const int8_t encoder_table[16] = {
    0,  1, -1,  0, 
   -1,  0,  0,  1, 
    1,  0,  0, -1, 
    0, -1,  1,  0
};

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (gpio_num == GPIO_BUTTON) {
        static int64_t last_btn_time = 0;
        int64_t now = esp_timer_get_time();

        // 200ms debounce
        if (now - last_btn_time > 200000) { 
            last_btn_time = now;
            int send_val = 100;
            xQueueSendFromISR(xRotaryQueue, &send_val, &xHigherPriorityTaskWoken);
        }
    }
    else {
        uint8_t a = gpio_get_level(GPIO_ROTARY_A);
        uint8_t b = gpio_get_level(GPIO_ROTARY_B);

        static uint8_t prev_state = 0;
        uint8_t current_state = (a << 1) | b;
        uint8_t index = (prev_state << 2) | current_state;
        int step = encoder_table[index];
        prev_state = current_state;

        static int accumulator = 0;
        
        if (step != 0) {
            accumulator += step;
            if (accumulator >= 2) {
                int send_val = 1;
                xQueueSendFromISR(xRotaryQueue, &send_val, &xHigherPriorityTaskWoken);
                accumulator = 0;
            } 
            else if (accumulator <= -2) {
                int send_val = -1;
                xQueueSendFromISR(xRotaryQueue, &send_val, &xHigherPriorityTaskWoken);
                accumulator = 0;
            }
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing RTOS Objects...");

    xRotaryQueue = xQueueCreate(10, sizeof(int));
    xStatsMutex = xSemaphoreCreateMutex();
    

    if(xRotaryQueue == NULL || xStatsMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create RTOS objects");
        while(1);
    }
    xTaskCreate(task_input_monitor, "INPUT", 2048,  NULL, 5, NULL); // High Priority
    xTaskCreate(task_network_ssh,   "SSH",   8192,  NULL, 3, NULL); // Medium
    xTaskCreate(task_display_ui,    "DISP",  4096,  NULL, 1, NULL); // Low Priority
}

void task_network_ssh(void *pvParameters) {
    while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
}

void task_display_ui(void *pvParameters) {
    while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
}

void task_input_monitor(void *pvParameters) {
    setup_rotary_gpio();
    int buffer = 0;

    ESP_LOGI("INPUT", "Listening for events...");

    while (1) {
        if (xQueueReceive(xRotaryQueue, &buffer, portMAX_DELAY) == pdTRUE)
        {
            if (buffer == 100) {
                ESP_LOGI("INPUT", "Action: BUTTON CLICK");
                // TODO: Later, this might toggle a menu mode
            } 
            else if (buffer == 1) {
                ESP_LOGI("INPUT", "Action: CW (+1)");
                // TODO: Increase volume variable
            } 
            else if (buffer == -1) {
                ESP_LOGI("INPUT", "Action: CCW (-1)");
                // TODO: Decrease volume variable
            }
        }
    }
}
