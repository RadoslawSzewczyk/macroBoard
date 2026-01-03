#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"

static const char *TAG = "MACRO_BOARD";

#define GPIO_ROTARY_A    18
#define GPIO_ROTARY_B    19
#define GPIO_BUTTON      21

#ifndef CONFIG_ESP_WIFI_SSID
    #error "Please configure WiFi in menuconfig (Project Configuration)!"
#endif

typedef struct {
    int cpu_usage;
    int ram_usage;
} pc_stats_t;

typedef enum {
    CMD_VOL_UP    = 1,
    CMD_VOL_DOWN  = -1,
    CMD_BTN_CLICK = 100
} board_command_t;

pc_stats_t global_stats;

QueueHandle_t xRotaryQueue;
QueueHandle_t xCommandQueue;
SemaphoreHandle_t xStatsMutex;

void task_input_monitor(void *pvParameters);
void task_network_ssh(void *pvParameters);
void task_display_ui(void *pvParameters);
static void IRAM_ATTR gpio_isr_handler(void* arg);

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW("WIFI", "Disconnected. Retrying...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI("WIFI", "Connected! IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

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

void setup_wifi()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    }; 
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI("WIFI", "WiFi Initialized.");
}

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
            int send_val = CMD_BTN_CLICK;
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
                int send_val = CMD_VOL_UP;
                xQueueSendFromISR(xRotaryQueue, &send_val, &xHigherPriorityTaskWoken);
                accumulator = 0;
            } 
            else if (accumulator <= -2) {
                int send_val = CMD_VOL_DOWN;
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
    xCommandQueue = xQueueCreate(10, sizeof(int));
    xStatsMutex = xSemaphoreCreateMutex();

    if(xRotaryQueue == NULL || xCommandQueue == NULL || xStatsMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create RTOS objects");
        while(1);
    }
    setup_wifi();
    xTaskCreate(task_input_monitor, "INPUT", 2048,  NULL, 5, NULL); // High Priority
    xTaskCreate(task_network_ssh,   "SSH",   8192,  NULL, 3, NULL); // Medium
    xTaskCreate(task_display_ui,    "DISP",  4096,  NULL, 1, NULL); // Low Priority
}

void task_network_ssh(void *pvParameters) {
    int command_buffer;

    ESP_LOGI("NET", "Network Task Started. Waiting for commands...");

    while(1) {
        if (xQueueReceive(xCommandQueue, &command_buffer, portMAX_DELAY) == pdTRUE) {
            
            if (command_buffer == CMD_BTN_CLICK) {
                 ESP_LOGW("NET", "Sending UDP: BUTTON_CLICK");
            } else {
                 ESP_LOGW("NET", "Sending UDP: VOL_CHANGE %d", command_buffer);
            }
            
        }
    }
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
            if (buffer == CMD_BTN_CLICK) {
                ESP_LOGI("INPUT", "Action: BUTTON CLICK");
                // TODO: Later, this might toggle a menu mode
            } 
            else if (buffer == CMD_VOL_UP) {
                ESP_LOGI("INPUT", "Action: CW (+1)");
                // TODO: Increase volume variable
            } 
            else if (buffer == CMD_VOL_DOWN) {
                ESP_LOGI("INPUT", "Action: CCW (-1)");
                // TODO: Decrease volume variable
            }
            xQueueSend(xCommandQueue, &buffer, portMAX_DELAY);
        }
    }
}
