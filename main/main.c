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
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/spi_master.h"

static const char *TAG = "MACRO_BOARD";

#define PORT 5005

#define GPIO_ROTARY_A    32
#define GPIO_ROTARY_B    33
#define GPIO_BUTTON      21

#define PIN_NUM_MISO       -1 // Not used for ST7789
#define PIN_NUM_MOSI       23
#define PIN_NUM_CLK        18
#define PIN_NUM_CS         5
#define PIN_NUM_DC         2
#define PIN_NUM_RST        4
#define PIN_NUM_BK_LIGHT   -1

#ifndef CONFIG_ESP_WIFI_SSID
    #error "Please configure WiFi in menuconfig (Project Configuration)!"
#endif

esp_lcd_panel_handle_t panel_handle = NULL;

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

void setup_display() {
    ESP_LOGI("DISP", "Initializing SPI Bus...");
    
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_NUM_CLK,
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 320 * 240 * 2 + 8
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_NUM_DC,
        .cs_gpio_num = PIN_NUM_CS,
        .pclk_hz = 20 * 1000 * 1000, // Reduced to 20MHz for stability
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI3_HOST, &io_config, &io_handle));

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_RST,
        .rgb_endian = LCD_RGB_ENDIAN_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    
    // LANDSCAPE MODE
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true)); 
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, true));

    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
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
    char rx_buffer[128];
    int addr_family = AF_INET;
    int ip_protocol = 0;
    
    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE("NET", "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(5005);
    
    if (bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0) {
        ESP_LOGE("NET", "Socket unable to bind: errno %d", errno);
    }
    ESP_LOGI("NET", "Socket bound to port 5005");

    struct sockaddr_in pc_addr;
    pc_addr.sin_addr.s_addr = inet_addr(CONFIG_PC_IP_ADDRESS);
    pc_addr.sin_family = AF_INET;
    pc_addr.sin_port = htons(5005);

    int flags = fcntl(sock, F_GETFL);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    int command_buffer;

    while(1) {
        if (xQueueReceive(xCommandQueue, &command_buffer, pdMS_TO_TICKS(10)) == pdTRUE) {
            const char* msg = "";
            if (command_buffer == CMD_BTN_CLICK) msg = "CLICK";
            else if (command_buffer == CMD_VOL_UP) msg = "VOL_UP";
            else if (command_buffer == CMD_VOL_DOWN) msg = "VOL_DOWN";
            
            sendto(sock, msg, strlen(msg), 0, (struct sockaddr *)&pc_addr, sizeof(pc_addr));
        }

        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

        if (len > 0) {
            rx_buffer[len] = 0;
            
            if (strncmp(rx_buffer, "STATS:", 6) == 0) {
                int cpu, ram;
                sscanf(rx_buffer, "STATS:%d:%d", &cpu, &ram);
                
                if (xSemaphoreTake(xStatsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    global_stats.cpu_usage = cpu;
                    global_stats.ram_usage = ram;
                    xSemaphoreGive(xStatsMutex);
                    
                    ESP_LOGI("NET", "Updated Stats -> CPU: %d%% | RAM: %d%%", cpu, ram);
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

uint16_t color_565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

void task_display_ui(void *pvParameters) {
    setup_display();
    
    const int h_res = 320;
    const int v_lines = 20;
    
    uint16_t *line_buffer = heap_caps_malloc(h_res * v_lines * sizeof(uint16_t), MALLOC_CAP_DMA);

    ESP_LOGI("DISP", "Clearing Screen...");
    
    memset(line_buffer, 0, h_res * v_lines * sizeof(uint16_t));

    for (int y = 0; y < 240; y += v_lines) {
        esp_lcd_panel_draw_bitmap(panel_handle, 0, y, h_res, y + v_lines, line_buffer);
    }

    while(1) {
        int cpu = 0, ram = 0;
        
        if (xSemaphoreTake(xStatsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            cpu = global_stats.cpu_usage;
            ram = global_stats.ram_usage;
            xSemaphoreGive(xStatsMutex);
        }

        memset(line_buffer, 0, h_res * v_lines * sizeof(uint16_t));
        
        int bar_width = (cpu * h_res) / 100; 
        uint16_t color = (cpu > 80) ? color_565(255, 0, 0) : color_565(0, 255, 0);

        for (int y = 0; y < v_lines; y++) {
            for (int x = 0; x < h_res; x++) {
                if (x < bar_width) {
                    line_buffer[y * h_res + x] = color;
                } else {
                    line_buffer[y * h_res + x] = 0x0000;
                }
            }
        }
        esp_lcd_panel_draw_bitmap(panel_handle, 0, 50, h_res, 50 + v_lines, line_buffer);

        memset(line_buffer, 0, h_res * v_lines * sizeof(uint16_t));

        bar_width = (ram * h_res) / 100;
        
        for (int y = 0; y < v_lines; y++) {
            for (int x = 0; x < h_res; x++) {
                if (x < bar_width) {
                    line_buffer[y * h_res + x] = color_565(0, 0, 255);
                } else {
                    line_buffer[y * h_res + x] = 0x0000;
                }
            }
        }
        esp_lcd_panel_draw_bitmap(panel_handle, 0, 100, h_res, 100 + v_lines, line_buffer);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
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
