/*
 * FireBeetle 2 ESP32-P4 LED Blink Test
 * Based on DFRobot Arduino example converted to ESP-IDF
 * LED is on GPIO3 (IO3)
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "LED_BLINK";

#define LED_PIN 3  // On-board LED on FireBeetle 2 ESP32-P4

void app_main(void)
{
    ESP_LOGI(TAG, "=================================");
    ESP_LOGI(TAG, "  FireBeetle 2 ESP32-P4 LED Test");
    ESP_LOGI(TAG, "  LED on GPIO%d", LED_PIN);
    ESP_LOGI(TAG, "=================================");

    // Configure LED pin as output
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    int led_state = 0;
    
    while (1) {
        led_state = !led_state;
        gpio_set_level(LED_PIN, led_state);
        ESP_LOGI(TAG, "LED %s", led_state ? "ON" : "OFF");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
