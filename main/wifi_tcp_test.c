/*
 * ESP-IDF WiFi TCP Client Test
 * Converted from Arduino WiFiMulti example
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

static const char *TAG = "WIFI_TCP";

// WiFi credentials - UPDATE THESE
#define WIFI_SSID "SSID"
#define WIFI_PASS "passpasspass"

// TCP server settings
#define TCP_HOST "192.168.1.10"
#define TCP_PORT 1337

// Event group for WiFi connection
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int retry_count = 0;
#define MAX_RETRY 10

// WiFi event handler
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (retry_count < MAX_RETRY) {
            esp_wifi_connect();
            retry_count++;
            ESP_LOGI(TAG, "Retrying WiFi connection... (%d/%d)", retry_count, MAX_RETRY);
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "WiFi connected!");
        ESP_LOGI(TAG, "IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        retry_count = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// Initialize WiFi station mode
static esp_err_t wifi_init_sta(void)
{
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                    ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                    IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Waiting for WiFi...");

    // Wait for connection or failure
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "WiFi connection failed!");
        return ESP_FAIL;
    }
}

// TCP client task
static void tcp_client_task(void *pvParameters)
{
    char rx_buffer[512];
    char *request = "GET /index.html HTTP/1.1\r\n\r\n";

    while (1) {
        ESP_LOGI(TAG, "Connecting to %s:%d", TCP_HOST, TCP_PORT);

        // Create socket
        int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            continue;
        }

        // Server address
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(TCP_HOST);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(TCP_PORT);

        // Connect
        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            ESP_LOGE(TAG, "Connection failed: errno %d", errno);
            ESP_LOGI(TAG, "Waiting 5 seconds before retrying...");
            close(sock);
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "Connected!");

        // Send request
        err = send(sock, request, strlen(request), 0);
        if (err < 0) {
            ESP_LOGE(TAG, "Send failed: errno %d", errno);
            close(sock);
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "Sent: %s", request);

        // Set receive timeout
        struct timeval timeout;
        timeout.tv_sec = 5;
        timeout.tv_usec = 0;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        // Receive response
        int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len < 0) {
            ESP_LOGE(TAG, "Receive failed: errno %d", errno);
        } else if (len == 0) {
            ESP_LOGW(TAG, "Connection closed by server");
        } else {
            rx_buffer[len] = 0;
            ESP_LOGI(TAG, "Received %d bytes:", len);
            // Print first line only (like Arduino example)
            char *newline = strchr(rx_buffer, '\r');
            if (newline) *newline = 0;
            ESP_LOGI(TAG, "%s", rx_buffer);
        }

        ESP_LOGI(TAG, "Closing connection.");
        close(sock);

        ESP_LOGI(TAG, "Waiting 5 seconds before restarting...");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "=================================");
    ESP_LOGI(TAG, "  ESP32-P4 WiFi TCP Client Test");
    ESP_LOGI(TAG, "=================================");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Connect to WiFi
    if (wifi_init_sta() == ESP_OK) {
        // Start TCP client task
        xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);
    } else {
        ESP_LOGE(TAG, "Failed to connect to WiFi. Restarting...");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        esp_restart();
    }
}
