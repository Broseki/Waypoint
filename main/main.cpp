#include <stdio.h>
#include <string> // Added for std::string
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "portmacro.h"
#include "sdkconfig.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include <Espressif_MQTT_Client.h>
#include <ThingsBoard.h>

static const char *TAG = "Waypoint";

#define BLINK_GPIO (gpio_num_t)CONFIG_BLINK_GPIO

// ThingsBoard Configuration
#define TELEMETRY_PERIOD_SECONDS 30
#define THINGSBOARD_SERVER CONFIG_MQTT_SERVER
#define THINGSBOARD_PORT CONFIG_MQTT_PORT
#define THINGSBOARD_DEVICE_TOKEN CONFIG_MQTT_KEY

// PKI
extern "C" {
    extern const uint8_t whitetail_cert_pem_start[] asm("_binary_whitetail_pem_start");
    extern const uint8_t whitetail_cert_pem_end[] asm("_binary_whitetail_pem_end");
}

// WiFi
const char *WIFI_SSID = CONFIG_WIFI_SSID;
const char *WIFI_PASSWORD = CONFIG_WIFI_PASSWORD;

// ThingsBoard client instance
Espressif_MQTT_Client<> mqttClient;
ThingsBoard tb(mqttClient);

// Network interface handle
static esp_netif_t *s_sta_netif = NULL;

static uint8_t s_led_state = 0;

void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

// This task is now dedicated to blinking the LED
void blink_task(void *pvParameters) {
    while (1) {
        blink_led();
        /* Toggle the LED state */
        s_led_state = !s_led_state;
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}

// This task handles connecting and sending telemetry to ThingsBoard
void telemetry_task(void *pvParameters) {
    while (1) {
        // Wait for the telemetry period before sending the next update
        vTaskDelay((TELEMETRY_PERIOD_SECONDS * 1000) / portTICK_PERIOD_MS);

        if (!tb.connected()) {
            ESP_LOGI(TAG, "Connecting to ThingsBoard...");
            if (!tb.connect(THINGSBOARD_SERVER, THINGSBOARD_DEVICE_TOKEN, THINGSBOARD_PORT)) {
                ESP_LOGE(TAG, "Failed to connect to ThingsBoard");
                continue; // Retry connection in the next cycle
            }
            ESP_LOGI(TAG, "Connected to ThingsBoard");
        }

        esp_netif_ip_info_t ip_info;
        esp_err_t ret = esp_netif_get_ip_info(s_sta_netif, &ip_info);

        if (ret == ESP_OK) {
            char ip_str[IP4ADDR_STRLEN_MAX];
            esp_ip4addr_ntoa(&ip_info.ip, ip_str, IP4ADDR_STRLEN_MAX);
            ESP_LOGI(TAG, "Sending telemetry: IP address %s", ip_str);
            tb.sendTelemetryData("ip_address", ip_str);
        } else {
            ESP_LOGE(TAG, "Failed to get IP address. Error: %s", esp_err_to_name(ret));
        }

        // Process incoming MQTT messages and maintain connection
        tb.loop();
    }
}

void configure_thingsboard() {
    ESP_LOGI(TAG, "Configuring ThingsBoard...");

    // Declare the string as static. It will now persist after the function returns.
    static std::string cert;

    // Only create the string the first time the function is called.
    if (cert.empty()) {
        const size_t cert_len = whitetail_cert_pem_end - whitetail_cert_pem_start;
        cert.assign(reinterpret_cast<const char*>(whitetail_cert_pem_start), cert_len);
    }

    // The pointer from cert.c_str() will now remain valid indefinitely.
    mqttClient.set_server_certificate(cert.c_str());
    ESP_LOGI(TAG, "ThingsBoard configured.");
}

void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "WiFi Start");
                esp_wifi_connect();
                break;
            case WIFI_EVENT_STA_DISCONNECTED:
                ESP_LOGW(TAG, "WiFi Disconnected, retrying...");
                esp_wifi_connect();
                break;
            default:
                break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

void configure_wifi() {
    ESP_LOGI(TAG, "Initializing WiFi...");

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    // Create default WiFi station and store the handle
    s_sta_netif = esp_netif_create_default_wifi_sta();

    // Register WiFi and IP event handler
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config));
    strncpy(reinterpret_cast<char*>(wifi_config.sta.ssid), WIFI_SSID, sizeof(wifi_config.sta.ssid) -1);
    strncpy(reinterpret_cast<char*>(wifi_config.sta.password), WIFI_PASSWORD, sizeof(wifi_config.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi initialization complete. Connecting to %s...", WIFI_SSID);
}

void configure_nvs_flash() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

void configure_led(void)
{
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

void configure() {
    ESP_LOGI(TAG, "Configuring System...");
    configure_led();
    configure_nvs_flash();
    configure_wifi();
    configure_thingsboard();
    ESP_LOGI(TAG, "Configuration complete!");
}

extern "C" void app_main(void)
{
    /* Configure the peripheral according to the LED type */
    configure();

    xTaskCreate(
        blink_task,         // Task function
        "BlinkTask",        // Task name (for debugging)
        2048,               // Stack size in bytes
        NULL,               // Task parameter
        5,                  // Task priority
        NULL                // Task handle (optional)
    );

    xTaskCreate(
        telemetry_task,     // Task function
        "TelemetryTask",    // Task name (for debugging)
        4096,               // Stack size (increased for TLS)
        NULL,               // Task parameter
        5,                  // Task priority
        NULL                // Task handle (optional)
    );
}
