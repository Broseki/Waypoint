#include <stdio.h>
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

static const char *TAG = "Waypoint";

#define BLINK_GPIO CONFIG_BLINK_GPIO

// ThingsBoard Configuration
#define TELEMETRY_PERIOD_SECONDS 30
#define THINGSBOARD_SERVER CONFIG_MQTT_SERVER
#define THINGSBOARD_PORT CONFIG_MQTT_PORT
#define THINGSBOARD_DEVICE_TOKEN CONFIG_MQTT_KEY

// PKI
extern const uint8_t certs_whitetail_pem_start[] asm("_binary_certs_whitetail_pem_start");
extern const uint8_t certs_whitetail_pem_end[]   asm("_binary_certs_whitetail_pem_end");

// WiFi
const char *WIFI_SSID = CONFIG_WIFI_SSID;
const char *WIFI_PASSWORD = CONFIG_WIFI_PASSWORD;

static uint8_t s_led_state = 0;

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

void send_telemetry(void *pvParameters) {
    while (1) {
        blink_led();
        /* Toggle the LED state */
        s_led_state = !s_led_state;
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}

void blink_led_task(void *pvParameters) {
    while (1) {
        ESP_LOGI(TAG, "TELEMETRY");
        vTaskDelay((TELEMETRY_PERIOD_SECONDS * 1000) / portTICK_PERIOD_MS);
    }
}

static void configure_thingsboard() {
    return;
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
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

static void configure_wifi() {
    ESP_LOGI(TAG, "Initializing WiFi...");

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

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

    wifi_config_t wifi_config = { 0 };
    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, WIFI_PASSWORD, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi initialization complete. Connecting to %s...", WIFI_SSID);
}

static void configure_nvs_flash() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
}

static void configure_led(void)
{
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

static void configure() {
    ESP_LOGI(TAG, "Configuring System...");
    configure_led();
    configure_nvs_flash();
    configure_wifi();
    configure_thingsboard();
    ESP_LOGI(TAG, "Configuration complete!");
}

void app_main(void)
{
    /* Configure the peripheral according to the LED type */
    configure();

    xTaskCreate(
        blink_led_task,    // Task function
        "BlinkLED",        // Task name (for debugging)
        2048,               // Stack size in bytes
        NULL,               // Task parameter
        10,                 // Task priority
        NULL                // Task handle (optional)
    );

    xTaskCreate(
        send_telemetry,    // Task function
        "Telemetry",        // Task name (for debugging)
        2048,               // Stack size in bytes
        NULL,               // Task parameter
        10,                 // Task priority
        NULL                // Task handle (optional)
    );

}
