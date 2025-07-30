#include <stdio.h>
#include <string>
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
#include "esp_sntp.h"
#include <Espressif_MQTT_Client.h>
#include <ThingsBoard.h>

static const char *TAG = "Waypoint";

#define BLINK_GPIO (gpio_num_t)CONFIG_BLINK_GPIO

// ThingsBoard Configuration
#define TELEMETRY_PERIOD_SECONDS 30
#define THINGSBOARD_SERVER CONFIG_MQTT_SERVER
#define THINGSBOARD_PORT CONFIG_MQTT_PORT
#define THINGSBOARD_DEVICE_TOKEN CONFIG_MQTT_KEY

// ThingsBoard connection variables
static SemaphoreHandle_t tb_connected_sem = NULL;
static bool tb_connection_ready = false;
static StaticSemaphore_t tb_connected_sem_buffer;

// ThingsBoard client instance
static Espressif_MQTT_Client<> mqttClient;
static ThingsBoard tb(mqttClient);
static bool isNetworkConnected = false;

// PKI
extern "C" {
    extern const uint8_t whitetail_cert_pem_start[] asm("_binary_whitetail_pem_start");
    extern const uint8_t whitetail_cert_pem_end[] asm("_binary_whitetail_pem_end");
}
static std::string cert;

// WiFi
const char *WIFI_SSID = CONFIG_WIFI_SSID;
const char *WIFI_PASSWORD = CONFIG_WIFI_PASSWORD;
const char *NTP_SERVER = CONFIG_NTP_SERVER;

// Network interface handle
static esp_netif_t *s_sta_netif = NULL;

static uint8_t s_led_state = 0;

void blink_led(void)
{
    gpio_set_level(BLINK_GPIO, s_led_state);
}

void blink_task(void *pvParameters) {
    while (1) {
        blink_led();
        /* Toggle the LED state */
        s_led_state = !s_led_state;
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}

void initialize_sntp(void) {
    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();

    // Wait for time to be set
    time_t now = 0;
    struct tm timeinfo = {};
    int retry = 0;
    const int retry_count = 10;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    time(&now);
    localtime_r(&now, &timeinfo);
    ESP_LOGI(TAG, "System time is set. (Current time: %s)", asctime(&timeinfo));
}

void thingsboard_connection_task(void *pvParameters) {
    while (1) {
        if (!isNetworkConnected) {
            tb_connection_ready = false;
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            continue;
        }

        // Try to connect if not connected
        if (!tb_connection_ready) {
            ESP_LOGI(TAG, "Attempting to connect to ThingsBoard...");
            if (tb.connect(THINGSBOARD_SERVER, THINGSBOARD_DEVICE_TOKEN, THINGSBOARD_PORT)) {
                tb_connection_ready = true;
                ESP_LOGI(TAG, "Successfully connected to ThingsBoard");
                vTaskDelay(5000 / portTICK_PERIOD_MS);
                // Signal that connection is ready
                xSemaphoreGive(tb_connected_sem);
            } else {
                ESP_LOGE(TAG, "Failed to connect to ThingsBoard");
                vTaskDelay(10000 / portTICK_PERIOD_MS);
                continue;
            }
        }

        // Maintain connection
        if (!tb.loop()) {
            ESP_LOGW(TAG, "ThingsBoard connection lost");
            tb_connection_ready = false;
            // Take the semaphore to indicate disconnection
            xSemaphoreTake(tb_connected_sem, 0);
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// This task handles connecting and sending telemetry to ThingsBoard
void telemetry_task(void *pvParameters) {
    while (1) {
        // Wait for the telemetry period
        vTaskDelay((TELEMETRY_PERIOD_SECONDS * 1000) / portTICK_PERIOD_MS);

        // Check if ThingsBoard is connected (non-blocking check)
        if (xSemaphoreTake(tb_connected_sem, 0) == pdTRUE) {
            // Connection is available, give it back immediately
            xSemaphoreGive(tb_connected_sem);

            esp_netif_ip_info_t ip_info;
            esp_err_t ret = esp_netif_get_ip_info(s_sta_netif, &ip_info);

            if (ret == ESP_OK) {
                char ip_str[IP4ADDR_STRLEN_MAX];
                esp_ip4addr_ntoa(&ip_info.ip, ip_str, IP4ADDR_STRLEN_MAX);
                ESP_LOGI(TAG, "Sending telemetry: IP address %s", ip_str);

                if (!tb.sendTelemetryData("ip_address", ip_str)) {
                    ESP_LOGE(TAG, "Failed to send telemetry");
                    // Signal connection task to reconnect
                    tb_connection_ready = false;
                }
            } else {
                ESP_LOGE(TAG, "Failed to get IP address. Error: %s", esp_err_to_name(ret));
            }
        } else {
            ESP_LOGW(TAG, "ThingsBoard not connected, skipping telemetry");
        }
    }
}

void configure_thingsboard() {
    ESP_LOGI(TAG, "Configuring ThingsBoard...");

    // Only create the string the first time the function is called.
    if (cert.empty()) {
        const size_t cert_len = whitetail_cert_pem_end - whitetail_cert_pem_start;
        cert.assign(reinterpret_cast<const char*>(whitetail_cert_pem_start), cert_len);
    }

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
        initialize_sntp();
        isNetworkConnected = true;
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

    tb_connected_sem = xSemaphoreCreateBinaryStatic(&tb_connected_sem_buffer);

    xTaskCreate(
        blink_task,         // Task function
        "blink_task",        // Task name (for debugging)
        1024,               // Stack size in bytes
        NULL,               // Task parameter
        5,                  // Task priority
        NULL                // Task handle (optional)
    );

    xTaskCreate(
        thingsboard_connection_task,         // Task function
        "thingsboard_connection_task",        // Task name (for debugging)
        4096,               // Stack size in bytes
        NULL,               // Task parameter
        6,                  // Task priority
        NULL                // Task handle (optional)
    );

    xTaskCreate(
        telemetry_task,     // Task function
        "telemetry_task",    // Task name (for debugging)
        2048,               // Stack size
        NULL,               // Task parameter
        5,                  // Task priority
        NULL                // Task handle (optional)
    );
}
