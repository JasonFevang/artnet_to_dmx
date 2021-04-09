extern "C" {
    void app_main(void);
}

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "Art_Net.hpp"
#include "dmx.h"
#include "driver/gpio.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#define EXAMPLE_ESP_WIFI_SSID      "Trace Mountains"
#define EXAMPLE_ESP_WIFI_PASS      "turntwice"
#define EXAMPLE_ESP_MAXIMUM_RETRY  5

static const gpio_num_t dmx_tx_pin = GPIO_NUM_18;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "main";

static int s_retry_num = 0;

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        s_retry_num++;
        ESP_LOGI(TAG, "retry to connect to the AP");
        if (s_retry_num == EXAMPLE_ESP_MAXIMUM_RETRY) {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config;
    
    strcpy((char *)wifi_config.sta.ssid, EXAMPLE_ESP_WIFI_SSID);
    strcpy((char *)wifi_config.sta.password, EXAMPLE_ESP_WIFI_PASS);
    wifi_config.sta.scan_method = WIFI_FAST_SCAN;
    wifi_config.sta.bssid_set = 0;
    ESP_LOGI(TAG, "%d", sizeof(wifi_config.sta.bssid));
    memset(wifi_config.sta.bssid, 0, 6);
    wifi_config.sta.channel = 0;
    wifi_config.sta.listen_interval = 0;
    wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    wifi_config.sta.threshold.rssi = 0;
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
    vEventGroupDelete(s_wifi_event_group);
}

ArtNetBaseClass<0, 1> artnet;
esp_dmx_t dmx;

SemaphoreHandle_t arrive_sem;
uint8_t arrive_buf[MAX_DATA_SLOTS] = { 0 };
int arrive_len = 0;
SemaphoreHandle_t out_sem;
uint8_t out_buf[MAX_SLOTS] = { 0 };
int out_len = 0;
TaskHandle_t copy_task_h = nullptr;

void copy_task(void *arg){
    while(true){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Wait on the output sem first
        xSemaphoreTake(out_sem, portMAX_DELAY);
        xSemaphoreTake(arrive_sem, portMAX_DELAY);

        memcpy(out_buf + 1, arrive_buf, arrive_len);
        out_len = arrive_len + 1;

        xSemaphoreGive(arrive_sem);
        xSemaphoreGive(out_sem);
    }
}

// Have to be fast otherwise
void data_rx(uint8_t *data, int len){
    xSemaphoreTake(arrive_sem, portMAX_DELAY);
    memcpy(arrive_buf, data, len);
    arrive_len = len;
    xSemaphoreGive(arrive_sem);
    xTaskNotifyGive(copy_task_h);
}

// Take as long as you want
static uint8_t *dmx_buf_cb(void *arg, uint16_t *frame_sz){
    xSemaphoreTake(out_sem, portMAX_DELAY);
    *frame_sz = out_len;
    return out_buf;
} 

static void post_dmx_cb(void *arg){
    xSemaphoreGive(out_sem);
} 

static void blink(void *arg){
    gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);
    while(true){
        gpio_set_level(GPIO_NUM_5, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(GPIO_NUM_5, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{
    // Start blink task to indicate power
    xTaskCreate(blink, "blink", 2048, NULL, tskIDLE_PRIORITY + 1, NULL);

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Connect to wifi
    wifi_init_sta();


    // Configure dmx output
    esp_dmx_init_cfg_t dmx_cfg = ESP_DMX_INIT_CFG_DEFAULT();
    dmx_cfg.uart = UART_NUM_1;
    esp_dmx_tx_cfg_t dmx_tx_cfg = ESP_DMX_INIT_TX_DEFAULT();
    dmx_tx_cfg.pin_tx = dmx_tx_pin;
    dmx_tx_cfg.sync = esp_dmx_sync_stream;
    dmx_tx_cfg.tx_timer_group = TIMER_GROUP_0;
    dmx_tx_cfg.tx_timer_idx = TIMER_0;
    dmx_tx_cfg.tx_buf_cb = dmx_buf_cb;
    dmx_tx_cfg.post_tx_cb = post_dmx_cb;
    esp_err_t err = esp_dmx_init(&dmx, &dmx_cfg, NULL, &dmx_tx_cfg);
    if(err){
        ESP_LOGE(TAG, "errror initializing transmitter %d", err);
        return;
    }

    // Configure Art-Net
    artnet.begin(ST_NODE);
    artnet.setShortName("ANToDmx");
    artnet.setLongName("Artnet To Dmx");
    artnet.setNetAddress(0);
    artnet.setSubNetAddress(0);
    artnet.setOutputUniverseAddress(0, 1);
    artnet.setDataReceivedCallback(0, data_rx);

    arrive_sem = xSemaphoreCreateMutex();
    out_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(out_sem);
    xTaskCreate(copy_task, "copyTask", 2048, NULL, tskIDLE_PRIORITY+1, &copy_task_h);
}
