
#if ! __cplusplus >= 201402L
#error c++ standard is not at least c++14
#endif

#include <cstdio>
#include <thread>
#include <chrono>

#include "esp_system.h"
#include "esp_spi_flash.h"

#include <hash/crc.hpp>
#include <utils/pp_for_each.hpp>

/*add head files for wifi module*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include <string.h>

extern "C"{
	#include "websocket_server.h"
}

/*
 *
 * 1. Start with initializing LEDC module:
 *    a. Set the timer of LEDC first, this determines the frequency
 *       and resolution of PWM.
 *    b. Then set the LEDC channel you want to use,
 *       and bind with one of the timers.
 *
 * 2. You need first to install a default fade function,
 *    then you can use fade APIs.
 *
 * 3. You can also set a target duty directly without fading.
 *
 * 4. This demo uses GPIO32/33/25/26 as LEDC output,
 *    and it will change the duty repeatedly.
 *
 * 5. GPIO32/33 are from high speed channel group.
 *    GPIO25/26 are from low speed channel group.
 *
 */

#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       (32)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_HS_CH1_GPIO       (33)
#define LEDC_HS_CH1_CHANNEL    LEDC_CHANNEL_1

#define LEDC_LS_TIMER          LEDC_TIMER_1
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_LS_CH2_GPIO       (25)
#define LEDC_LS_CH2_CHANNEL    LEDC_CHANNEL_2
#define LEDC_LS_CH3_GPIO       (26)
#define LEDC_LS_CH3_CHANNEL    LEDC_CHANNEL_3

#define LEDC_TEST_CH_NUM       (4)
#define LEDC_TEST_DUTY         (4000)
#define LEDC_TEST_FADE_TIME    (8000)

//maybe we can use WiFi configuration that you can set via 'make menuconfig'
/*
#define ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define ESP_MAX_STA_CONN       CONFIG_MAX_STA_CONN
*/
#define ESP_WIFI_SSID      "esp32-mcb"
#define ESP_WIFI_PASS      "esp32pwd"
#define ESP_MAX_STA_CONN    5


static TaskHandle_t task_wifi_manager = NULL;

static const char *TAG = "wifi softAP";
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

static QueueHandle_t client_queue;
const static int client_queue_size = 10;

using namespace std::literals::chrono_literals;

void print_esp_info()
{
	// Print chip information
	esp_chip_info_t chip_info;
	esp_chip_info(&chip_info);
	std::printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
		chip_info.cores,
		(chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
		(chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

	std::printf("silicon revision %d, ", chip_info.revision);

	std::printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
		(chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

}

//void wifi_leds_controlled_by_pwm(void *pvParameter)
void wifi_leds_controlled_by_pwm()
{
	const static char* TAG = "wifi_leds_controlled_by_pwm";
	int ch;

    //Prepare and set configuration of timers that will be used by LED Controller

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HS_MODE,           // timer mode
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .timer_num = LEDC_HS_TIMER,           // timer index
        .freq_hz = 5000,                      // frequency of PWM signal
    };
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);

    // Prepare and set configuration of timer1 for low speed channels
    ledc_timer.speed_mode = LEDC_LS_MODE;
    ledc_timer.timer_num = LEDC_LS_TIMER;
    ledc_timer_config(&ledc_timer);

	/*
     * Prepare individual configuration
     * for each channel of LED Controller
     * by selecting:
     * - controller's channel number
     * - output duty cycle, set initially to 0
     * - GPIO number where LED is connected to
     * - speed mode, either high or low
     * - timer servicing selected channel
     *   Note: if different channels use one timer,
     *         then frequency and bit_num of these channels
     *         will be the same
     */
	ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
        {
            .gpio_num   = LEDC_HS_CH0_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .channel    = LEDC_HS_CH0_CHANNEL,
            .timer_sel  = LEDC_HS_TIMER,
            .duty       = 0,
        },
        {
            .gpio_num   = LEDC_HS_CH1_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .channel    = LEDC_HS_CH1_CHANNEL,
            .timer_sel  = LEDC_HS_TIMER,
            .duty       = 0,
       },
        {
            .gpio_num   = LEDC_LS_CH2_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .channel    = LEDC_LS_CH2_CHANNEL,
            .timer_sel  = LEDC_LS_TIMER,
            .duty       = 0,
        },
        {
            .gpio_num   = LEDC_LS_CH3_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .channel    = LEDC_LS_CH3_CHANNEL,
            .timer_sel  = LEDC_LS_TIMER,
            .duty       = 0,
        },
    };
	
	// Set LED Controller with previously prepared configuration
    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        ledc_channel_config(&ledc_channel[ch]);
    }

    // Initialize fade service.
    ledc_fade_func_install(0);

    while (1) {
        printf("1. LEDC fade up to duty = %d\n", LEDC_TEST_DUTY);
        for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
            ledc_set_fade_with_time(ledc_channel[ch].speed_mode,
                    ledc_channel[ch].channel, LEDC_TEST_DUTY, LEDC_TEST_FADE_TIME);
            ledc_fade_start(ledc_channel[ch].speed_mode,
                    ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
        }
        vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);

        printf("2. LEDC fade down to duty = 0\n");
        for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
            ledc_set_fade_with_time(ledc_channel[ch].speed_mode,
                    ledc_channel[ch].channel, 0, LEDC_TEST_FADE_TIME);
            ledc_fade_start(ledc_channel[ch].speed_mode,
                    ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
        }
        vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);
    }
}

//WebSocket frame receive queue

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_AP_STACONNECTED:
        ESP_LOGI(TAG, "station:"MACSTR" join, AID=%d",
                 MAC2STR(event->event_info.sta_connected.mac),
                 event->event_info.sta_connected.aid);
        break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
        ESP_LOGI(TAG, "station:"MACSTR"leave, AID=%d",
                 MAC2STR(event->event_info.sta_disconnected.mac),
                 event->event_info.sta_disconnected.aid);
        break;
    default:
        break;
    }
    return ESP_OK;
}

void wifi_init_softap()
{
    s_wifi_event_group = xEventGroupCreate();

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    tcpip_adapter_init();
	ESP_ERROR_CHECK(tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP));

	tcpip_adapter_ip_info_t info;
	memset(&info, 0, sizeof(info));
	IP4_ADDR(&info.ip, 192, 168, 1, 1);
	IP4_ADDR(&info.gw, 192, 168, 1, 1);
	IP4_ADDR(&info.netmask, 255, 255, 255, 0);
	ESP_LOGI(TAG,"setting gateway IP");
	ESP_ERROR_CHECK(tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP, &info));
	ESP_LOGI(TAG,"starting DHCPS adapter");
	ESP_ERROR_CHECK(tcpip_adapter_dhcps_start(TCPIP_ADAPTER_IF_AP));

    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_AP) );
/*
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = ESP_WIFI_SSID,
			.password = ESP_WIFI_PASS,
            .ssid_len = strlen(ESP_WIFI_SSID),
			.authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .max_connection = ESP_MAX_STA_CONN,
        },
    };
*/
	wifi_config_t wifi_config = {};
	strcpy((char*)wifi_config.ap.ssid,ESP_WIFI_SSID);
	strcpy((char*)wifi_config.ap.password,ESP_WIFI_PASS);
	wifi_config.ap.ssid_len = strlen(ESP_WIFI_SSID);
	wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
	wifi_config.ap.max_connection = ESP_MAX_STA_CONN;


    if (strlen(ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished.SSID:%s password:%s",
             ESP_WIFI_SSID, ESP_WIFI_PASS);
}

// handle websocket events
void websocket_callback(uint8_t num,WEBSOCKET_TYPE_t type,char* msg,uint64_t len) {
    const static char* TAG = "websocket_callback";
    int value;

    switch(type) {
        case WEBSOCKET_CONNECT:
            ESP_LOGI(TAG,"client %i connected!",num);
            break;
        case WEBSOCKET_DISCONNECT_EXTERNAL:
            ESP_LOGI(TAG,"client %i sent a disconnect message",num);
            //led_duty(0);
            break;
        case WEBSOCKET_DISCONNECT_INTERNAL:
            ESP_LOGI(TAG,"client %i was disconnected",num);
            break;
        case WEBSOCKET_DISCONNECT_ERROR:
            ESP_LOGI(TAG,"client %i was disconnected due to an error",num);
            //led_duty(0);
            break;
        case WEBSOCKET_TEXT:
            if(len) {
                switch(msg[0]) {
                    case 'L':
                        if(sscanf(msg,"L%i",&value)) {
                            ESP_LOGI(TAG,"LED value: %i",value);
                            //led_duty(value);
                            ws_server_send_text_all_from_callback(msg,len); // broadcast it!
                        }
                }
            }
            break;
        case WEBSOCKET_BIN:
            ESP_LOGI(TAG,"client %i sent binary message of size %i:\n%s",num,(uint32_t)len,msg);
            break;
        case WEBSOCKET_PING:
            ESP_LOGI(TAG,"client %i pinged us with message of size %i:\n%s",num,(uint32_t)len,msg);
            break;
        case WEBSOCKET_PONG:
            ESP_LOGI(TAG,"client %i responded to the ping",num);
            break;
    }
}

//deal with websocket server
static void http_serve(struct netconn *conn) {
    const static char* TAG = "http_server";
    const static char HTML_HEADER[] = "HTTP/1.1 200 OK\nContent-type: text/html\n\n";
    const static char ERROR_HEADER[] = "HTTP/1.1 404 Not Found\nContent-type: text/html\n\n";
    const static char JS_HEADER[] = "HTTP/1.1 200 OK\nContent-type: text/javascript\n\n";
    const static char CSS_HEADER[] = "HTTP/1.1 200 OK\nContent-type: text/css\n\n";
    const static char ICO_HEADER[] = "HTTP/1.1 200 OK\nContent-type: image/x-icon\n\n";
    struct netbuf* inbuf;
    static char* buf;
    static uint16_t buflen;
    static err_t err;

    // default page
    extern const uint8_t root_html_start[] asm("_binary_index_html_start");
    extern const uint8_t root_html_end[] asm("_binary_index_html_end");
    const uint32_t root_html_len = root_html_end - root_html_start;

    // test.js
    extern const uint8_t test_js_start[] asm("_binary_test_js_start");
    extern const uint8_t test_js_end[] asm("_binary_test_js_end");
    const uint32_t test_js_len = test_js_end - test_js_start;

    // test.css
    extern const uint8_t test_css_start[] asm("_binary_test_css_start");
    extern const uint8_t test_css_end[] asm("_binary_test_css_end");
    const uint32_t test_css_len = test_css_end - test_css_start;

    // favicon.ico
    extern const uint8_t favicon_ico_start[] asm("_binary_favicon_ico_start");
    extern const uint8_t favicon_ico_end[] asm("_binary_favicon_ico_end");
    const uint32_t favicon_ico_len = favicon_ico_end - favicon_ico_start;

    // error page
    extern const uint8_t error_html_start[] asm("_binary_error_html_start");
    extern const uint8_t error_html_end[] asm("_binary_error_html_end");
    const uint32_t error_html_len = error_html_end - error_html_start;

    netconn_set_recvtimeout(conn,1000); // allow a connection timeout of 1 second
    ESP_LOGI(TAG,"reading from client...");
    err = netconn_recv(conn, &inbuf);
    ESP_LOGI(TAG,"read from client");

	if(err==ERR_OK) {
        netbuf_data(inbuf, (void**)&buf, &buflen);
        if(buf) {
            // default page
            if(strstr(buf,"GET / ") && !strstr(buf,"Upgrade: websocket")) {
                ESP_LOGI(TAG,"Sending /");
                netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER)-1,NETCONN_NOCOPY);
                netconn_write(conn, root_html_start,root_html_len,NETCONN_NOCOPY);
                netconn_close(conn);
                netconn_delete(conn);
                netbuf_delete(inbuf);
            }
			// default page websocket
			else if(strstr(buf,"GET / ")&& strstr(buf,"Upgrade: websocket")) {
				ESP_LOGI(TAG,"Requesting websocket on /");
				ws_server_add_client(conn,buf,buflen,"/",websocket_callback);
				netbuf_delete(inbuf);
			}
            else if(strstr(buf,"GET /test.js ")) {
                ESP_LOGI(TAG,"Sending /test.js");
                netconn_write(conn, JS_HEADER, sizeof(JS_HEADER)-1,NETCONN_NOCOPY);
                netconn_write(conn, test_js_start, test_js_len,NETCONN_NOCOPY);
                netconn_close(conn);
                netconn_delete(conn);
                netbuf_delete(inbuf);
            }
            else if(strstr(buf,"GET /test.css ")) {
                ESP_LOGI(TAG,"Sending /test.css");
                netconn_write(conn, CSS_HEADER, sizeof(CSS_HEADER)-1,NETCONN_NOCOPY);
                netconn_write(conn, test_css_start, test_css_len,NETCONN_NOCOPY);
                netconn_close(conn);
                netconn_delete(conn);
                netbuf_delete(inbuf);
            }
            else if(strstr(buf,"GET /favicon.ico ")) {
                ESP_LOGI(TAG,"Sending favicon.ico");
                netconn_write(conn,ICO_HEADER,sizeof(ICO_HEADER)-1,NETCONN_NOCOPY);
                netconn_write(conn,favicon_ico_start,favicon_ico_len,NETCONN_NOCOPY);
                netconn_close(conn);
                netconn_delete(conn);
                netbuf_delete(inbuf);
            }
			else if(strstr(buf,"GET /")) {
                ESP_LOGI(TAG,"Unknown request, sending error page: %s",buf);
                netconn_write(conn, ERROR_HEADER, sizeof(ERROR_HEADER)-1,NETCONN_NOCOPY);
                netconn_write(conn, error_html_start, error_html_len,NETCONN_NOCOPY);
                netconn_close(conn);
                netconn_delete(conn);
                netbuf_delete(inbuf);
            }
            else {
                ESP_LOGI(TAG,"Unknown request");
                netconn_close(conn);
                netconn_delete(conn);
                netbuf_delete(inbuf);
            }
        }
        else {
            ESP_LOGI(TAG,"Unknown request (empty?...)");
            netconn_close(conn);
            netconn_delete(conn);
            netbuf_delete(inbuf);
        }
    }
	else {
        ESP_LOGI(TAG,"error on read, closing connection");
        netconn_close(conn);
        netconn_delete(conn);
        netbuf_delete(inbuf);
    }
}

// handles clients when they first connect. passes to a queue
static void server_task(void* pvParameters) {
    const static char* TAG = "server_task";
    struct netconn *conn, *newconn;
    static err_t err;
    client_queue = xQueueCreate(client_queue_size,sizeof(struct netconn*));

    conn = netconn_new(NETCONN_TCP);
    netconn_bind(conn,NULL,80);
    netconn_listen(conn);
    ESP_LOGI(TAG,"server listening");
    do {
        err = netconn_accept(conn, &newconn);
        ESP_LOGI(TAG,"new client");
        if(err == ERR_OK) {
            xQueueSendToBack(client_queue,&newconn,portMAX_DELAY);
        }
    } while(err == ERR_OK);
    netconn_close(conn);
    netconn_delete(conn);
    ESP_LOGE(TAG,"task ending, rebooting board");
    esp_restart();
}

// receives clients from queue, handles them
static void server_handle_task(void* pvParameters) {
  const static char* TAG = "server_handle_task";
  struct netconn* conn;
  ESP_LOGI(TAG,"task starting");
  for(;;) {
    xQueueReceive(client_queue,&conn,portMAX_DELAY);
    if(!conn) continue;
    http_serve(conn);
  }
  vTaskDelete(NULL);
}

extern "C" void app_main()
{
	printf("----app-main----\n");

	print_esp_info();

	wifi_init_softap();
  
	ws_server_start();

	xTaskCreate(&server_task,"server_task",3000,NULL,9,NULL);

	xTaskCreate(&server_handle_task,"server_handle_task",4000,NULL,6,NULL);

	wifi_leds_controlled_by_pwm();
}
