/* Simple WiFi Example
 This example code is in the Public Domain (or CC0 licensed, at your option.)
 Unless required by applicable law or agreed to in writing, this
 software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 CONDITIONS OF ANY KIND, either express or implied.
 */
#include "wifi.h"

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
 but we only care about one event - are we connected
 to the AP with an IP? */
const int WIFI_CONNECTED_BIT = BIT0;

static const char* LOGTAG = "BoardBratWifi";

static esp_err_t event_handler(void *ctx, system_event_t *event) {
	switch (event->event_id) {
	case SYSTEM_EVENT_STA_START:
		esp_wifi_connect();
		break;
	case SYSTEM_EVENT_STA_GOT_IP:
		ESP_LOGI(LOGTAG, "got ip:%s", ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
		xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
		break;
	case SYSTEM_EVENT_AP_STACONNECTED:
		ESP_LOGI(LOGTAG, "station:"MACSTR" join, AID=%d",
				MAC2STR(event->event_info.sta_connected.mac),
				event->event_info.sta_connected.aid);
		break;
	case SYSTEM_EVENT_AP_STADISCONNECTED:
		ESP_LOGI(LOGTAG, "station:"MACSTR"leave, AID=%d",
				MAC2STR(event->event_info.sta_disconnected.mac),
				event->event_info.sta_disconnected.aid);
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
		esp_wifi_connect();
		xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
		break;
	default:
		break;
	}
	return ESP_OK;
}

void wifi_init_softap() {
	wifi_event_group = xEventGroupCreate();

	tcpip_adapter_init();
	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	wifi_config_t wifi_config = {
			.ap = { .ssid = EXAMPLE_ESP_WIFI_SSID,
					.ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
					.password = EXAMPLE_ESP_WIFI_PASS,
					.max_connection = EXAMPLE_MAX_STA_CONN,
					.authmode = WIFI_AUTH_WPA_WPA2_PSK
			},
		};

	if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
		wifi_config.ap.authmode = WIFI_AUTH_OPEN;
	}

	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());

	ESP_LOGI(LOGTAG, "wifi_init_softap finished. SSID:%s password:%s", EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);

	tcpip_adapter_ip_info_t ap_ip;
	tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_AP, &ap_ip);
	ESP_LOGI( LOGTAG, "Network info:");
	ESP_LOGI( LOGTAG, "\t-->IP: "IPSTR, IP2STR(&(ap_ip.ip)));
	ESP_LOGI( LOGTAG, "\t-->GW: "IPSTR, IP2STR(&(ap_ip.gw)));
}

void wifi_init_sta() {
	wifi_event_group = xEventGroupCreate();

	tcpip_adapter_init();
	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	wifi_config_t wifi_config = {
			.sta = {
					.ssid = 	EXAMPLE_ESP_WIFI_SSID,
					.password = EXAMPLE_ESP_WIFI_PASS
			},
	};

	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());

	ESP_LOGI(LOGTAG, "wifi_init_sta finished.");
	ESP_LOGI(LOGTAG, "connect to ap SSID:%s password:%s", EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);

	tcpip_adapter_ip_info_t ap_ip;
	tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_AP, &ap_ip);
	ESP_LOGI( LOGTAG, "Network info:");
	ESP_LOGI( LOGTAG, "\t-->IP: "IPSTR, IP2STR(&(ap_ip.ip)));
	ESP_LOGI( LOGTAG, "\t-->GW: "IPSTR, IP2STR(&(ap_ip.gw)));
}

