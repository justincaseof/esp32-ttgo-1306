#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_wifi_internal.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_task.h"
#include "esp_eth.h"
#include "esp_system.h"

#include "rom/ets_sys.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "tcpip_adapter.h"
#include "esp_log.h"

esp_err_t init_ip_config();

