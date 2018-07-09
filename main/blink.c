/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "sdkconfig.h"
#include "fonts.h"
#include "math.h"
#include "esp_log.h"

#include "ssd1306.h"

#include "wifi.h"

#define BLINK_GPIO 				GPIO_NUM_16
#define RELAIS_GPIO 			GPIO_NUM_22

#define TEMPERATURE_DESIRED		270	// celsius

/* according to "esp_wroom_32_datasheet_en.pdf" */
#define I2C_EXAMPLE_MASTER_PORT 			I2C_NUM_1
#define I2C_EXAMPLE_MASTER_SCL_IO    		4    		/*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO    		5    		/*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0   		/*!< I2C master does not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0   		/*!< I2C master does not need buffer */
#define I2C_EXAMPLE_MASTER_FREQ_HZ    		100000     	/*!< I2C master clock frequency */

/* We are using HSPI */
#define SPI_MASTER_MISO_SDO 	GPIO_NUM_12
#define SPI_MASTER_MOSI_SDI		GPIO_NUM_13
#define SPI_MASTER_CLK			GPIO_NUM_14
#define SPI_MASTER_CS			GPIO_NUM_15
#define SPI_MASTER_RDY			GPIO_NUM_17

static const char* LOGTAG = "BoardBrat";

/* typedefs, structs, ... */
typedef enum {
	OFF,
	ON,
	BLINK_SLOW,
	BLINK_FAST
} gpio_output_mode_t;

/* internal vars */
const TickType_t 			xDelay = 100 / portTICK_PERIOD_MS;
static uint32_t 			temperatureCheck_tick_counter = 0;
static gpio_output_mode_t 	led_blinkmode = OFF;
static gpio_output_mode_t 	relais_mode = OFF;
//static portMUX_TYPE 		myMutex = portMUX_INITIALIZER_UNLOCKED;
spi_device_handle_t 		spi;
uint8_t 					max31865_data[16];
char 						buf[64];

static void i2c_example_master_init()
{
    int i2c_master_port = I2C_EXAMPLE_MASTER_PORT;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
}

static void spi_master_init() {
	esp_err_t ret;
	spi_bus_config_t buscfg = {
		.miso_io_num = 		SPI_MASTER_MISO_SDO,
		.mosi_io_num = 		SPI_MASTER_MOSI_SDI,
		.sclk_io_num = 		SPI_MASTER_CLK,
		.quadwp_io_num = 	-1,
		.quadhd_io_num = 	-1,
		.max_transfer_sz = 	128,
		.flags = 			SPICOMMON_BUSFLAG_MASTER
	};
	spi_device_interface_config_t devcfg = {
		.clock_speed_hz = 	4 * 1000 * 1000,        		// Clock out at 1 MHz
		.mode = 			1,                              // SPI mode 1
		.spics_io_num = 	SPI_MASTER_CS,            		// CS pin
		.queue_size = 		2
	};
	//Initialize the SPI bus
	ret = spi_bus_initialize(HSPI_HOST, &buscfg, 0);	// last arg: diable DMA
	ESP_ERROR_CHECK(ret);
	printf("spi_bus_initialize: %d\r\n", ret);

	ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
	ESP_ERROR_CHECK(ret);
	printf("spi_bus_add_device: %d\r\n", ret);
}

uint32_t max31865_read() {
	// == SET UP STUFF ===
	esp_err_t _result;
	spi_transaction_t _spi_transaction1;
	spi_transaction_t _spi_transaction2;
	memset(&_spi_transaction1, 0, sizeof(_spi_transaction1));		// init with zeros
	memset(&_spi_transaction2, 0, sizeof(_spi_transaction2));		// init with zeros

	// === TRANSMIT ===
	_spi_transaction1.length = 		8 * 1;                     		// len is in BITS
	_spi_transaction1.flags = 		SPI_TRANS_USE_TXDATA;			// use internal 4 byte buf
	_spi_transaction1.tx_data[0] = 	0x00;							// start at address 0x00

	_result = spi_device_transmit(spi, &_spi_transaction1);  		// Transmit!
	assert( _result == ESP_OK );            						// Should have had no issues.

	// === RECEIVE ===
	_spi_transaction2.length = 		8 * 4;								// number of bytes that we intent to receive
	_spi_transaction2.flags = SPI_TRANS_USE_RXDATA;

	_result = spi_device_transmit(spi, &_spi_transaction2);
	assert( _result == ESP_OK );

	// === RETURN RESULT ===
	if(_result!=ESP_OK) {
		printf("#max31865_readCFG error\r\n");
	}

	return *(uint32_t*)_spi_transaction2.rx_data;
}

uint32_t max31865_readRTD() {
	// == SET UP STUFF ===
	esp_err_t _result;
	spi_transaction_t _spi_transaction1;
	spi_transaction_t _spi_transaction2;
	memset(&_spi_transaction1, 0, sizeof(_spi_transaction1));		// init with zeros
	memset(&_spi_transaction2, 0, sizeof(_spi_transaction2));		// init with zeros

	// === TRANSMIT ===
	_spi_transaction1.length = 		8 * 1;                     		// len is in BITS
	_spi_transaction1.flags = 		SPI_TRANS_USE_TXDATA;			// use internal 4 byte buf
	_spi_transaction1.tx_data[0] = 	0x01;							// start at address of RTD

	_result = spi_device_transmit(spi, &_spi_transaction1);  		// Transmit!
	assert( _result == ESP_OK );            						// Should have had no issues.

	// === RECEIVE ===
	_spi_transaction2.length = 		8 * 2;							// number of bytes that we intent to receive
	_spi_transaction2.flags = SPI_TRANS_USE_RXDATA;

	_result = spi_device_transmit(spi, &_spi_transaction2);
	assert( _result == ESP_OK );

	// === RETURN RESULT ===
	if(_result!=ESP_OK) {
		printf("#max31865_readCFG error\r\n");
	}

	return *(uint32_t*)_spi_transaction2.rx_data;
}

esp_err_t max31865_writeCFG() {
	// == SET UP STUFF ===
	esp_err_t _result;
	spi_transaction_t _spi_transaction1;
	memset(&_spi_transaction1, 0, sizeof(_spi_transaction1));		// init with zeros

	// === TRANSMIT ===
	_spi_transaction1.length = 		8 * 2;                     		// Command is 8 bits. len is in BITS
	_spi_transaction1.flags = 		SPI_TRANS_USE_TXDATA;			// use internal 4 byte buf
	_spi_transaction1.tx_data[0] = 	0x80;							// register address: write config
	_spi_transaction1.tx_data[1] = 	0xC3;							// data: bias, continuous mode, clear errors,

	_result = spi_device_transmit(spi, &_spi_transaction1);  		// Transmit!
	assert( _result == ESP_OK );           							// Should have had no issues.

	// DEBUG
	if(_result != ESP_OK) {
		printf("#max31865_writeCFG error\r\n");
	}
	return _result;
}

uint32_t lastChangeAt = -1;
void handleTemperature(float temperature) {
	if (temperature < TEMPERATURE_DESIRED) {
		if (lastChangeAt == -1 || temperatureCheck_tick_counter - lastChangeAt > 5) {
			// SWITCH ON
			printf("ON\r\n");
			led_blinkmode = BLINK_FAST;
			relais_mode = ON;
			lastChangeAt = temperatureCheck_tick_counter;
		} else {
			// no switching, yet. avoid constant relais toggle
		}
	} else {
		printf("OFF\r\n");
		led_blinkmode = BLINK_SLOW;
		relais_mode = OFF;
	}

	// === finally, increase "TICK COUNTER"
	++temperatureCheck_tick_counter;
}

static float a = 0.00390830;
static float b = -0.0000005775;
static float c = -0.00000000000418301;
float Reference_Resistor = 4300.;           // Reference Resistor installed on the board.
float RTD_Resistance = 1000.;				// PT1000 has 1kOhm @ 0deg celsius
float calculate_temp(uint32_t rtd)
{
	// Hani conversion
	float RTD = (float) rtd;
	float R = (RTD * Reference_Resistor) / 32768; // Conversion of ADC RTD code to resistance
	float Temp = -RTD_Resistance * a
			+ sqrt(
					RTD_Resistance * RTD_Resistance * a * a
							- 4 * RTD_Resistance * b * (RTD_Resistance - R)); //Conversion of RTD resistance to Temperature
	Temp = Temp / (2 * RTD_Resistance * b);
	return Temp;
}

void blink_task(void *pvParameter)
{
	gpio_set_level(BLINK_GPIO, 0);
	vTaskDelay(xDelay);
	gpio_set_level(BLINK_GPIO, 1);
	vTaskDelay(xDelay);
	gpio_set_level(BLINK_GPIO, 0);
	vTaskDelay(xDelay);
	gpio_set_level(BLINK_GPIO, 1);
	vTaskDelay(xDelay);
	gpio_set_level(BLINK_GPIO, 0);
	vTaskDelay(xDelay);
	gpio_set_level(BLINK_GPIO, 1);
	vTaskDelay(xDelay);

    while(1) {
    	printf("#blink_task::tick()\r\n");
    	// === LED ===
    	switch(led_blinkmode) {
		case OFF:
			gpio_set_level(BLINK_GPIO, 0);
			vTaskDelay(1000 / portTICK_PERIOD_MS);	// delay in this mode, too.
			break;
		case ON:
			gpio_set_level(BLINK_GPIO, 1);
			vTaskDelay(1000 / portTICK_PERIOD_MS);	// delay in this mode, too.
			break;
		case BLINK_SLOW:
			gpio_set_level(BLINK_GPIO, 0);
			vTaskDelay(600 / portTICK_PERIOD_MS);
			gpio_set_level(BLINK_GPIO, 1);
			vTaskDelay(600 / portTICK_PERIOD_MS);
			break;
		case BLINK_FAST:
			gpio_set_level(BLINK_GPIO, 0);
			vTaskDelay( xDelay );
			gpio_set_level(BLINK_GPIO, 1);
			vTaskDelay( xDelay );
			break;
		default:
			printf("#unknown state\r\n");
			vTaskDelay(1000 / portTICK_PERIOD_MS);
			break;
		}
    }
}

void relais_task(void *pvParameter)
{
	gpio_set_level(RELAIS_GPIO, 0);

    while(1) {
    	printf("#relais_task::tick()\r\n");
    	vTaskDelay(1000 / portTICK_PERIOD_MS);	// delay in this mode, too.
    	// === RELAIS ===
    	switch(relais_mode) {
		case ON:
			gpio_set_level(RELAIS_GPIO, 1);
			break;
		default:
			gpio_set_level(RELAIS_GPIO, 0);
			break;
		}
    }
}

void temperature_check_task(void *pvParameter)
{
	printf("Writing MAX31865 Config...\r\n");
	max31865_writeCFG();						// continuous mode.
	printf("...done.\r\n");

    while(1) {
		printf("#temperature_check_task::tick()\r\n");
		vTaskDelay(1000 / portTICK_PERIOD_MS);

		// MAX31865
		uint32_t _rtd = max31865_readRTD();
		if (_rtd & 0x00000001) {
			SSD1306_GotoXY(4, 20);
			SSD1306_Puts("T=ERROR  ", &Font_7x10, SSD1306_COLOR_WHITE);

			printf("ERROR\r\n");
		} else {
			uint32_t rtd = _rtd >> 1;

			////////////////////////////// V3 :: WORKING!
			float calculated_temp_2 = calculate_temp(rtd);
			printf("OK. RTD= %d, temp: %.1f\r\n", rtd, calculated_temp_2);

			// OUTPUT :: LINE 1
			sprintf(buf, "TdegC: %.0f    ", calculated_temp_2);
			SSD1306_GotoXY(4, 20);
			SSD1306_Puts(buf, &Font_7x10, SSD1306_COLOR_WHITE);

			handleTemperature(calculated_temp_2);

			// OUTPUT :: LINE 2
			sprintf(buf, "led=%d    ", led_blinkmode);
			SSD1306_GotoXY(4, 35);
			SSD1306_Puts(buf, &Font_7x10, SSD1306_COLOR_WHITE);

//			// OUTPUT :: LINE 3
			sprintf(buf, "relais=%d   ", relais_mode);
			SSD1306_GotoXY(4, 47);
			SSD1306_Puts(buf, &Font_7x10, SSD1306_COLOR_WHITE);
		}
		SSD1306_UpdateScreen();
    }
}


void app_main()
{
	// === I/O ===
	gpio_pad_select_gpio(BLINK_GPIO);
	gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
	gpio_set_level(BLINK_GPIO, 0);

	gpio_pad_select_gpio(RELAIS_GPIO);
	gpio_set_direction(RELAIS_GPIO, GPIO_MODE_OUTPUT);
	gpio_set_level(RELAIS_GPIO, 0);


	// === I2C ===
	i2c_example_master_init();

	// === SSD1306 Display ===
	SSD1306_Init();
	SSD1306_Fill(SSD1306_COLOR_BLACK); // clear screen
	SSD1306_GotoXY(4, 4);
	SSD1306_Puts("B04rdBr4T!", &Font_11x18, SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();


	// === SPI ===
	spi_master_init();
	// MAX 31865 'ready' indication
	gpio_pad_select_gpio(SPI_MASTER_RDY);
	gpio_set_direction(SPI_MASTER_RDY, GPIO_MODE_INPUT);


	// === lwIP ===

	// === WiFi ===
	//Initialize NVS
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

#if EXAMPLE_ESP_WIFI_MODE_AP
	ESP_LOGI(LOGTAG, "ESP_WIFI_MODE_AP");
	wifi_init_softap();
#else
	ESP_LOGI(LOGTAG, "ESP_WIFI_MODE_STA");
	wifi_init_sta();
#endif /*EXAMPLE_ESP_WIFI_MODE_AP*/


    // === TASKS ===
	TaskHandle_t* p_blinkTask = malloc(sizeof(TaskHandle_t));
	TaskHandle_t* p_relaisTask = malloc(sizeof(TaskHandle_t));
	TaskHandle_t* p_tempCheckTask = malloc(sizeof(TaskHandle_t));
	xTaskCreate(&blink_task, "blink_task", CONFIG_FREERTOS_IDLE_TASK_STACKSIZE * 8, NULL, tskIDLE_PRIORITY, p_blinkTask);
	xTaskCreate(&relais_task, "relais_task", CONFIG_FREERTOS_IDLE_TASK_STACKSIZE * 8, NULL, tskIDLE_PRIORITY, p_relaisTask);
	xTaskCreate(&temperature_check_task, "tempChkTask", 	CONFIG_FREERTOS_IDLE_TASK_STACKSIZE * 8, NULL, tskIDLE_PRIORITY, p_tempCheckTask);

}
