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

#include "ssd1306.h"

#define BLINK_GPIO 16

/* according to "esp_wroom_32_datasheet_en.pdf" */
#define I2C_EXAMPLE_MASTER_PORT 			I2C_NUM_1
#define I2C_EXAMPLE_MASTER_SCL_IO    		4    		/*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO    		5    		/*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0   		/*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0   		/*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_FREQ_HZ    		100000     	/*!< I2C master clock frequency */

/* We are using VSPI */
#define SPI_MASTER_MISO_SDO 	19
#define SPI_MASTER_MOSI_SDI		23
#define SPI_MASTER_CLK			18
#define SPI_MASTER_CS			17
#define SPI_MASTER_RDY			21

uint8_t cntr = 0;

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
	spi_device_handle_t spi;
	spi_bus_config_t buscfg={
		.miso_io_num=PIN_NUM_MISO,
		.mosi_io_num=PIN_NUM_MOSI,
		.sclk_io_num=PIN_NUM_CLK,
		.quadwp_io_num=-1,
		.quadhd_io_num=-1,
		.max_transfer_sz=PARALLEL_LINES*320*2+8
	};
	spi_device_interface_config_t devcfg={
		.clock_speed_hz=10*1000*1000,           //Clock out at 10 MHz
		.mode=0,                                //SPI mode 0
		.spics_io_num=PIN_NUM_CS,               //CS pin
		.queue_size=7,                          //We want to be able to queue 7 transactions at a time
		.pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
	};
	//Initialize the SPI bus
	ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
	ESP_ERROR_CHECK(ret);
	//Attach the LCD to the SPI bus
	ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
	ESP_ERROR_CHECK(ret);
	//Initialize the LCD
	lcd_init(spi);
	//Initialize the effect displayed
	ret=pretty_effect_init();
	ESP_ERROR_CHECK(ret);
}

void blink_task(void *pvParameter)
{
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    gpio_pad_select_gpio(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    while(1) {
        /* Blink off (output low) */
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        printf("#cntr: %d\r\n", ++cntr);

        SSD1306_GotoXY(40, 20);
        char buf[64];
        sprintf(buf, "%04d", cntr);
		SSD1306_Puts(buf, &Font_11x18, SSD1306_COLOR_WHITE);
		SSD1306_UpdateScreen();
    }
}

void app_main()
{
	printf("#1.1\r\n");
	i2c_example_master_init();

	printf("#1.2\r\n");
	SSD1306_Init();
	SSD1306_Fill(SSD1306_COLOR_BLACK); // clear screen
	SSD1306_GotoXY(40, 4);
	SSD1306_Puts("BIER!", &Font_11x18, SSD1306_COLOR_WHITE);
	SSD1306_GotoXY(2, 20);
	SSD1306_UpdateScreen();

	printf("#2\r\n");

    xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);

}
