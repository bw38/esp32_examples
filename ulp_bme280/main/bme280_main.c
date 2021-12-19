/* ULP Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp32/ulp.h"
#include "ulp_main.h"

#include "interface.h"
#include "bme280_i2c_if.h"

// Deviation from the last measured value (+/-)
#define	DPRESS	50		// 50Pa = 0.5hPa
#define DTEMP	0.25	// 0.25 °C
#define DHUM	2		// 2%

#define CNT_WAKEUP	10

RTC_DATA_ATTR uint32_t cycles = 0;

i2c_port_t i2c_mport = -1;

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");


static void init_ulp_program(void)
{
    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
            (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);



    /* Set ULP wake up period to 20ms */
    ulp_set_wakeup_period(0, 2* 1000*1000);

    /* Disconnect GPIO12 and GPIO15 to remove current drain through
     * pullup/pulldown resistors.
     * GPIO12 may be pulled high to select flash voltage.
     */
    rtc_gpio_isolate(GPIO_NUM_12);
    rtc_gpio_isolate(GPIO_NUM_15);
    esp_deep_sleep_disable_rom_logging(); // suppress boot messages
}

static void start_ulp_program(void)
{
    /* Start the program */
    esp_err_t err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}


void rtc_io_init(){
	rtc_gpio_init(GPIO_SCL);
	rtc_gpio_set_direction(GPIO_SCL, RTC_GPIO_MODE_OUTPUT_ONLY);
	rtc_gpio_pullup_dis(GPIO_SCL);
	rtc_gpio_pulldown_dis(GPIO_SCL);

	rtc_gpio_init(GPIO_SDA);
	rtc_gpio_set_direction(GPIO_SDA, RTC_GPIO_MODE_INPUT_ONLY);
	rtc_gpio_pullup_dis(GPIO_SDA);
	rtc_gpio_pulldown_dis(GPIO_SDA);
	rtc_gpio_set_level(GPIO_SDA, 0);
}


i2c_port_t i2c_master_init()
{
	//I2C-Schnittstelle initialisieren
	i2c_port_t mport = I2C_NUM_0;
    i2c_config_t conf;
    conf.clk_flags = 0; // ab IDF > v4.1
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = GPIO_SDA;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;	//10k pullup auf Sensorboard
    conf.scl_io_num = GPIO_SCL;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;	//10k Pullup auf Sensorboard
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(mport, &conf);
    if (i2c_driver_install(mport, conf.mode, 0, 0, 0) == 0) {
    	printf("I2C Masterinit Ok\n");
    } else {
    	i2c_mport = -1;
    	printf("I2C Masterinit failed\n");
    }
	return mport;
}

// Wakeup-Stub - run after wake before boot
void RTC_IRAM_ATTR esp_wake_deep_sleep(void) {
 	//Extra Delay in Wakeup Stub -> s. SDK-Config/ESP32-specific 0..5000µs
 	esp_default_wake_deep_sleep();
 	cycles++;
}

void app_main(void)
{
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause != ESP_SLEEP_WAKEUP_ULP) {
        printf("\n----------Cold-Start -------------\n");

        i2c_mport = i2c_master_init();
        if (i2c_mport >= 0) {
        	if (bme280_i2c_init(DEV_ADDR, i2c_mport, false) == ESP_OK) {
        		printf("BME280 Init Ok\n");
        		rtc_io_init();
        		init_ulp_program();

        		//individual config for weather-monitoring, s. datasheet
        		ulp_ctrl_hum = BME280_OVERSAMPLING_1X;			//oversampling humidity
        		ulp_ctrl_meas = (BME280_OVERSAMPLING_1X << 5) | //oversampling temperature
        						(BME280_OVERSAMPLING_1X << 2) | //oversampling pressure
								BME280_FORCED_MODE;				//mode
        		ulp_config = 	BME280_FILTER_COEFF_OFF << 2;	//filter

        		start_ulp_program();
        	} else {
        		printf("Communication Error with BME280\n");
        	}
        }
    } else {
        printf("ULP wakeup reason: %d\nmain cycles: %d\n", ulp_wakeup_reason & 0xFFFF, cycles);
        ulp_cnt_force_wakeup = CNT_WAKEUP;
        printf("ulp cycles : %d\n", ulp_cycles & 0xFFFF);

        if ((ulp_com_err & 0xFFFF) != 0) {
        	printf("ULP-I2C-Communication Error - Restart in 5sek\n");
        	vTaskDelay(5000 / portTICK_PERIOD_MS);
        	esp_restart();
        }
        struct bme280_uncomp_data uncomp_data = { 0 };
        uint8_t mreg[8];
        uint32_t* pmres = &ulp_mraw;

        printf("raw data  =>    ");
        for (int i = 0; i<8; i++){
        	mreg[i] = (uint8_t) *pmres;
        	pmres++;
        	printf("0x%.2X ", mreg[i]);
        	if ((i == 2) || (i == 5)) printf("| ");
        }
        printf("\n");
        bme280_parse_sensor_data(mreg, &uncomp_data);

        struct bme280_data comp_data;
        bme280_i2c_get_result(&uncomp_data, &comp_data);
        printf("comp data => Press: %7.2fhPa |  Temp: %5.2f°C | Hum: %.2f%%\n",
        		comp_data.pressure / 100.0, comp_data.temperature, comp_data.humidity);

        //Offset-Calc for next wakeup (sleep-window)
        //two-point calculation
        //1.point = last result
        //2.point = last result + offset
        struct bme280_data comp_data2;
        const uint16_t moffs = 0x1000;			//fix offset for delta-calc
        uncomp_data.pressure    += moffs;
        uncomp_data.humidity    += moffs;
        bme280_i2c_get_result(&uncomp_data, &comp_data2);
        double dpress = (comp_data2.pressure - comp_data.pressure) / moffs;
        double dhum   = (comp_data2.humidity - comp_data.humidity) / moffs;
        uncomp_data.temperature += moffs*16;
        bme280_i2c_get_result(&uncomp_data, &comp_data2);
        double dtemp  = (comp_data2.temperature - comp_data.temperature) / moffs;

        bme280_parse_sensor_data(mreg, &uncomp_data);
        //write msb:lsb - values to ulp-variables
        uint32_t* plimit = &ulp_limit;
        *plimit = (uint32_t)(uncomp_data.pressure -    (round((DPRESS*16)/dpress))) >> 4; plimit++;
        *plimit = (uint32_t)(uncomp_data.pressure +    (round((DPRESS*16)/dpress))) >> 4; plimit++;
        *plimit = (uint32_t)(uncomp_data.temperature - (round((DTEMP*16)/dtemp))) >> 4; plimit++;
        *plimit = (uint32_t)(uncomp_data.temperature + (round((DTEMP*16)/dtemp))) >> 4; plimit++;
        *plimit = (uint32_t)(uncomp_data.humidity -    (round(DHUM/dhum))); plimit++;
        *plimit = (uint32_t)(uncomp_data.humidity +    (round(DHUM/dhum)));

        plimit = &ulp_limit;
        printf("Limit Press: 0x%.4X < > ", *plimit); plimit++;
        printf("0x%.4X\n",  *plimit);              plimit++;
        printf("Limit Temp:  0x%.4X < > ", *plimit); plimit++;
        printf("0x%.4X\n",  *plimit);              plimit++;
        printf("Limit Humi:  0x%.4X < > ", *plimit); plimit++;
        printf("0x%.4X\n",  *plimit);

    }


    printf("Entering deep sleep\n\n");

    ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );
    esp_deep_sleep_start();
}


