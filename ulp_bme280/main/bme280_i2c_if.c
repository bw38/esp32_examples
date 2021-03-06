/*
 * bme280_i2c_if.c
 *
 *  Created on: 26.04.2019
 *      Author: joerg

  ESP32-I2C - Beispielcode:
  https://github.com/espressif/esp-idf/blob/a20d02b7f196c407bc9f39b781e31a0a4f665968/examples/peripherals/i2c/i2c_self_test/main/i2c_example_main.c
  Bosch-Treiber: file bme280.c /  2020-03-28 / v3.5.0
  https://github.com/BoschSensortec/BME280_driver
*/


#include "bme280_i2c_if.h"

#define DEBUG_I2C_NO

//interne Varibalen
RTC_DATA_ATTR static struct bme280_dev bme280_device;
RTC_DATA_ATTR static uint8_t dev_addr = 0;	//0x76 | 0x77
//struct bme280_data bme280_comp_data;
i2c_port_t i2c_mport;

RTC_DATA_ATTR static int8_t rtc_bme280_init_result;



//Hilfsfunktionen --------------------------------------------------
#ifdef DEBUG_I2C
static void print_now(){
	printf("[%.4d]", (int)esp_timer_get_time() / 1000);
}
#endif


// I2C - Anpassung an BME280.h --------------------------------

//only called once in coldstart (2ms)
void bme280_delay_10ms_i2c(uint32_t period, void *intf_ptr) {
#ifdef DEBUG_I2C
	print_now();
	printf("delay_us: %d\n", period);
#endif
	vTaskDelay(1);   //Tick-Rate in MenuConfig 10ms !!!
}

int8_t bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    if (len == 0) return ESP_OK;

    uint8_t dev_addr = *((uint8_t *)intf_ptr);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, reg_addr, 1);

    i2c_master_start(cmd);	//Repeated Start
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, 1);

    if (len > 1) i2c_master_read(cmd, reg_data, len - 1, 0);
    i2c_master_read_byte(cmd, reg_data + len - 1, 1);

    i2c_master_stop(cmd);
    int8_t res = i2c_master_cmd_begin(i2c_mport, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

#ifdef DEBUG_I2C
    print_now();
    printf("Rd** Dev: %.2X | Reg: %.2X | Len: %d |Data: ", dev_addr, reg_addr,  len);
	for (int i=0; i<len; i++) printf(" %.2X", reg_data[i]);
	printf("\n");
#endif
	return res;
}

int8_t bme280_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *((uint8_t *)intf_ptr);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, 1);
    for (int i=0; i<len; i++)
    {
    	i2c_master_write_byte(cmd, reg_addr + i, 1);
    	i2c_master_write_byte(cmd, reg_data[i], 1);
    }
    i2c_master_stop(cmd);
    int8_t ret = i2c_master_cmd_begin(i2c_mport, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

#ifdef DEBUG_I2C
    print_now();
	printf("Wr** Dev: %.2X | Reg: %.2X | Len: %d |Data: ", dev_addr, reg_addr,  len);
	for (int i=0; i<len; i++) printf(" %.2X", reg_data[i]);
	printf("\n");
#endif

    return ret;
}
// --------------------------------------------------------------------------------------------

//Schnittstelle initialisieren
//erforderlich nach jedem boot bzw wakeup
//wu == true -> wakeup aus Tiefschlaf
esp_err_t bme280_i2c_init(uint8_t i2c_addr, i2c_port_t _i2c_mport, bool wu)
{
	i2c_mport = _i2c_mport;
    //Chip wird nach Kaltstart oder vorherg. Fehler initialisiert
    if (! wu) rtc_bme280_init_result = -1;
	if (rtc_bme280_init_result != 0) {
		dev_addr = i2c_addr;
		bme280_device.intf_ptr = &dev_addr;
		bme280_device.intf  = BME280_I2C_INTF;
		bme280_device.read  = bme280_i2c_read;
		bme280_device.write = bme280_i2c_write;
		bme280_device.delay_us = bme280_delay_10ms_i2c;
		rtc_bme280_init_result = bme280_init(&bme280_device); // --> bme280.h

		#ifdef DEBUG_I2C
		printf("BME-Init: %d\n", rtc_bme280_init_result);
		#endif
	}
	return rtc_bme280_init_result;
}


esp_err_t bme280_i2c_get_result (struct bme280_uncomp_data *uncomp_data, struct bme280_data *comp_data) {

	int8_t err =
			bme280_compensate_data(BME280_ALL,
	                              uncomp_data,
	                              comp_data,
	                              &bme280_device.calib_data);
	 return err;
}

