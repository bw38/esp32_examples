/*
 * interface.h
 *
 *  Created on: 09.12.2021
 *      Author: joerg
 */

#ifndef MAIN_INTERFACE_H_
#define MAIN_INTERFACE_H_

//I2C-Addr BME280, Manufactorer-Addr
#define DEV_ADDR	0x76

//Ports in GPIO-Matrix
#define GPIO_SCL	27
#define GPIO_SDA	26
//corresponding RTX-IO
#define	RTC_IO_SCL	17
#define RTC_IO_SDA	7

//Clockspeed I2C-IF in main
#define	I2C_MASTER_FREQ_HZ	50000



#endif /* MAIN_INTERFACE_H_ */
