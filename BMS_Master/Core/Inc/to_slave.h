/*
 * slave.h
 *
 *  Created on: Mar 24, 2025
 *      Author: marco
 */

#ifndef INC_TO_SLAVE_H_
#define INC_TO_SLAVE_H_

#include "can_bus.h"

#define VOLTAGE_INFO 0xA1
#define AUTO_VOLTAGE_INFO 0xAA
#define STOP_VOLTAGE_INFO 0xA0

#define TEMP_INFO 0x61
#define AUTO_TEMP_INFO 0x66
#define STOP_TEMP_INFO 0x60

#define BALANCING_INFO 		0xB1
#define AUTO_BALANCING_INFO 0xBB
#define STOP_BALANCING_INFO 0xB0
#define START_BALANCING		0xBF
#define STOP_BALANCING		0xBA

#define READY_TO_CHARGE		0xCC

// Info for slave CAN communication
#define SLAVE_BYTES_NUM 2
#define SLAVE_HEADER 0x111

void ask_for_voltages();
void auto_ask_for_voltages(uint8_t delay_10_ms);
void stop_auto_ask_for_voltages();

void ask_for_temperatures();
void auto_ask_for_temperatures(uint8_t delay_10_ms);
void stop_auto_ask_for_temperatures();

void ask_for_balancing_info();
void auto_ask_for_balancing_info(uint8_t delay_10_ms);
void stop_auto_ask_for_balancing_info();
void start_balancing();
void stop_balancing();


#endif /* INC_TO_SLAVE_H_ */
