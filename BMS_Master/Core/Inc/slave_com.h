/*
 * slave.h
 *
 *  Created on: Mar 24, 2025
 *      Author: marco
 */

#ifndef INC_SLAVE_COM_H_
#define INC_SLAVE_COM_H_

#include "can_bus.h"

// Messages to send to slave
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

// Slave return IDs. Should be returned in the slave's message's Std_ID
#define RET_VOLTAGE_INFO_STDID 0x20A
#define RET_TEMP_INFO_STDID 0x206
#define RET_BALANC_INFO_STDID 0x20B

#define SLAVE_ID_MASK 0x0F0
#define INFO_ID_MASK 0xF0F

// Info for slave CAN communication
#define SLAVE_BYTES_NUM 2
#define SLAVE_HEADER 0x111

void init_slave_info_lists();

int store_received_info(CAN_RxHeaderTypeDef* RxHeader, uint8_t RxData[8]);

int all_voltages_under_threshold();
int all_temps_under_threshold();
int all_cells_balanced();
float get_pack_voltage(uint8_t pack_id);
int all_balancing_charge_present();


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


#endif /* INC_SLAVE_COM_H_ */
