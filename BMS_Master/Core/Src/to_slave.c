#include "to_slave.h"

extern CAN_HandleTypeDef hcan2;

void ask_for_voltages()
{

	uint8_t message[2] = {0};

	message[0] = VOLTAGE_INFO;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN1_send_mess(&hcan2, message);

	uint32_t default_btr = CAN2->BTR;


}

void auto_ask_for_voltages(uint8_t delay_10_ms)
{

	uint8_t message[2] = {0};

	message[0] = AUTO_VOLTAGE_INFO;
	message[1] = delay_10_ms;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN1_send_mess(&hcan2, message);

}

void stop_auto_ask_for_voltages()
{

	uint8_t message[2] = {0};

	message[0] = STOP_VOLTAGE_INFO;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN1_send_mess(&hcan2, message);

}

void ask_for_temperatures()
{

	uint8_t message[2] = {0};

	message[0] = TEMP_INFO;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN1_send_mess(&hcan2, message);

}

void auto_ask_for_temperatures(uint8_t delay_10_ms)
{

	uint8_t message[2] = {0};

	message[0] = AUTO_TEMP_INFO;
	message[1] = delay_10_ms;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN1_send_mess(&hcan2, message);

}

void stop_auto_ask_for_temperatures()
{

	uint8_t message[2] = {0};

	message[0] = STOP_TEMP_INFO;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN1_send_mess(&hcan2, message);

}

void ask_for_balancing_info()
{

	uint8_t message[2] = {0};

	message[0] = BALANCING_INFO;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN1_send_mess(&hcan2, message);

}

void auto_ask_for_balancing_info(uint8_t delay_10_ms)
{

	uint8_t message[2] = {0};

	message[0] = AUTO_BALANCING_INFO;
	message[1] = delay_10_ms;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN1_send_mess(&hcan2, message);

}

void stop_auto_ask_for_balancing_info()
{

	uint8_t message[2] = {0};

	message[0] = STOP_BALANCING_INFO;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN1_send_mess(&hcan2, message);

}

void start_balancing()

{
	uint8_t message[2] = {0};

	message[0] = START_BALANCING;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN1_send_mess(&hcan2, message);

}

void stop_balancing()

{
	uint8_t message[2] = {0};

	message[0] = STOP_BALANCING;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN1_send_mess(&hcan2, message);

}

void get_ready_for_charging()
{
	uint8_t message[2] = {0};

	message[0] = READY_TO_CHARGE;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN1_send_mess(&hcan2, message);

}

