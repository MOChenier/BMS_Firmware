/*
 * misc.h
 *
 *  Created on: Feb 5, 2025
 *      Author: marco
 */

#ifndef INC_MISC_H_
#define INC_MISC_H_

#include "stm32f4xx_hal.h"

#define PRECHARGE_PIN_GROUP			GPIOC
#define PRECHARGE_PIN				GPIO_PIN_0

#define MAIN_CONTACT_PIN_GROUP		GPIOC
#define MAIN_CONTACT_PIN			GPIO_PIN_1

#define CHARGER_DETECT_PIN_GROUP	GPIOC
#define CHARGER_DETECT_PIN			GPIO_PIN_3

#define EMERGENCY_STOP_PIN_GROUP	GPIOC
#define EMERGENCY_STOP_PIN			GPIO_PIN_4

#define IGNITION_PIN_GROUP			GPIOC
#define IGNITION_PIN				GPIO_PIN_5

#define YELLOW_LED_PIN_GROUP		GPIOB
#define YELLOW_LED_PIN				GPIO_PIN_14

#define RED_LED_PIN_GROUP			GPIOB
#define RED_LED_PIN					GPIO_PIN_15

int check_if_ignition_ON(void);
int check_if_charger_present(void);

void deactivate_main_contactor(void);
void activate_main_contactor(void);
void deactivate_precharge_contactor(void);
void activate_precharge_contactor(void);




#endif /* INC_MISC_H_ */
