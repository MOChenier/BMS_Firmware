/*
 * retarget.c
 *
 *  Created on: Feb 25, 2025
 *      Author: marco
 */


#include "stdio.h"
#include "stdint.h"
#include "stm32f4xx.h"

int __io_putchar(int ch)
{
 // Write character to ITM ch.0
 ITM_SendChar(ch);
 return(ch);
}

