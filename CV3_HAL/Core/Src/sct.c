/*
 * sct.c
 *
 *  Created on: 16. 10. 2020
 *      Author: Pavel
 */

#include "main.h"
#include "STM32F0xx.h"

#include "sct.h"


void sct_led(uint32_t value)
{
	for (uint8_t j = 0; j<32; j++)
	{
		HAL_GPIO_WritePin(GPIOB,SCT_SDI_Pin,value&1);
		value=value>>1;
		HAL_GPIO_WritePin(GPIOB, SCT_CLK_Pin, 1);
		HAL_GPIO_WritePin(GPIOB, SCT_CLK_Pin, 0);
	}
	HAL_GPIO_WritePin(GPIOB, SCT_NLA_Pin, 1);
	HAL_GPIO_WritePin(GPIOB, SCT_NLA_Pin, 0);
}

void sct_init(void)
{
	sct_led(0);
}

void sct_value(uint16_t value)
{
	value = value%1000;
	uint32_t out = reg_values[2][value%10];
	value = value /10;
	if(value>0)
	{
		out |= reg_values[1][value%10];
		value = value /10;
		if(value>0)
		{
			out |= reg_values[0][value%10];
		}
	}
	sct_led(out);
}
