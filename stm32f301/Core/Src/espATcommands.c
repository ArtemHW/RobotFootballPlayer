/*
 * espATcommands.c
 *
 *  Created on: Feb 5, 2024
 *      Author: Artem Kagirov
 */

#include <espATcommands.h>


void sendATCommand(UART_HandleTypeDef *huart, const char* atcom, int sizeOfAT, char* response, int sizeOfResp, int delayms)
{
//	if(atcom[sizeOfAT-1] == '\0') {
//		sizeOfAT -= 1;
//	}
	for(uint8_t i = 0; i < sizeOfAT; i++) {
		if(atcom[i] == '\0') {
			sizeOfAT = i + 1;
		}
	}
	for(uint8_t i = 0; i < sizeOfResp; i++) {
		response[i] = 0;
	}
	HAL_UART_Transmit(huart, atcom, sizeOfAT, delayms);
	HAL_UART_Receive(huart, response, sizeOfResp, delayms);
}
