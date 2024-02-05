/*
 * espATcommands.h
 *
 *  Created on: Feb 5, 2024
 *      Author: Artem Kagirov
 */

#ifndef INC_ESPATCOMMANDS_H_
#define INC_ESPATCOMMANDS_H_

#include "main.h"

void sendATCommand(UART_HandleTypeDef *huart, const char* atcom, int sizeOfAT, char* response, int sizeOfResp, int delayms);


#endif /* INC_ESPATCOMMANDS_H_ */
