/*
 * espATcommands.h
 *
 *  Created on: Feb 5, 2024
 *      Author: Artem Kagirov
 */

#ifndef INC_ESPATCOMMANDS_H_
#define INC_ESPATCOMMANDS_H_

#include "main.h"

//void sendATCommand(UART_HandleTypeDef *huart, const char* atcom, int sizeOfAT, char* response, int sizeOfResp, int delayms);
void sendATCommand(UART_HandleTypeDef *huart, const char* atcom, int sizeOfAT, int delayms);
void receiveAnswer(UART_HandleTypeDef *huart, char* response, int sizeOfResp, int delayms);
void atCipclose(UART_HandleTypeDef *huart, int delayms);
void atCipclose_DMA(UART_HandleTypeDef *huart);
uint8_t atSend_USART3_DMA(const uint8_t *pData, uint16_t amount);

#endif /* INC_ESPATCOMMANDS_H_ */
