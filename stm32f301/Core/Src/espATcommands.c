/*
 * espATcommands.c
 *
 *  Created on: Feb 5, 2024
 *      Author: Artem Kagirov
 */

#include <espATcommands.h>


//void sendATCommand(UART_HandleTypeDef *huart, const char* atcom, int sizeOfAT, char* response, int sizeOfResp, int delayms)
//{
////	if(atcom[sizeOfAT-1] == '\0') {
////		sizeOfAT -= 1;
////	}
//	for(uint8_t i = 0; i < sizeOfAT; i++) {
//		if(atcom[i] == '\0') {
//			sizeOfAT = i;
//		}
//	}
//	for(uint8_t i = 0; i < sizeOfResp; i++) {
//		response[i] = 0;
//	}
//	HAL_UART_Transmit(huart, atcom, sizeOfAT, delayms);
//	HAL_UART_Receive(huart, response, sizeOfResp, delayms);
//}

void sendATCommand(UART_HandleTypeDef *huart, const char* atcom, int sizeOfAT, int delayms)
{
	for(uint16_t i = 0; i < sizeOfAT; i++) {
		if(atcom[i] == '\0') {
			sizeOfAT = i;
		}
	}
	HAL_UART_Transmit(huart, atcom, sizeOfAT, delayms);
}

void receiveAnswer(UART_HandleTypeDef *huart, char* response, int sizeOfResp, int delayms)
{
	for(uint16_t i = 0; i < sizeOfResp; i++) {
		response[i] = 0;
	}
	HAL_UART_Receive(huart, response, sizeOfResp, delayms);
}

void atCipclose(UART_HandleTypeDef *huart, int delayms)
{
	char pData[] = "AT+CIPCLOSE\r\n";
	HAL_UART_Transmit(huart, pData, sizeof(pData)-1, delayms);
}

void atCipclose_DMA(UART_HandleTypeDef *huart)
{
	char pData[] = "AT+CIPCLOSE\r\n";
	HAL_StatusTypeDef resp;
	resp = HAL_UART_Transmit_DMA(huart, (uint8_t*)pData, sizeof(pData)-1);
	__asm__ volatile("NOP");
}

uint8_t atSend_USART3_DMA(const uint8_t *pData, uint16_t amount)
{
	if((DMA1_Channel2->CCR & DMA_CCR_EN) == DMA_CCR_EN) {
		if((DMA1_Channel2->CNDTR) != 0) {
			return 1;
		} else if((DMA1_Channel2->CCR & DMA_CCR_TCIE) != DMA_CCR_TCIE) {
			DMA1_Channel2->CCR &= ~DMA_CCR_EN;
			USART3->CR3 &= ~USART_CR3_DMAT;
			if((DMA1->ISR & DMA_ISR_TCIF2) == DMA_ISR_TCIF2) {
				DMA1->IFCR |= DMA_IFCR_CTCIF2;
			}
			DMA1_Channel2->CCR |= DMA_CCR_TCIE;
		}
	}
	USART3->CR3 |= USART_CR3_DMAT;

	DMA1_Channel2->CNDTR = amount;
	DMA1_Channel2->CPAR = (uint32_t)&(USART3->TDR);
	DMA1_Channel2->CMAR = (uint32_t)pData;

	DMA1_Channel2->CCR |= DMA_CCR_EN;
	return 0;
}
