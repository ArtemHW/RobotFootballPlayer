/*
 * espATcommands.c
 *
 *  Created on: Feb 5, 2024
 *      Author: Artem Kagirov
 */

#include <espATcommands.h>
extern uint32_t debugVar;

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

float parseFloat(const char *buffer, uint16_t startIndex)
{
    float value = 0.0f;
    uint8_t decimalFlag = 0;
    uint8_t decimalPlaces = 0;

    for (uint16_t i = startIndex; buffer[i % ESPRXBUFFERSIZE] != '_'; i++) {
        if (buffer[i % ESPRXBUFFERSIZE] == '.') {
            decimalFlag |= (1<<0);
        } else if (buffer[i % ESPRXBUFFERSIZE] == '-') {
            decimalFlag |= (1<<1);
        } else if (buffer[i % ESPRXBUFFERSIZE] >= '0' && buffer[i % ESPRXBUFFERSIZE] <= '9') {
            if ((decimalFlag & (1<<0)) == (1<<0)) {
                decimalPlaces++;
                value = value + (buffer[i % ESPRXBUFFERSIZE] - '0') / pow(10, decimalPlaces);
            } else {
                value = value * 10 + (buffer[i % ESPRXBUFFERSIZE] - '0');
            }
        }
        if(value > 50) {
        	__asm__ volatile("NOP");
        }
    }

    if((decimalFlag & (1<<1)) == (1<<1)) {
    	value *= -1.0f;
    }

    return value;
}

void espRecon(UART_HandleTypeDef *huart)
{
	char pData[100];
	debugVar = 122;

	sprintf(pData, "AT+CIPCLOSE\r\n");
	HAL_UART_Transmit(huart, pData, strlen(pData), 250);
	vTaskDelay(50);

	debugVar = 123;

	sprintf(pData, "AT+CIPSTART=\"TCP\",\"192.168.137.1\",8080\r\n");
	HAL_UART_Transmit(huart, pData, strlen(pData), 250);
	vTaskDelay(40);

	debugVar = 124;
	// Creating the entire GET request string
	sprintf(pData, "GET /robot HTTP/1.1\r\n"
					  "Host: 192.168.137.1\r\n");
	int getRequestLength = strlen(pData);
	char pDataBuf[20];
	sprintf(pDataBuf, "AT+CIPSEND=%d\r\n", getRequestLength);
	debugVar = 125;
	HAL_UART_Transmit(huart, pDataBuf, strlen(pDataBuf), 250);
	vTaskDelay(10);
	debugVar = 126;
	HAL_UART_Transmit(huart, pData, getRequestLength, 250);
	vTaskDelay(50);
}
