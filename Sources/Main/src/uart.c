//  UART
//
//  Использованные аппаратные ресурсы:
//  USART2
//  GPIOA.2 - Tx
//  GPIOA.3 - Rx

#include "stm32f10x.h"


#define UART_USART            (USART2)
#define UART_Pin_Tx           (GPIO_Pin_2)
#define UART_Pin_Rx           (GPIO_Pin_3)
#define UART_GPIO             (GPIOA)

#define UART_BR               (115200)


void UART_Init (void) {
  USART_InitTypeDef USART_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOA, ENABLE);                        //Включаем тактирование порта

	GPIO_InitStruct.GPIO_Pin = UART_Pin_Tx;                                       //Линии UART
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;                                 //Максимальная скорость
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;                                  //Альтернативная функция пинов
	GPIO_Init (UART_GPIO, &GPIO_InitStruct);                                      //Инициализируем линии UART

	GPIO_InitStruct.GPIO_Pin = UART_Pin_Rx;                                       //Линии UART
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;                                 //Максимальная скорость
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;                                  //Альтернативная функция пинов
	GPIO_Init (UART_GPIO, &GPIO_InitStruct);                                      //Инициализируем линии UART
  
  RCC_APB1PeriphClockCmd (RCC_APB1Periph_USART2, ENABLE);
  
  USART_InitStruct.USART_BaudRate = UART_BR;
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART_InitStruct.USART_Parity = USART_Parity_No;
  USART_InitStruct.USART_StopBits = USART_StopBits_1;
  USART_InitStruct.USART_WordLength = USART_WordLength_8b;
  USART_Init (UART_USART, &USART_InitStruct);
  
  USART_Cmd (UART_USART, ENABLE);
  
}


void UART_Tx (uint8_t b) {
  USART_SendData (UART_USART, b);
  while (USART_GetFlagStatus (UART_USART, USART_FLAG_TC) == RESET);
}

uint8_t UART_IsRx (void) {
  return (USART_GetFlagStatus (UART_USART, USART_FLAG_RXNE) != RESET);
}

uint8_t UART_Rx (void) {
  uint16_t i = 50000;  
  while (USART_GetFlagStatus (UART_USART, USART_FLAG_RXNE) == RESET)
    if (!i--) break;
  return USART_ReceiveData (UART_USART);
}




