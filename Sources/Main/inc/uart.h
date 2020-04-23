#ifndef __UART_H
#define __UART_H

void UART_Init (void);
void UART_Tx (uint8_t b);
uint8_t UART_IsRx (void);
uint8_t UART_Rx (void);



#endif
