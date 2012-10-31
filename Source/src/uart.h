#ifndef __UART_H
#define __UART_H

extern unsigned char Rxbuf;
extern unsigned char time_no_data;

void InitUsart0(void);
__interrupt void RX_USART0(void);

#endif // __UART_H
