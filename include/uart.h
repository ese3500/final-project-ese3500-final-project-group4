#ifndef UART_H
#define UART_H

static void UART_init(int prescale);

static void UART_send( unsigned char data);

static void UART_putstring(char* StringPtr);

#endif 