/*
 * IncFile1.h
 *
 * Created: 26-03-2017 22:00:21
 *  Author: N
 */ 


#ifndef STDIO_UART_H_
#define STDIO_UART_H_

void ioinit(void);      // initializes IO - call in main!
int uart_putchar(char c, FILE *stream); //UART to stdio (output) function
int uart_getchar(FILE *stream); //UART to stdio (input) function

#endif /* STDIO_UART_H_ */