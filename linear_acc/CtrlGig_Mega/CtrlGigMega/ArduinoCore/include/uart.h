/*
 * uart.h
 *
 * Created: 28/02/2018 21:45:35
 *  Author: felip
 */ 


#ifndef UART_H_
#define UART_H_


void treatSerialIncome(void);
void readNewSetCommands(void);
uint8_t processCommands(s_motor_ctrl *s);
#endif /* UART_H_ */