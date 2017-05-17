#ifndef USART_H_
#define USART_H_

void initUSART(void);
char readUSART(void);
void initUSARTInterruption(void);
char readUSARTFromInterruption(void);

#endif;
