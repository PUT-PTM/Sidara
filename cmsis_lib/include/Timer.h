#ifndef Timer_H_
#define Timer_H_

void initTimer2(int period, int prescaler);
void initTimer3(int period, int prescaler);
void initTimer4(int period, int prescaler);
void initTimer5(int period, int prescaler);
void initTimer7(int period, int prescaler);

void initTimer2Interruption(void);
void initTimer3Interruption(void);
void initTimer4Interruption(void);
void initTimer5Interruption(void);
void initTimer7Interruption(void);

void initTimer2_5HZ(void);
void initTimer2_2HZ(void);
void initTimer2Interrupt(void);
void initTimer3_1HZ(void);

void initTimer5For30msDelay(void);
void delay30ms(void);

//void TIM2_IRQHandler(void);

#endif
