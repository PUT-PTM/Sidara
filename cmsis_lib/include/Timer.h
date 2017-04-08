#ifndef Timer_H_
#define Timer_H_

void initTimer2(int period, int prescaler);
void initTimer3(int period, int prescaler);
void initTimer4(int period, int prescaler);
void initTimer5(int period, int prescaler);
void initTimer7(int period, int prescaler);

void initTimer2Interruption();
void initTimer3Interruption();
void initTimer4Interruption();
void initTimer5Interruption();
void initTimer7Interruption();

void initTimer2_5HZ(void);
void initTimer2_2HZ(void);
void initTimer2Interrupt(void);
void initTimer3_1HZ(void);

void initTimer2For30msDelay();

//void TIM2_IRQHandler(void);

#endif
