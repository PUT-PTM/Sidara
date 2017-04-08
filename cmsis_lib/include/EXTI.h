#ifndef EXTI_H_
#define EXTI_H_

void initNVICForEXTI(void);
void initUserButton(void);
void initEXTIUserButton(void);
//void EXTI0_IRQHandler(void);

void initTimer2ForDebouncing(void);

void initNVICForEXTI1(void);

#endif
