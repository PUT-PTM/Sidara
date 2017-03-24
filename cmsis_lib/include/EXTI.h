#ifndef EXTI_H_
#define EXTI_H_

void initNVIC(void);
void initUserButton(void);
void initEXTIUserButton(void);
//void EXTI0_IRQHandler(void);

void initTimer2ForDebouncing(void);

#endif
