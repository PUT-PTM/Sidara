#ifndef DAC_H_
#define DAC_H_

void initDAC(void);
void sendDAC(int number);
float nDigital_to_Voltage(int ADC_Result);
float getSine(float Amplituda, float deltaalfa);

#endif;
