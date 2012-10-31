#ifndef __ADC_H
#define __ADC_H

#define ADC_IGNITION  0
#define ADC_LIGHT     1
#define ADC_P         5


extern unsigned int adc_value[6];

void InitADC (void);
void ADC_IRQ(void);

#endif // __ADC_H