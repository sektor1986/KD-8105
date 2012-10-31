#include "mcu.h"
#include "adc.h"

unsigned int adc_value[6];

void InitADC (void)
{	

  ADCSL = 0xC0;	// stop mode, 10 bit
  ADCSH = 0xA0;
  
  ADTGCRH0_RLTE = 0;
  
  ADSR = 0x910D;	// 24 cy. sampling; 88 cy. conversion; channel 8-13 
  ADER1_ADE8 = 1;	// ADC_IGNITION		0
  ADER1_ADE9 = 1;	// ADC_LIGHT		1
  ADER1_ADE13 = 1;  // ADC_P			5
  ADER2_ADE17 = 0;
  ADER2_ADE19 = 0;
  ADER2_ADE21 = 0;
  ADER2_ADE23 = 0;
	
  ADCSH_STRT = 1;        // start adc

}


//__interrupt void ADC_IRQ(void)
void ADC_IRQ(void)
{
	static unsigned char AdcCnt = 0;
	
	ADCSL = 0xC0;   // Clear INT
	ADCSH = 0xA0;
  
	adc_value[AdcCnt] = ADCRLH;  			// write to array
	
	AdcCnt++;
	if (AdcCnt > 5)
		AdcCnt = 0;
	ADCSH_STRT = 1;		          // re-start ADC
}

