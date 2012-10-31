#include "mcu.h"
#include "74hc595.h"

UNION_DATA_LED dataLED;

void Init74hc595(void)
{
	DDR17_D0 = 1; //HC595_PIN_DATA
	DDR04_D5 = 1; //HC595_PIN_CLK
	DDR13_D6 = 1; //HC595_PIN_STROBE
	DDR13_D6 = 1; //OE
	HC595_ENABLE
	Sethc595(0);
}

void Sethc595(unsigned char data74hc595)
{
	char i;

	for(i=8;i>0;i--)
	{
		if(data74hc595 & 0x80)
			HC595_DATA_ON 
		else 
			HC595_DATA_OFF
		
       
		data74hc595 <<= 1;
		HC595_CLK		
	}	
	HC595_STROBE

}
