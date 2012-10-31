#ifndef __74hc595_H
#define __74hc595_H

//74hc595
#define HC595_PIN_DATA		PDR17_P0
#define HC595_PIN_CLK		PDR04_P5
#define HC595_PIN_STROBE	PDR13_P6
#define HC595_PIN_OE        PDR13_P6

#define HC595_DATA_ON		HC595_PIN_DATA = 1;
#define HC595_DATA_OFF		HC595_PIN_DATA = 0;
#define HC595_CLK			{HC595_PIN_CLK = 1; HC595_PIN_CLK = 0;}
#define HC595_STROBE		{HC595_PIN_STROBE = 1; HC595_PIN_STROBE = 0;}
#define HC595_ENABLE        HC595_PIN_OE = 0;
#define HC595_DISABLE       HC595_PIN_OE = 1;

typedef union _UNION_DATA_LED
{
	unsigned char u_dataLED;
	struct _STRUCT_DATA_LED
	{
		unsigned long led0 : 1;
		unsigned long led1 : 1;
		unsigned long led2 : 1;
		unsigned long led3 : 1;
		unsigned long led4 : 1;
		unsigned long led5 : 1;
		unsigned long led6 : 1;
		unsigned long led7 : 1;
		unsigned long led8 : 1;
	} stc_DataLED;
} UNION_DATA_LED;


extern UNION_DATA_LED dataLED;


void Init74hc595(void);
void Sethc595(unsigned char data74hc595);

#endif // __74hc595_H
