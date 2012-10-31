#include "mcu.h"
#include "backlight.h"
#include "adc.h"

unsigned long Backlight = 0; //0..250
unsigned long BacklightMon = 0; //0..250

#define STATE_BACKLIGHT PDR13_P5
#define STATE_BACKLIGHT_MON PDR04_P4

void InitBacklight(void)
{
	DDR04_D4 = 1;
	DDR13_D5 = 1;
	STATE_BACKLIGHT = 0;
	STATE_BACKLIGHT_MON = 0;
	TMR1  = 10;                /* set reload value in [us] 1.6*x   */ 
	TMCSR1  = 0x81B;           /* prescaler 1.6us at 40MHz         */ 
}

void UpdateBacklight(void)
{
	static unsigned int count = 0;
	static unsigned int count_mon = 0;
	
	//STATE_BACKLIGHT = ~STATE_BACKLIGHT;

	Backlight &= 0xFF;
	if (count > Backlight)
		STATE_BACKLIGHT = 0;
	
	if (count == 255)
	{
		count = 0;
		if (Backlight != 0)
			STATE_BACKLIGHT = 1;
	}
	count++;
	
	BacklightMon &= 0xFF;
	if (count_mon > BacklightMon)
		STATE_BACKLIGHT_MON = 0;
	
	if (count_mon == 255)
	{
		count_mon = 0;
		if (BacklightMon != 0)
			STATE_BACKLIGHT_MON = 1;
	}
	count_mon++;
	
}

void SetValueBacklight(void)
{
	unsigned long new_backlight = 0;
	unsigned long new_backlight_mon = 0;
	
	if (adc_value[ADC_LIGHT] > 200)
	{
		if (adc_value[ADC_LIGHT] < 512)
			new_backlight = adc_value[ADC_LIGHT] >> 1;
		else
			new_backlight = 255;
		new_backlight_mon = 100;		
	}
	else
	{
		new_backlight = 0;
		new_backlight_mon = 250;
	}


	if (new_backlight > Backlight)
		Backlight++;
	else if (new_backlight < Backlight)
		Backlight--;
		
	if (new_backlight_mon > BacklightMon)
		BacklightMon++;
	else if (new_backlight_mon < BacklightMon)
		BacklightMon--;		
}

void SetBacklightNull(void)
{
	Backlight = 0;
	BacklightMon = 0;	
}

__interrupt void Backlight_IRQ(void)
//void SMC_IRQ (void)
{   /* background task for motor controlling */ 
	UpdateBacklight();			//Подсветка

	TMCSR1_UF = 0;    /* reset underflow interrupt request flag    */ 
}

