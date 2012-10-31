#include "mcu.h"
#include "4Imp.h"
#include "timer.h"
#include "icu.h"
#include "driver_update.h"

unsigned char count = 0;

void Update4Imp(void);

void Init_4_imp(void)
{
	DDR13_D0 = 1;
	PDR13_P0 = 0;
	Timer_Start(TIMER_ID_4IMP, 1, TRUE, Update4Imp);		
}

void Update4Imp(void)
{

	if (ProbegM > 1.0) 	
	{
		ProbegM = ProbegM - 1.0;
		count += 8;
	}
	
	if (count > 0)
	{
		count--;
		PDR13_P0 = ~PDR13_P0;
	}
}

