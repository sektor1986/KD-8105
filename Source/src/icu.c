#include "mcu.h"
#include "icu.h"

volatile unsigned long valueFUEL;
volatile unsigned long valueSpeed_L;
volatile unsigned long valueSpeed_R;
volatile unsigned long valuePTS;
volatile unsigned long valueRPM;

unsigned char PASS_COUNTER_FUEL_IRQ;
unsigned char PASS_COUNTER_SPEED_L_IRQ;
unsigned char PASS_COUNTER_SPEED_R_IRQ;
unsigned char PASS_COUNTER_PTS_IRQ;
unsigned char PASS_COUNTER_RPM_IRQ;


UNION_FLAG_FR flagFr; //// Флаги для определения поступления импульсов с датчиков

volatile unsigned int FRT0_ovl_cnt;
volatile unsigned int EI0_old;
volatile unsigned int EI0_new;
volatile unsigned int EI1_old;
volatile unsigned int EI1_new;
volatile unsigned int EI2_old;
volatile unsigned int EI2_new;
volatile unsigned int EI3_old;
volatile unsigned int EI3_new;
volatile unsigned int ICU1_old;
volatile unsigned int ICU1_new;
volatile unsigned int FRT00_ovl_cnt_new;
volatile unsigned int FRT00_ovl_cnt_old;
volatile unsigned int FRT01_ovl_cnt_new;
volatile unsigned int FRT01_ovl_cnt_old;
volatile unsigned int FRT02_ovl_cnt_new;
volatile unsigned int FRT02_ovl_cnt_old;
volatile unsigned int FRT03_ovl_cnt_new;
volatile unsigned int FRT03_ovl_cnt_old;
volatile unsigned int FRT04_ovl_cnt_new;
volatile unsigned int FRT04_ovl_cnt_old;

volatile unsigned long value;
unsigned char flag_freq = 0;

volatile unsigned int calc_koef_old;
volatile unsigned int calc_koef_new;
volatile unsigned int FRT0_ovl_calc_koef_new;
volatile unsigned int FRT0_ovl_calc_koef_old;
volatile unsigned long value_calc_koef;

unsigned long speed_output_counter;
unsigned char flag_calc_koef = 0;


float frequency;
unsigned char acceleration_limit = 0;

volatile unsigned long value_temp = 0;
volatile unsigned long W = 0;

float MetrInImp = 0.0;
float ProbegM = 0.0;

float km = 0.0;
float km_sut = 0.0;
float ProbegForSave = 0.0;

void InitFRTimer0(void)
{  
	TCCS0  |=0x49;		//div  2, interrupt enabled
	//TCCS0  |=0x4C;			// div 8
}

//------------------------------------------------------------------------------ 
// FRT0 (IRQ) 
//------------------------------------------------------------------------------ 
__interrupt void FRT0_IRQ (void)
{
	FRT0_ovl_cnt++;                      // Count the overflows 
	TCCS0_IVF = 0;
}

void InitICU(void)
{
	InitFRTimer0();   // Инициализация таймера
	// Уровень топлива INT0 P03_6
	PIER03_IE6 = 1;  
	ELVR0_LALB00 = 0;	// Rising edge
	ELVR0_LALB01 = 1;	
	EIRR0_ER0 = 0;  // reset interrupt request 
	ENIR0_EN0 = 1;  // enable interrupt request 

	// Скорость левое колесо INT1 P03_7
	PIER03_IE7 = 1; 
	ELVR0_LALB10 = 0;	// Rising edge
	ELVR0_LALB11 = 1;	 
	EIRR0_ER1 = 0;  // reset interrupt request 
	ENIR0_EN1 = 1;  // enable interrupt request 
	
	// Скорость правое колесо INT2 P13_0
	PIER13_IE0 = 1; 
	ELVR0_LALB20 = 0;	// Rising edge
	ELVR0_LALB21 = 1;	 
	EIRR0_ER2 = 0;  // reset interrupt request 
	ENIR0_EN2 = 1;  // enable interrupt request 
	
	// BOM INT3 P13_1 
	PIER13_IE1 = 1;  
	ELVR0_LALB30 = 0;	// Rising edge
	ELVR0_LALB31 = 1;	
	EIRR0_ER3 = 0;  // reset interrupt request 
	ENIR0_EN3 = 1;  // enable interrupt request 	
	
	//обороты RPM IN1 P06_5
	PIER06_IE5 = 1;  
	ICS01_EG1  = 1;   // Define start edge: 1:Rising  2:Falling  3:Both Edges
	ICS01_ICE1 = 1;   // Enable IRQ	
	
	// обнуление флагов прихода частоты
	flagFr.u_flagFr = 0x00;
}

//Прерывание по входу от датчика уровня топлива
__interrupt void EI0_IRQ (void)
{	
	static unsigned char PassCounter = 1;
	
	if (EIRR0_ER0)
	{
		EIRR0_ER0 = 0;		// Сброс флага прерывания
		flagFr.stc_FlagFr.fuel = 1;
			
		if ((PassCounter == 0) && (ELVR0_LALB00 == 0))  //Если нужное прерывание (счетчик досчитан, импульс по фронту)
		{
			EI0_new = TCDT0;                     // Save current ICU value
		
			FRT00_ovl_cnt_new = FRT0_ovl_cnt;      // Save current FRT value

			if (TCCS0_IVF && (TCDT0 < 0x8000))   // Check for FRT/ICU race condition 
			{                                     // In case that FRT was not yet handled 
				FRT00_ovl_cnt_new++;               // then increase temp FRT counter
			}
				
			// Calculation of time period
			valueFUEL = abs(FRT00_ovl_cnt_new - FRT00_ovl_cnt_old) * 0x10000UL + EI0_new - EI0_old;
 
			EI0_old = EI0_new;                  // Save current ICU value as reference for next cycle
			FRT00_ovl_cnt_old = FRT00_ovl_cnt_new;  // Save current FRT value as reference for next cycle		
			PassCounter = PASS_COUNTER_FUEL_IRQ;
		}
		else
		{
			PassCounter--;
		}
	
		ELVR0_LALB00 = 0;
	}
}

//Прерывание по входу от датчика скорости левого колеса
__interrupt void EI1_IRQ (void)
{	
	static unsigned char PassCounter = 1;
	
	if (EIRR0_ER1)
	{
		EIRR0_ER1 = 0;		// Сброс флага прерывания
		flagFr.stc_FlagFr.spd_l = 1;
			
		if ((PassCounter == 0) && (ELVR0_LALB10 == 0))  //Если нужное прерывание (счетчик досчитан, импульс по фронту)
		{
			EI1_new = TCDT0;                     // Save current ICU value
		
			FRT01_ovl_cnt_new = FRT0_ovl_cnt;      // Save current FRT value

			if (TCCS0_IVF && (TCDT0 < 0x8000))   // Check for FRT/ICU race condition 
			{                                     // In case that FRT was not yet handled 
				FRT01_ovl_cnt_new++;               // then increase temp FRT counter
			}
				
			// Calculation of time period
			valueSpeed_L = abs(FRT01_ovl_cnt_new - FRT01_ovl_cnt_old) * 0x10000UL + EI1_new - EI1_old;
 
			EI1_old = EI1_new;                  // Save current ICU value as reference for next cycle
			FRT01_ovl_cnt_old = FRT01_ovl_cnt_new;  // Save current FRT value as reference for next cycle		
			PassCounter = PASS_COUNTER_SPEED_L_IRQ;
		}
		else
		{
			PassCounter--;
		}
	
		ELVR0_LALB10 = 0;
	}
}

//Прерывание по входу от датчика скорости правого колеса
__interrupt void EI2_IRQ (void)
{	
	static unsigned char PassCounter = 1;
	
	if (EIRR0_ER2)
	{
		EIRR0_ER2 = 0;		// Сброс флага прерывания
		flagFr.stc_FlagFr.spd_r = 1;
			
		if ((PassCounter == 0) && (ELVR0_LALB20 == 0))  //Если нужное прерывание (счетчик досчитан, импульс по фронту)
		{
			EI2_new = TCDT0;                     // Save current ICU value
		
			FRT02_ovl_cnt_new = FRT0_ovl_cnt;      // Save current FRT value

			if (TCCS0_IVF && (TCDT0 < 0x8000))   // Check for FRT/ICU race condition 
			{                                     // In case that FRT was not yet handled 
				FRT02_ovl_cnt_new++;               // then increase temp FRT counter
			}
				
			// Calculation of time period
			valueSpeed_R = abs(FRT02_ovl_cnt_new - FRT02_ovl_cnt_old) * 0x10000UL + EI2_new - EI2_old;
 
			EI2_old = EI2_new;                  // Save current ICU value as reference for next cycle
			FRT02_ovl_cnt_old = FRT02_ovl_cnt_new;  // Save current FRT value as reference for next cycle		
			PassCounter = PASS_COUNTER_SPEED_R_IRQ;
		}
		else
		{
			PassCounter--;
		}
	
		ELVR0_LALB20 = 0;
	}
}

//Прерывание по входу от датчика ВОМа
__interrupt void EI3_IRQ (void)
{	
	static unsigned char PassCounter = 1;
	
	if (EIRR0_ER3)
	{
		EIRR0_ER3 = 0;		// Сброс флага прерывания
		flagFr.stc_FlagFr.pts = 1;
			
		if ((PassCounter == 0) && (ELVR0_LALB30 == 0))  //Если нужное прерывание (счетчик досчитан, импульс по фронту)
		{
			EI3_new = TCDT0;                     // Save current ICU value
		
			FRT03_ovl_cnt_new = FRT0_ovl_cnt;      // Save current FRT value

			if (TCCS0_IVF && (TCDT0 < 0x8000))   // Check for FRT/ICU race condition 
			{                                     // In case that FRT was not yet handled 
				FRT03_ovl_cnt_new++;               // then increase temp FRT counter
			}
				
			// Calculation of time period
			valuePTS = abs(FRT03_ovl_cnt_new - FRT03_ovl_cnt_old) * 0x10000UL + EI3_new - EI3_old;
 
			EI3_old = EI3_new;                  // Save current ICU value as reference for next cycle
			FRT03_ovl_cnt_old = FRT03_ovl_cnt_new;  // Save current FRT value as reference for next cycle		
			PassCounter = PASS_COUNTER_PTS_IRQ;
		}
		else
		{
			PassCounter--;
		}
	
		ELVR0_LALB30 = 0;
	}
}

__interrupt void ICU1_IRQ (void)
{
	static unsigned char PassCounter = 1;
	//Пропускаем прерывания пока счетчик пропусков не станет равен 0
	if ((PassCounter == 0) && (ICS01_EG1 == 1))  //Если нужное прерывание (счетчик досчитан, импульс по фронту)
	{
		ICU1_new = IPCP1;                     // Save current ICU value
		FRT04_ovl_cnt_new = FRT0_ovl_cnt;      // Save current FRT value
		flagFr.stc_FlagFr.rpm = 1;
	
		if (TCCS0_IVF && (IPCP1 < 0x8000))   // Check for FRT/ICU race condition 
		{                                     // In case that FRT was not yet handled 
			FRT04_ovl_cnt_new++;                 // then increase temp FRT counter
		}

		// Calculation of time period
		valueRPM = abs(FRT04_ovl_cnt_new - FRT04_ovl_cnt_old) * 0x10000UL + ICU1_new - ICU1_old;
 
		ICU1_old = ICU1_new;                  // Save current ICU value as reference for next cycle
		FRT04_ovl_cnt_old = FRT04_ovl_cnt_new;  // Save current FRT value as reference for next cycle 
  
		PassCounter = PASS_COUNTER_RPM_IRQ; 	
	}
	else
	{
		PassCounter--;			
	}		

	ICS01_ICP1 = 0; // Clear ICU Irq-flag
}
