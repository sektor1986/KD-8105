#ifndef __ICU_H
#define __ICU_H

typedef union _UNION_FLAG_FR
{
	unsigned char u_flagFr;
	struct _STRUCT_FLAG_FR
	{
		unsigned char fuel : 1;
		unsigned char spd_l : 1;
		unsigned char spd_r : 1;
		unsigned char pts : 1;
		unsigned char rpm : 1;
		unsigned char can : 1;
		unsigned char res2 : 1;
		unsigned char res3 : 1;
	} stc_FlagFr;
} UNION_FLAG_FR;

// Флаги для определения поступления импульсов с датчиков
extern UNION_FLAG_FR flagFr;

extern volatile unsigned long valueSpeed_L;
extern volatile unsigned long valueSpeed_R;
extern volatile unsigned long valuePTS;
extern volatile unsigned long valueRPM;
extern volatile unsigned long valueFUEL;

extern unsigned char PASS_COUNTER_SPEED_L_IRQ;
extern unsigned char PASS_COUNTER_SPEED_R_IRQ;
extern unsigned char PASS_COUNTER_PTS_IRQ;
extern unsigned char PASS_COUNTER_RPM_IRQ;
extern unsigned char PASS_COUNTER_FUEL_IRQ;

void InitICU(void);

__interrupt void FRT0_IRQ (void);
__interrupt void EI0_IRQ (void);
__interrupt void EI1_IRQ (void);
__interrupt void EI2_IRQ (void);
__interrupt void EI3_IRQ (void);
__interrupt void ICU1_IRQ (void);

#endif // __ICU_H