#ifndef __LCD_H
#define __LCD_H

//#define ENABLE_POINT VRAM11_DH3
//#define ENABLE_KM    VRAM8_DL0

#define SIMBOL_P2		VRAM2_DL1
#define SIMBOL_P3		VRAM15_DL1
#define SIMBOL_P4		VRAM19_DH2
#define SIMBOL_P5		VRAM18_DL2
#define SIMBOL_DP3		VRAM15_DL2
#define SIMBOL_KM    	VRAM18_DL0
#define SIMBOL_H    	VRAM15_DL0
#define SIMBOL_I    	VRAM19_DH0
#define SIMBOL_slash	VRAM2_DL0
#define ENABLE_KM_H    	{SIMBOL_KM = 1; SIMBOL_slash = 1; SIMBOL_H = 1;}
#define ENABLE_L_H    	{SIMBOL_I = 1; SIMBOL_slash = 1; SIMBOL_H = 1;}
#define SIMBOL_N		VRAM5_DH3
#define SIMBOL_Z		VRAM2_DL3
#define SIMBOL_R		VRAM27_DH3
#define SIMBOL_D		VRAM15_DL3
#define SIMBOL_K		VRAM9_DH3
#define SIMBOL_V		VRAM19_DH3
#define SIMBOL_1		VRAM18_DH3
#define SIMBOL_2		VRAM18_DL3
#define SIMBOL_T		VRAM10_DH3
#define SIMBOL_min		VRAM19_DH1
#define SIMBOL_m1		VRAM18_DL1


#define SPACE          10

void InitLCD (void);
void NumToLCDStr(unsigned long num);
void NumToLCDStrFix(unsigned long num, unsigned char count_simbol);

void segment1(unsigned char NB);
void segment2(unsigned char NB);
void segment3(unsigned char NB);
void segment4(unsigned char NB);
void segment5(unsigned char NB);
void segment6(unsigned char NB);


void ClearLCDLine(void);
void Disable_simbols(void);
void Enable_simbols(void);

#endif // __LCD_H
