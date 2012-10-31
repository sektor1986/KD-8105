#include "mcu.h"
#include "utils.h"
#include "lcd.h"
                                    //  0     1     2     3     4     5     6     7     8     9     10    11    12    13    14    15    16    17    18
									//	0     1     2     3     4     5     6     7     8     9     space F     U     E     L     C     -     n     o
unsigned char const LCD_SEG[19]  =     {0x5F, 0x50, 0x6B, 0x79, 0x74, 0x3D, 0x3F, 0x58, 0x7F, 0x7D, 0x00, 0x2E, 0x57, 0x2F, 0x07, 0x0F, 0x20, 0x46, 0x4E};									
								   // 0       1       2       3       4       5       6       L       M        H      N       A      all    space
unsigned int const LCD_SEG1[14] = {0xB81D, 0x0005, 0x781E, 0x581F, 0xC007, 0xD81B, 0xF81B, 0xB010, 0xA445, 0xE007, 0xA425, 0x0247, 0xFFFF, 0x0000};
/*
unsigned char const LCD_SEG_1_3[17]  = {0xAF, 0x06, 0xCB, 0x4F, 0x66, 0x6D, 0xED, 0x07, 0xEF, 0x6F, 0x00, 0xA9, 0xA7, 0xE7, 0xE3, 0xE9, 0xE6};
unsigned char const LCD_SEG_7_9[11]  = {0xFA, 0x60, 0xD6, 0xF4, 0x6C, 0xBC, 0xBE, 0xE0, 0xFE, 0xFC, 0x00};
unsigned char const LCD_SEG_4[11]    = {0x5F, 0x06, 0x6B, 0x2F, 0x36, 0x3D, 0x7D, 0x07, 0x7F, 0x3F, 0x00};
unsigned char const LCD_SEG_10_5[12] = {0xAF, 0xA0, 0xCB, 0xE9, 0xE4, 0x6D, 0x6F, 0xA8, 0xEF, 0xED, 0x00, 0x0F};
unsigned char const LCD_SEG_6[11]    = {0x7B, 0x0A, 0x37, 0x1F, 0x4E, 0x5D, 0x7D, 0x0B, 0x7F, 0x5F, 0x00};
*/
void InitLCD(void)
{
  //LCD Control register LCR
  LCR_CSS = 0;  // 0= perclk1  1= clock selected by lecr/cksel 
  LCR_LCEN = 0; // 0= disable display in the timer mode    1 = Enable  
  LCR_VSEL = 1; // activate internal resistors
  LCR_BK = 0;   // 0 = enable display | 1 = Blank display
  
  LCR_MS0 = 1;  // 1/4 duty cycle  N=4
  LCR_MS1 = 1;
  
  LCR_FP0 = 1;  // CLKP1/(2^15 x N) @ pll 16MHz --> CLKP1 @ 16MHz
  LCR_FP1 = 0;

  //LCD common pin switching register
  LCDCMR_COMEN0 = 1; //Common pin x enable
  LCDCMR_COMEN1 = 1;
  LCDCMR_COMEN2 = 1;
  LCDCMR_COMEN3 = 1;
  LCDCMR_DTCH = 0;  // Bias control //  0=1/3 Bias //   1=Reserved

  //LCD extended control register LECR
  LECR_CKSEL = 1 ; //0 = sub clock CLKSC  1 = RC-Clock CLKRC

	LCDER0_SEG4 = 1;
	LCDER0_SEG5 = 1;
	LCDER0_SEG6 = 1;
	LCDER1_SEG8 = 1;
	LCDER1_SEG9 = 1;
	LCDER1_SEG10 = 1;
	LCDER1_SEG11 = 1;
	LCDER2_SEG19 = 1;
	LCDER2_SEG20 = 1;
	LCDER2_SEG21 = 1;
	LCDER2_SEG23 = 1;
	LCDER3_SEG30 = 1;
	LCDER4_SEG36 = 1;
	LCDER4_SEG37 = 1;
	LCDER4_SEG38 = 1;
	LCDER4_SEG39 = 1;
	LCDER6_SEG55 = 1;
	LCDER7_SEG56 = 1;

  //Voltage line enable register LCDVER
  LCDVER_V0 = 0;  //1 = External divide resistors 0 = Internal divide resistors
  LCDVER_V1 = 0;
  LCDVER_V2 = 0;
  LCDVER_V3 = 0;  // set V3 to 1 for external dimming
}


void segment1(unsigned char NB)
{
	VRAM4_DH = LCD_SEG1[NB];
	VRAM4_DL = LCD_SEG1[NB]>>4;
	VRAM3_DL = LCD_SEG1[NB]>>8;
	VRAM2_DH = LCD_SEG1[NB]>>12;
}

void segment2(unsigned char NB)
{
	VRAM5_DH0 = LCD_SEG[NB]>>4;
	VRAM5_DH1 = LCD_SEG[NB]>>5;
	VRAM5_DH2 = LCD_SEG[NB]>>6;
	VRAM5_DL = LCD_SEG[NB];
}

void segment3(unsigned char NB)
{
	VRAM27_DH0 = LCD_SEG[NB]>>4;
	VRAM27_DH1 = LCD_SEG[NB]>>5;
	VRAM27_DH2 = LCD_SEG[NB]>>6;
	VRAM28_DL = LCD_SEG[NB];
}

void segment4(unsigned char NB)
{
	VRAM9_DH0 = LCD_SEG[NB]>>4;
	VRAM9_DH1 = LCD_SEG[NB]>>5;
	VRAM9_DH2 = LCD_SEG[NB]>>6;
	VRAM10_DL = LCD_SEG[NB];
}

void segment5(unsigned char NB)
{
	VRAM18_DH0 = LCD_SEG[NB]>>4;
	VRAM18_DH1 = LCD_SEG[NB]>>5;
	VRAM18_DH2 = LCD_SEG[NB]>>6;
	VRAM19_DL = LCD_SEG[NB];
}

void segment6(unsigned char NB)
{
	VRAM10_DH0 = LCD_SEG[NB]>>4;
	VRAM10_DH1 = LCD_SEG[NB]>>5;
	VRAM10_DH2 = LCD_SEG[NB]>>6;
	VRAM11_DH = LCD_SEG[NB];
}

void NumToLCDStr(unsigned long num)
{
	unsigned long num_str = 0;
	
	num_str = ToBCD(num);	
			
	segment6(num_str & 0xF);       // Вывод едениц     
	if (num > 9)                    // Вывод десяток
		segment5((num_str>>4)&0xF);
	else
		if ((SIMBOL_P5) || (SIMBOL_P4))
			segment5(0);
		else
			segment5(10);
	
	if (num > 99)                   // Вывод сотен
		segment4((num_str>>8)&0xF);
	else
		if (SIMBOL_P4)
			segment4(0);
		else
			segment4(10);
		
	if (num > 999)                  // Вывод тысяч
		segment3((num_str>>12)&0xF);
	else
		if (SIMBOL_P3)
			segment3(0);
		else
			segment3(10);	
		
	if (num > 9999)                 // Вывод десяти тысяч 
		segment2((num_str>>16)&0xF);
	else
		segment2(10);	
} 

void NumToLCDStrFix(unsigned long num, unsigned char count_simbol)
{
	unsigned long num_str = 0;
	
	num_str = ToBCD(num);	
			
	segment6(num_str & 0xF);           // Вывод едениц     
    if (count_simbol > 1)              
		segment5((num_str>>4)&0xF);    // Вывод десяток
	else
		segment5(SPACE);
	
	if (count_simbol > 2)                    
		segment4((num_str>>8)&0xF);    // Вывод сотен
	else
		segment4(SPACE);
		
	if (count_simbol > 3)                      
		segment3((num_str>>12)&0xF);    // Вывод тысяч
	else
		segment3(SPACE);
		
	if (count_simbol > 4)                 
		segment2((num_str>>16)&0xF);    // Вывод десяти тысяч 
	else
		segment2(SPACE);	
}

void ClearLCDLine(void)
{
	segment2(SPACE);
	segment3(SPACE);
	segment4(SPACE);
	segment5(SPACE);
	segment6(SPACE);	
}

void Disable_simbols(void)
{
	SIMBOL_P2 = 0;
	SIMBOL_P3 = 0;
	SIMBOL_P4 = 0;
	SIMBOL_P5 = 0;	
	SIMBOL_DP3 = 0;
	SIMBOL_KM = 0;
	SIMBOL_H = 0;
	SIMBOL_I = 0;
	SIMBOL_slash = 0;
	SIMBOL_N = 0;
	SIMBOL_Z = 0;
	SIMBOL_R = 0;
	SIMBOL_D = 0;
	SIMBOL_K = 0;
	SIMBOL_V = 0;
	SIMBOL_1 = 0;
	SIMBOL_2 = 0;
	SIMBOL_T = 0;
	SIMBOL_min = 0;
	SIMBOL_m1 = 0;
}

void Enable_simbols(void)
{
	SIMBOL_P2 = 1;
	SIMBOL_P3 = 1;
	SIMBOL_P4 = 1;
	SIMBOL_P5 = 1;
	SIMBOL_DP3 = 1;
	SIMBOL_KM = 1;
	SIMBOL_H = 1;
	SIMBOL_I = 0;
	SIMBOL_slash = 1;
	SIMBOL_N = 1;
	SIMBOL_Z = 1;
	SIMBOL_R = 1;
	SIMBOL_D = 1;
	SIMBOL_K = 1;
	SIMBOL_V = 1;
	SIMBOL_1 = 1;
	SIMBOL_2 = 1;
	SIMBOL_T = 1;
	SIMBOL_min = 1;
	SIMBOL_m1 = 1;
}
