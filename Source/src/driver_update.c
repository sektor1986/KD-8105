#include "mcu.h"
#include "basics.h"
#include "button.h"
#include "driver_update.h"
#include "timer.h"
#include "icu.h"
#include "lcd.h"
#include "smc.h"
#include "utils.h"
#include "flash.h"
#include "adc.h"
#include "rtc.h"
#include "j1939.h"
#include "backlight.h"
#include "uart.h"
#include "74hc595.h"
#include "options.h"

#define PI           3.14159
#define max(a,b) ((a) > (b) ? (a) : (b))

#define PARKING  PDR05_P4 
#define DRIVE_LOAD   PDR05_P1
#define LOW_SPEED    PDR03_P7
#define V8_OFF       PDR17_P0

#define MAX_ACCELERATION   10.0       // Максимальное ускорение  km/c*c

#define MAX_SPEED 120
#define SPEED_NO_CALC (MAX_SPEED + 7)

#define TIME_SHOW_PROBEG_IN_MODE_EMPTY      8

static en_gui_mode_t    m_enGuiMode = mode_Motochasi;
static en_gui_mode_t    m_enGuiMode_before_emty = mode_Motochasi;

unsigned int TIME_OBNUL_FR_SPEED_L = 0;           // Константа для таймера, установка 0 при пропадании частоты (скорость левое колесо)
unsigned int TIME_OBNUL_FR_SPEED_R = 0;           // Константа для таймера, установка 0 при пропадании частоты (скорость правое колесо)
unsigned int TIME_OBNUL_FR_TAH = 0;               // Константа для таймера, возврат стрелки на 0 (тахометр)
unsigned int TIME_OBNUL_FR_VOM = 0;               // Константа для таймера, ВОМ
unsigned int TIME_OBNUL_FR_FUEL = 0;              // Константа для таймера, уровень топлива

#define TIME_RETURN_MENU        7
unsigned int Time_return_menu = 7;

unsigned char params_Z[6] = {23, 34, 54, 56, 69, 23};
unsigned int params_I[5]  = {1000, 3430, 3640, 4000, 1000};
unsigned int params_K[10] = {2360, 2416, 2485, 2500, 2720, 3000, 3320, 3500, 4000, 2360};
unsigned int params_KV2[6] = {346, 460, 520, 597, 600, 346};
unsigned int params_ZV[6] = {0, 12, 15, 70, 78, 78};

#define MAX_PARAMS_Z    6
#define MAX_PARAMS_I    5
#define MAX_PARAMS_K    10
#define MAX_PARAMS_KV2   6
#define MAX_PARAMS_ZV    6

unsigned char flag_edit_params = 0;
unsigned char num_edit_simbol = 0;
unsigned char current_num_params = 0;
unsigned char max_count_params = 1;

unsigned char count_ed = 0;
unsigned char count_des = 0;
unsigned char count_sot = 0;
unsigned char count_tis = 0;

unsigned char count_rewriteK = 0;
unsigned char visible_countRewrK = 0;
unsigned char Time_visible_countRewrK = 0;

typedef union _UNION_ERROR
{
	unsigned char u_error;
	struct _STRUCT_ERROR
	{
		unsigned long no_speed_r : 1;
		unsigned long no_speed_l : 1;
		unsigned long no_fuel : 1;
		unsigned long no_can : 1;
		unsigned long led4 : 1;
		unsigned long led5 : 1;
		unsigned long led6 : 1;
		unsigned long led7 : 1;
		unsigned long led8 : 1;
	} stc_Error;
} UNION_ERROR;

UNION_ERROR ErrorBuf;

unsigned char flagVisibleError = 1;
unsigned char flagVisibleErrorFuel = 1;
unsigned char flagVisibleErrorRSpeed = 1;
unsigned char flagVisibleErrorLSpeed = 1;
unsigned char flagVisibleErrorCAN= 1;

unsigned char flag_dual_press = 0;
unsigned char Num = 0;                          // номер редактируемого символа
unsigned char blink = 0;                        // мерцание
unsigned int TimerBlink = 0;                    // Переменная таймера мерцания 
unsigned long value_fr_old = 0;                 // Переменная для хренения предыдущего значения счетчика частоты
unsigned char flag_smcUpdate = 1;
unsigned int nul_to_spd = 0; 
unsigned char timer_show_probeg_in_mode_empty = 0;  

// Коэффициенты для спидометра
unsigned int R = 600;                         // Радиус колеса в милиметрах
unsigned int Z  = 69;                          // введенное значение количества зубьев
float I = 1.0;                        // введенное значение передаточного отношения колесного редуктора
float K_SPEED = 0;

// Коэффициенты для тахометра
float K = 2.36;
unsigned char P = 6;
float K_TAH = 0;

// Коэффициенты для ВОМ
float KV2 = 1;
unsigned char ZV = 15;
float K_VOM = 0;

// Обьем топливного бака
unsigned int V = 500;


volatile float Speed = 0.0;                              // Переменная для хранения значения скорости
volatile float Tahom = 0.0;                              // Переменная для хранения значения оборотов двигателя
volatile float Vom   = 0.0;                              // Переменая для хранения значения оборотов вом
volatile float Fuel  = 0.0;                              // Переменная для хранения уровная топлива в процентах
unsigned long Motochasi = 0;

unsigned long Vom_visible = 0;
unsigned long Fuel_visible = 0;

unsigned char save_params_flag = 0;

unsigned char N[5] = {0, 0, 0, 0, 0};

unsigned int PASS     = 0;

unsigned int timer_mode_empty = 0;

void UpdateBlink(void);
void DriverUpdate_1s(void);
void DriverUpdate_60s(void);;
void DriverUpdate(void);
void RecalcVariables(void);
void SaveKFrom_Uart(void);
void InitMode(en_gui_mode_t mode);
void UpdateSMC(void);
void UpdateLCD(void);
void UpdateVOM(void);
void UpdateFUEL(void);
// Функции сохранения параметров
void LoadAllParams(void);
unsigned int SaveAllParams(void);
unsigned int SaveAllParamsSecondary(void);
//Инициализаця системы
void InitSystem(void);
void InitTestMode(void);
void WaitDualKeyPres(void);



void DriverInit(void)
{		
	ErrorBuf.u_error = 0;
	DDR05_D4 = 1;
	LoadAllParams();		// Чтение параметров из flash	
	params_Z[MAX_PARAMS_Z-1] = Z;
	params_I[MAX_PARAMS_I-1] = (unsigned int)(I*1000);
	params_K[MAX_PARAMS_K-1] = (unsigned int)(K*1000);
	params_KV2[MAX_PARAMS_KV2-1] = (unsigned int)(KV2*1000);
	params_ZV[MAX_PARAMS_ZV-1] = ZV;
	RecalcVariables();
	InitSystem(); 			// Инициализация системы    */ 
	Timer_Start(TIMER_250MS, 250, TRUE, UpdateBlink);	
	Timer_Start(TIMER_ID_MAIN, 20, TRUE, DriverUpdate);	
	Timer_Start(TIMER_1S, 1000, TRUE, DriverUpdate_1s);
	Timer_Start(TIMER_60S, 60000, TRUE, DriverUpdate_60s);	
	InitMode(mode_Motochasi);	
}

void InitSystem(void)
{
	NumToLCDStr(23456);
}

void InitTestMode(void)
{
	RecalcVariables();
	Enable_simbols();
	NumToLCDStr(88888);
	segment1(12);
	Sethc595(0x7F);
	Timer_Wait(TIMER_ID_WAIT, 2000, TRUE);  
	Disable_simbols();
	segment1(11);
	InitMode(m_enGuiMode_before_emty);	
}

void DriverUpdate(void)
{	
	static unsigned char save_params_flag = 0;
	static unsigned char timer_mode_empty = 0;
	static unsigned char flag_update = 1;
	
	if ((adc_value[ADC_IGNITION] > 160) && (adc_value[ADC_IGNITION] < 370))
	{
		if ((m_enGuiMode == Empty) && (adc_value[ADC_IGNITION] > 170) && (adc_value[ADC_IGNITION] < 350))
		{
			m_enGuiMode = m_enGuiMode_before_emty;
			InitMode(m_enGuiMode);
			flag_update = 1;
			SmcNormalParams();
			timer_mode_empty = 0;
		}		

		save_params_flag = 1;
	}
	else
	{
		if (save_params_flag)
		{
			SaveAllParams();	
			save_params_flag = 0;
		}	
		if (timer_mode_empty < 5)
			timer_mode_empty++;
		else
		{
			Sethc595(0x00);
			SmcParamsForReturn();
			if (m_enGuiMode != Empty)
			{	
				m_enGuiMode_before_emty = m_enGuiMode;	
				m_enGuiMode = Empty;
				InitMode(m_enGuiMode);
			}	
			flag_update = 0;
		}			
	}
	
	if (flag_update)
	{
		SetValueBacklight();
		UpdateSMC();
		UpdateVOM();
		UpdateFUEL();
		UpdateLCD();
	}
	else
	{
		SetBacklightNull();
	}
	
	
}

void RecalcVariables(void)
{
	K_SPEED = 0.022619448 * (float)R / Z / I; // V=2Pi * Rk/1000000 * 3600*F/(Z*Vi); 
	K_TAH = 60.0 / P / K;
	if (ZV != 0)
		K_VOM = 60.0 / ZV;
	else
		K_VOM = 0;
	
	PASS_COUNTER_FUEL_IRQ = 2;
	PASS_COUNTER_SPEED_L_IRQ = 2;
	PASS_COUNTER_SPEED_R_IRQ = 2;
	PASS_COUNTER_PTS_IRQ = 2;
	PASS_COUNTER_RPM_IRQ = 2;
	
	TIME_OBNUL_FR_SPEED_L = 500 * K_SPEED;  // Per = 3660*1000/K/20  
	TIME_OBNUL_FR_SPEED_R = 500 * K_SPEED;  // Per = 3660*1000/K/20  
	TIME_OBNUL_FR_TAH = 3;
	TIME_OBNUL_FR_VOM = 150;
	TIME_OBNUL_FR_FUEL = 100;
}

void UpdateBlink(void)
{
	static unsigned char i = 0;
	blink = ~blink;
	i++;
	if (i > 2) 
		i = 0;
	if ((adc_value[ADC_P] < 300) && (i == 2))
		PARKING = ~PARKING;
	else
		PARKING = 0;
		
	Vom_visible = Vom;
	Fuel_visible = Fuel * V / 100.0;
}

void DriverUpdate_1s(void)		
{	
	static unsigned char temp = 0;
	static unsigned char counter_error_speed_l = 0;
	static unsigned char counter_error_speed_r = 0;
	static unsigned char counter_error_fuel = 0;
	static unsigned char counter_error_CAN = 0;
	
	switch (m_enGuiMode)
	{		
		case mode_ParamsZ:
		case mode_ParamsI:
		case mode_ParamsR:
		case mode_ParamsK:
		case mode_ParamsKV2:
		case mode_ParamsZV:
		case mode_ParamsV:
			Time_return_menu--;	
			if (Time_return_menu == 0)
				InitTestMode();
			break;	
		default:
			break;
	}
	
	// Увеличение моточасов на 1 секунду
	if (Tahom > 50)
		Motochasi++;
	
	if (flagFr.stc_FlagFr.spd_r || flagFr.stc_FlagFr.spd_l)
	{
		if (!flagFr.stc_FlagFr.spd_l)
			counter_error_speed_l++;
		else
			counter_error_speed_l = 0;
		if (!flagFr.stc_FlagFr.spd_r)
			counter_error_speed_r++;
		else
			counter_error_speed_r = 0;			
	}
	else
	{
		counter_error_speed_l = 0;
		counter_error_speed_r = 0;	
		ErrorBuf.stc_Error.no_speed_r = 0;
		ErrorBuf.stc_Error.no_speed_l = 0;		
	}
	/*Если после начала движения нет сигнала с датчика правого колеса*/
	if (counter_error_speed_r > 11)
	{
		counter_error_speed_r = 11;
		if ((!ErrorBuf.stc_Error.no_speed_r) && (flagVisibleErrorRSpeed))
		{
			ErrorBuf.stc_Error.no_speed_r = 1;
			//InitMode(mode_ErrorSpeedR);
		}
	} 
	
	/*Если после начала движения нет сигнала с датчика левого колеса*/
	if (counter_error_speed_l > 11)
	{
		counter_error_speed_l = 11;
		if ((!ErrorBuf.stc_Error.no_speed_l) && (flagVisibleErrorLSpeed))
		{
			ErrorBuf.stc_Error.no_speed_l = 1;
			//InitMode(mode_ErrorSpeedL);
		}
	} 
	
	if (!flagFr.stc_FlagFr.fuel)
		counter_error_fuel++;	
	else
		counter_error_fuel = 0;
		
	/*Если данных с датчика уровня топлива нет в течении 2 секунд*/
	if (counter_error_fuel > 2)
	{
		counter_error_fuel = 2;
		ErrorBuf.stc_Error.no_fuel = 1;
	}
	
	#if (DEVICE_TYPE == KD8105_CAN)
		/*Если данных по CAN не в течении 5 секунд*/
		if (!flagFr.stc_FlagFr.can)
			counter_error_CAN++;	
		else
		{
			counter_error_CAN = 0;
			flagFr.stc_FlagFr.can = 0;
		}
		
		if (counter_error_CAN > 5)
		{
			counter_error_CAN = 5;
			ErrorBuf.stc_Error.no_can = 1;
		}
	#endif

	
	if ((m_enGuiMode == mode_Motochasi) || (m_enGuiMode == mode_Vom) || (m_enGuiMode == mode_Fuel) || (m_enGuiMode == mode_ErrorFuel) || (m_enGuiMode == mode_ErrorCAN))
	{
		if ((ErrorBuf.stc_Error.no_speed_l) && flagVisibleErrorLSpeed)
		{
			
			m_enGuiMode_before_emty = m_enGuiMode;
			InitMode(mode_ErrorSpeedL);
		}
		else if ((ErrorBuf.stc_Error.no_speed_r) && flagVisibleErrorRSpeed)
		{
			m_enGuiMode_before_emty = m_enGuiMode;
			InitMode(mode_ErrorSpeedR);
		}
		else if ((ErrorBuf.stc_Error.no_fuel) && (m_enGuiMode != mode_ErrorSpeedL) && (m_enGuiMode != mode_ErrorSpeedR)  && flagVisibleErrorFuel) 
		{
			m_enGuiMode_before_emty = m_enGuiMode;
			InitMode(mode_ErrorFuel);
		}
		#if (DEVICE_TYPE == KD8105_CAN)
			else if ((ErrorBuf.stc_Error.no_can) && (m_enGuiMode != mode_ErrorSpeedL) && (m_enGuiMode != mode_ErrorSpeedR) && (m_enGuiMode != mode_ErrorFuel) && flagVisibleErrorCAN)
			{
				m_enGuiMode_before_emty = m_enGuiMode;
				InitMode(mode_ErrorCAN);
			}
		#endif
	}
	
	// Проверка данных с UART
	time_no_data++;
	if (time_no_data > 2)
	{
		Rxbuf = 0;
		time_no_data = 2;
	}
				
}

void DriverUpdate_60s(void)
{
	// Если после сохранения проехали более 5 км
/*
	if (ProbegForSave > 5000.0)
	{
		//Обнуляем пробег 
		ProbegForSave = 0.0;
		// Сохраняем данные
		SaveAllParamsSecondary();		
	}
*/
}

void UpdateLCD(void)
{	
	unsigned long temp = 0;
	unsigned long temp_float = 0.0;
	
	switch (Rxbuf)
	{
		case 0x00:
			segment1(11);    // Символ А
			break;
		case 1:
			segment1(0);     // Символ 0
			break;
		case 2:
			segment1(1);     // Символ 1
			break;
		case 4:
			segment1(2);     // Символ 2
			break;
		case 8:
			segment1(3);     // Символ 3
			break;
		case 16:
			segment1(4);     // Символ 4
			break;
		case 32:
			segment1(5);     // Символ 5
			break;
		case 64:
			segment1(6);     // Символ 6
			break;
		case 193:
			segment1(7);     // Символ L
			break;
		case 194:
			segment1(8);     // Символ M
			break;
		case 196:
			segment1(9);     // Символ H
			break;
		case 200:
			segment1(10);     // Символ N
			break;
		default:
			break;
	}
	
	switch (m_enGuiMode)
	{	
		case mode_Motochasi:
			NumToLCDStr((unsigned long)Motochasi/3600);	
			break;
		case mode_Vom:
			NumToLCDStr((unsigned long)Vom_visible);
			break;		
		case mode_Fuel:
			NumToLCDStr((unsigned long)Fuel_visible);
			break;
		case mode_Vbort:
			NumToLCDStr((J1939CtrBufer[VEP1].CanMessage.stcData.aucData[6] | 
				(unsigned int)(J1939CtrBufer[VEP1].CanMessage.stcData.aucData[7]) << 8)*5);
			break;
		case mode_FuelRate:
			NumToLCDStr((J1939CtrBufer[LFE].CanMessage.stcData.aucData[0] | 
				(unsigned int)(J1939CtrBufer[LFE].CanMessage.stcData.aucData[1]) << 8)*5);	

			break;
		case mode_ProbegMotochNaOst:
			temp_float = (float)(J1939CtrBufer[LFE].CanMessage.stcData.aucData[0] | 
			(unsigned int)(J1939CtrBufer[LFE].CanMessage.stcData.aucData[1]) << 8) / 20.0;
			if (temp_float != 0)
			{
				if (Speed > 3.0)
				{
					NumToLCDStr((unsigned long)(Speed * Fuel_visible / temp_float));
					SIMBOL_H = 0;
					SIMBOL_KM = 1;
				}
				else
				{
					NumToLCDStr((unsigned long)(Fuel_visible / temp_float));
					SIMBOL_H = 1;
					SIMBOL_KM = 0;
				}
			}
			else
				NumToLCDStr(0);
			break;
		case mode_ParamsT:
			NumToLCDStr((unsigned long)(Motochasi/36));
			break;		
		case mode_ErrorSpeedL:
			segment2(0);
			segment3(16);		
			segment4(16);
			segment5(16);
			segment6(16);
			break;
		case mode_ErrorSpeedR:
			segment2(16);
			segment3(16);		
			segment4(16);
			segment5(16);
			segment6(0);
			break;			
		case mode_ErrorFuel:
			segment2(SPACE);
			segment3(11);		
			segment4(12);
			segment5(13);
			segment6(14);
			break;
		case mode_ErrorCAN:
			segment2(15);
			segment3(16);		
			segment4(8);
			segment5(12);
			segment6(5);
			break;			

		default:
			break;
	}	
	
	if (flag_edit_params)
	{
		temp = N[0] + N[1]*10 + N[2]*100 + N[3]*1000;
		switch (m_enGuiMode)
		{
			case mode_ParamsZ:
				NumToLCDStrFix(temp, 2);
				break;
			case mode_ParamsI:
				NumToLCDStrFix(temp, 4);
				break;	
			case mode_ParamsR:
				NumToLCDStrFix(temp, 4);
				break;					
			case mode_ParamsK:
				NumToLCDStrFix(temp, 4);
				break;	
			case mode_ParamsKV2:
				NumToLCDStrFix(temp, 4);
				break;
			case mode_ParamsZV:
				NumToLCDStrFix(temp, 2);
				break;
			case mode_ParamsV:
				NumToLCDStrFix(temp, 3);
				break;					
			default: 		
				break;			
		}
		if (blink)
		{
			switch (num_edit_simbol)
			{
				case 0:
					segment6(SPACE);
					break;
				case 1:
					segment5(SPACE);
					break;
				case 2:
					segment4(SPACE);
					break;
				case 3:
					segment3(SPACE);
					break;	
				default:
					break;				
			}			
		}
	}
	else
	{
		switch (m_enGuiMode)
		{
			case mode_ParamsZ:
				NumToLCDStr(Z);
				break;
			
			case mode_ParamsI:
				NumToLCDStr(I*1000);
				break;
			
			case mode_ParamsR:
				NumToLCDStr(R);
				break;
			
			case mode_ParamsK:
				NumToLCDStr(K*1000);
				break;
			
			case mode_ParamsKV2:
				NumToLCDStr(KV2*1000);
				break;
			
			case mode_ParamsZV:
				NumToLCDStr(ZV);
				break;		
			
			case mode_ParamsV:
				NumToLCDStr(V);
				break;			
			
			default:
				break;	
		}
	}
}

void UpdateSMC(void)
{	
	unsigned long temp_sms_inp = 0;
	static unsigned int TimerFr = 0;                       // Переменная таймера обнуленя частоты
	static unsigned int TimerSpeedL = 0; 
	static unsigned int TimerSpeedR = 0;
	float frequency = 0;
	unsigned long min_value_speed = 0;
	
	/* Указатель оборотов */
	if (J1939CtrBufer[EEC1].Available)   // обработка сигнала с CAN
	{
		Tahom = (J1939CtrBufer[EEC1].CanMessage.stcData.aucData[3] | 
				(unsigned int)(J1939CtrBufer[EEC1].CanMessage.stcData.aucData[4]) << 8) / 8;		
	}
	else                                 // обработка сигнала с частотного датчика
	{
		if (TimerFr > TIME_OBNUL_FR_TAH)
		{
			if (flagFr.stc_FlagFr.rpm == 0)
			{
				valueRPM = 0;
			}
			flagFr.stc_FlagFr.rpm = 0;
			TimerFr = 0;
		}
		else
		{
			TimerFr++;
		}
		if (valueRPM)
			frequency = 16000000.0/((float)valueRPM/(float)(PASS_COUNTER_RPM_IRQ+1));	
		else
			frequency = 0;
		Tahom = (float)frequency * (float)K_TAH;	
	}
	if (Tahom > 3550)
		Tahom = 3550;	
	
	if (Tahom < 1000)
	// 1 градус 512 шагов, 1об/мин = 32 / 1000  = 0,032 гр; 1об/мин = 0,032*512 = 16,384 шага; max = 1000*16,384 = 16384
		temp_sms_inp = Tahom * 16.384;
	else if (Tahom < 1500)
	// 1 градус 512 шагов, 1об/мин = 34.67 / 500  = 0,06934 гр; 1об/мин = 0,06934*512 = 35.502 шага; max = 16384 + 35.502*500 = 34135
		temp_sms_inp  = (Tahom - 1000) * 35.502 + 16384;
	else if (Tahom < 2000)
	// 1 градус 512 шагов, 1об/мин = 33.33 / 500  = 0,06666 гр; 1об/мин = 0,06666*512 = 34,1299 шага; max = 34135 + 34.1299*500 = 51200
		temp_sms_inp  = (Tahom - 1500) * 34.1299 + 34135;
	else 
	// 1 градус 512 шагов, 1об/мин = 50 / 1500  = 0,03333 гр; 1об/мин = 0,03333*512 = 17.0667 шага; max = 51200 + 17.0667*1500 = 76800
		temp_sms_inp  = (Tahom - 2000) * 17.0667 + 51200;	
	SMCpos[TAHOMETR].smc_inp = temp_sms_inp;
	/*Конец обрабoтки указателя оборотов*/		
	
	/* Указатель скорости */
	/* Левое колесо */
	if (TimerSpeedL > TIME_OBNUL_FR_SPEED_L)
	{
		if (flagFr.stc_FlagFr.spd_l == 0)
		{
			valueSpeed_L = 0;
		}
		flagFr.stc_FlagFr.spd_l = 0;
		TimerSpeedL = 0;
	}
	else
	{
		TimerSpeedL++;
	}
	/* Правое колесо*/
	if (TimerSpeedR > TIME_OBNUL_FR_SPEED_R)
	{
		if (flagFr.stc_FlagFr.spd_r == 0)
		{
			valueSpeed_R = 0;
		}
		flagFr.stc_FlagFr.spd_r = 0;
		TimerSpeedR = 0;
	}
	else
	{
		TimerSpeedR++;
	}	
	min_value_speed = max(valueSpeed_R, valueSpeed_L);
	
	if (min_value_speed)
		frequency = 16000000.0/((float)min_value_speed/(float)(PASS_COUNTER_SPEED_L_IRQ+1));	
	else
		frequency = 0;
	Speed = (float)frequency * (float)K_SPEED;	
	if (Speed > 53.0)
		Speed = 53.0;
	if (Speed < 20.0)
	// 1 градус 512 шагов, 1км/ч = 88 / 20  = 4,4 гр; 1км/ч = 4,4*512 = 2252,8 шага; max = 20*2252,8 = 45056
		temp_sms_inp = Speed * 2252,8;
	else
	// 1 градус 512 шагов, 1км/ч = 66 / 30  = 2.2 гр; 1км/ч = 2.2*512 = 1126,4 шага; max = 30*1126,4 + 45056 = 78848
		temp_sms_inp = (Speed - 20.0) * 1126.4 + 45056;		
	SMCpos[SPEEDOMETR].smc_inp = temp_sms_inp;
	/*Конец обрабoтки указателя скоорсти*/	
}

void UpdateVOM(void)
{
	static unsigned int TimerFr = 0;                       // Переменная таймера обнуленя частоты
	float frequency = 0;
	/* Указатель оборотов ВОМ */
	if (K_VOM !=0)
	{
		if (TimerFr > TIME_OBNUL_FR_VOM)
		{
			if (flagFr.stc_FlagFr.pts == 0)
			{
				valuePTS = 0;
			}
			flagFr.stc_FlagFr.pts = 0;
			TimerFr = 0;
		}
		else
		{
			TimerFr++;
		}
		if (valuePTS)
			frequency = 16000000.0/((float)valuePTS/(float)(PASS_COUNTER_PTS_IRQ+1));	
		else
			frequency = 0;	
		Vom = (float)frequency * (float)K_VOM;
	}
	else
	{
		Vom = Tahom * KV2;
	}
	//Зажигание светодиодов
	if (Vom)
	{
		if (Vom < 750)
		{	
			// Зажигания сигнализатора "540"
			dataLED.stc_DataLED.led5 = 1;
			dataLED.stc_DataLED.led6 = 0;		
			if (Vom >= 320)
				dataLED.stc_DataLED.led4 = 1;
			else if (Vom < 310)
				dataLED.stc_DataLED.led4 = 0;
			if (Vom >= 420)
				dataLED.stc_DataLED.led3 = 1;
			else if (Vom < 410)
				dataLED.stc_DataLED.led3 = 0;
			if (Vom >= 500)
				dataLED.stc_DataLED.led2 = 1;
			else if (Vom < 490)
				dataLED.stc_DataLED.led2 = 0;
			if (Vom >= 580)
				dataLED.stc_DataLED.led1 = 1;
			else if (Vom < 570)
				dataLED.stc_DataLED.led1 = 0;
			if (Vom >= 650)
				dataLED.stc_DataLED.led0 = 1;
			else if (Vom < 640)
				dataLED.stc_DataLED.led0 = 0;
		}
		else
		{
			// Зажигания сигнализатора "1000"
			dataLED.stc_DataLED.led6 = 1;
			dataLED.stc_DataLED.led5 = 0;		
			if (Vom >= 750)
				dataLED.stc_DataLED.led4 = 1;
			else if (Vom < 740)
				dataLED.stc_DataLED.led4 = 0;
			if (Vom >= 850)
				dataLED.stc_DataLED.led3 = 1;
			else if (Vom < 840)
				dataLED.stc_DataLED.led3 = 0;
			if (Vom >= 950)
				dataLED.stc_DataLED.led2 = 1;
			else if (Vom < 940)
				dataLED.stc_DataLED.led2 = 0;
			if (Vom >= 1050)
				dataLED.stc_DataLED.led1 = 1;
			else if (Vom < 1040)
				dataLED.stc_DataLED.led1 = 0;
			if (Vom >= 1150)
				dataLED.stc_DataLED.led0 = 1;
			else if (Vom < 1140)
				dataLED.stc_DataLED.led0 = 0;		
		}
	}
	else
	{
		dataLED.u_dataLED = dataLED.u_dataLED & 0x80;
	}
	
	Sethc595(dataLED.u_dataLED);
}

void UpdateFUEL(void)
{
	static unsigned int TimerFr = 0;                       // Переменная таймера обнуленя частоты
	float frequency = 0;
	/* Указатель уровня топлива */
	if (TimerFr > TIME_OBNUL_FR_FUEL)
	{
		if (flagFr.stc_FlagFr.fuel == 0)
		{
			valueFUEL = 0;
		}
		flagFr.stc_FlagFr.fuel = 0;
		TimerFr = 0;
	}
	else
	{
		TimerFr++;
	}
	if (valueFUEL)
		frequency = 16000000.0/((float)valueFUEL/(float)(PASS_COUNTER_FUEL_IRQ+1));	
	else
		frequency = 0;	
	if (frequency > 500)	
		Fuel = (float)(frequency - 500) / 10.0;
	else
		Fuel = 0.0;
	if (Fuel > 100.0)	
		Fuel = 100.0;
}

void InitMode(en_gui_mode_t mode)
{
	Disable_simbols();
	m_enGuiMode = mode;
	switch (mode)
	{
		case Empty:
			ClearLCDLine();
			segment1(13);
			break;
			
		case mode_Motochasi:
			SIMBOL_H = 1;	
			break;
			
		case mode_Vom:			
			SIMBOL_min = 1;
			SIMBOL_m1 = 1;
			break;
			
		case mode_Fuel:
			if (V != 0)
				SIMBOL_I = 1;
			else
				InitMode(mode_Vbort);
			break;
		
		case mode_Vbort:
			if (J1939CtrBufer[VEP1].Available) 
			{
				SIMBOL_P4 = 1;
				SIMBOL_V = 1;
			}
			else
				InitMode(mode_FuelRate);
			break;
			
		case mode_FuelRate:
			if (J1939CtrBufer[LFE].Available)
			{
				SIMBOL_P4 = 1;
				ENABLE_L_H
			}
			else
				InitMode(mode_ProbegMotochNaOst);
			break;
			
		case mode_ProbegMotochNaOst:
			if (J1939CtrBufer[LFE].Available)
				SIMBOL_H = 1;
			else
				InitMode(mode_Motochasi);
			break;			
		
		case mode_ParamsZ:
			max_count_params = MAX_PARAMS_Z;
			current_num_params = max_count_params-1;
			SIMBOL_Z = 1;
			break;
			
		case mode_ParamsI:		
			max_count_params = MAX_PARAMS_I;
			current_num_params = max_count_params-1;		
			SIMBOL_P3 = 1;
			SIMBOL_I = 1;
			break;
			
		case mode_ParamsR:
			SIMBOL_R = 1;
			break;	
			
		case mode_ParamsK:
			max_count_params = MAX_PARAMS_K;
			current_num_params = max_count_params-1;		
			SIMBOL_P3 = 1;
			SIMBOL_K = 1;
			break;	
			
		case mode_ParamsKV2:
			max_count_params = MAX_PARAMS_KV2;
			current_num_params = max_count_params-1;		
			SIMBOL_P3 = 1;
			SIMBOL_K = 1;
			SIMBOL_V = 1;
			SIMBOL_2 = 1;
			break;
			
		case mode_ParamsZV:
			max_count_params = MAX_PARAMS_ZV;
			current_num_params = max_count_params-1;		
			SIMBOL_Z = 1;
			SIMBOL_V = 1;
			break;
			
		case mode_ParamsV:
			SIMBOL_V = 1;
			break;	
			
		case mode_ParamsT:
			SIMBOL_P4 = 1;
			SIMBOL_T = 1;
			SIMBOL_H = 1;
			break;		
		
		case mode_ErrorSpeedL:
			flagVisibleErrorLSpeed = 0;
			ENABLE_KM_H
			break;
			
		case mode_ErrorSpeedR:
			flagVisibleErrorRSpeed = 0;
			ENABLE_KM_H
			break;
			
		case mode_ErrorFuel:
			flagVisibleErrorFuel = 0;
			break;	
			
		case mode_ErrorCAN:
			flagVisibleErrorCAN = 0;
			break;						
								
		default:
			break;		
	}
}

void ButoonPress(unsigned int Num_button, unsigned char enState)
{
	unsigned long num_str = 0;
	
	switch (Num_button)
	{
		case BUTTON_ID_B1:
			switch (enState)
			{
				case 1:
					switch (m_enGuiMode)
					{
						case mode_Motochasi:
							InitMode(mode_Vom);
							break; 
						case mode_Vom:
							InitMode(mode_Fuel);
							break;
						case mode_Fuel:
							InitMode(mode_Vbort);
							break;	
						case mode_Vbort:
							InitMode(mode_FuelRate);
							break;
						case mode_FuelRate:
							InitMode(mode_ProbegMotochNaOst);
							break;									
						case mode_ProbegMotochNaOst:
							InitMode(mode_Motochasi);
							break;	
							
						case mode_ErrorSpeedL:
						case mode_ErrorSpeedR:
							if (ErrorBuf.stc_Error.no_fuel)
								InitMode(mode_ErrorFuel);
							else
							#if (DEVICE_TYPE == KD8105_CAN)
							if (ErrorBuf.stc_Error.no_can)
								InitMode(mode_ErrorCAN);
							else
							#endif
							{
								//flagVisibleError = 0;
								InitMode(m_enGuiMode_before_emty);	
							}
							break;
						case mode_ErrorFuel:
							#if (DEVICE_TYPE == KD8105_CAN)
							if (ErrorBuf.stc_Error.no_can)
								InitMode(mode_ErrorCAN);
							else
							#endif
							{
								//flagVisibleError = 0;
								InitMode(m_enGuiMode_before_emty);	
							}
							break;
							
						case mode_ErrorCAN:
							//flagVisibleError = 0;
							InitMode(m_enGuiMode_before_emty);
							break;	
							
						case mode_ParamsZ:
						case mode_ParamsI:
						case mode_ParamsR:
						case mode_ParamsK:
						case mode_ParamsKV2:
						case mode_ParamsZV:
						case mode_ParamsV:
							Time_return_menu = TIME_RETURN_MENU;
							Timer_Start(TIMER_ID_WAIT, 350, FALSE, WaitDualKeyPres);	
							if (flag_dual_press)
							{
								flag_edit_params = ~flag_edit_params;
								flag_dual_press = 0;
								num_edit_simbol = 0;
								
								if (flag_edit_params)
								{
									switch (m_enGuiMode)
									{
										case mode_ParamsZ:
											num_str = ToBCD(Z);
											N[3] = (num_str>>12)&0xF;
											N[2] = (num_str>>8)&0xF;
											N[1] = (num_str>>4)&0xF;
											N[0] = num_str & 0xF;
											break;
										
										case mode_ParamsI:
											num_str = ToBCD(I*1000);
											N[3] = (num_str>>12)&0xF;
											N[2] = (num_str>>8)&0xF;
											N[1] = (num_str>>4)&0xF;
											N[0] = num_str & 0xF;
											break;
										
										case mode_ParamsR:
											num_str = ToBCD(R);
											N[3] = (num_str>>12)&0xF;
											N[2] = (num_str>>8)&0xF;
											N[1] = (num_str>>4)&0xF;
											N[0] = num_str & 0xF;
											break;		
										
										case mode_ParamsK:
											num_str = ToBCD(K*1000);
											N[3] = (num_str>>12)&0xF;
											N[2] = (num_str>>8)&0xF;
											N[1] = (num_str>>4)&0xF;
											N[0] = num_str & 0xF;
											break;		
										
										case mode_ParamsKV2:
											num_str = ToBCD(KV2*1000);
											N[3] = (num_str>>12)&0xF;
											N[2] = (num_str>>8)&0xF;
											N[1] = (num_str>>4)&0xF;
											N[0] = num_str & 0xF;
											break;	
										
										case mode_ParamsZV:
											num_str = ToBCD(ZV);
											N[3] = (num_str>>12)&0xF;
											N[2] = (num_str>>8)&0xF;
											N[1] = (num_str>>4)&0xF;
											N[0] = num_str & 0xF;
											break;		
										
										case mode_ParamsV:
											num_str = ToBCD(V);
											N[3] = (num_str>>12)&0xF;
											N[2] = (num_str>>8)&0xF;
											N[1] = (num_str>>4)&0xF;
											N[0] = num_str & 0xF;
											break;
										default: 
											break;
									}
									
								}
								else
								{
									switch (m_enGuiMode)
									{
										case mode_ParamsZ:
											Z = N[1]*10 + N[0];
											params_Z[sizeof(params_Z)/sizeof(params_Z[0])] = Z;
											break;
										
										case mode_ParamsI:
											I = (float)(N[0] + N[1]*10 + N[2]*100 + N[3]*1000)/1000;
											params_I[sizeof(params_I)/sizeof(params_I[0])] = I*1000;
											break;
										
										case mode_ParamsR:
											R = (float)(N[0] + N[1]*10 + N[2]*100 + N[3]*1000);
											break;		
										
										case mode_ParamsK:
											K = (float)(N[0] + N[1]*10 + N[2]*100 + N[3]*1000)/1000;
											params_K[sizeof(params_K)/sizeof(params_K[0])] = K*1000;
											break;		
										
										case mode_ParamsKV2:
											KV2 = (float)(N[0] + N[1]*10 + N[2]*100 + N[3]*1000)/1000;
											params_KV2[sizeof(params_KV2)/sizeof(params_KV2[0])] = KV2*1000;
											break;	
										
										case mode_ParamsZV:
											ZV = N[1]*10 + N[0];
											params_ZV[sizeof(params_ZV)/sizeof(params_ZV[0])] = ZV;
											break;		
										
										case mode_ParamsV:
											V = (float)(N[0] + N[1]*10 + N[2]*100 + N[3]*1000)/1000;
											break;
											
										default: 
											break;										
									}	
								}
							}
							else
							{
								flag_dual_press = 1;
							}							
							break;
					}
					break;
					
				case 2:
					break;
					
				default:
					break;
			}
			break;
			
		case BUTTON_ID_B2:
			switch (enState)
			{
				case 1:
					switch (m_enGuiMode)
					{
						case mode_Motochasi:
						case mode_Vom:
						case mode_Fuel:
							m_enGuiMode_before_emty = m_enGuiMode;
							InitMode(mode_ParamsZ);
							break;
							
						case mode_ParamsZ:
							Time_return_menu = TIME_RETURN_MENU;
							if (flag_edit_params)
							{
								num_edit_simbol++;
								if (num_edit_simbol > 1)
									num_edit_simbol = 0;
							}
							else
							{
								params_Z[max_count_params-1] = Z;
								InitMode(mode_ParamsI);
							}
							break;
							
						case mode_ParamsI:
							Time_return_menu = TIME_RETURN_MENU;
							if (flag_edit_params)
							{
								num_edit_simbol++;
								if (num_edit_simbol > 3)
									num_edit_simbol = 0;
							}					
							else
							{	
								params_I[max_count_params-1] = I*1000;
								InitMode(mode_ParamsR);
							}
							break;
							
							
						case mode_ParamsR:
							Time_return_menu = TIME_RETURN_MENU;
							if (flag_edit_params)
							{
								num_edit_simbol++;
								if (num_edit_simbol > 3)
									num_edit_simbol = 0;
							}		
							else			
								InitMode(mode_ParamsK);
							break;							
							
						case mode_ParamsK:
							Time_return_menu = TIME_RETURN_MENU;
							if (flag_edit_params)
							{
								num_edit_simbol++;
								if (num_edit_simbol > 3)
									num_edit_simbol = 0;
							}	
							else
							{					
								params_K[max_count_params-1] = K*1000;
								InitMode(mode_ParamsKV2);
							}
							break;
							
						case mode_ParamsKV2:
							Time_return_menu = TIME_RETURN_MENU;
							if (flag_edit_params)
							{
								num_edit_simbol++;
								if (num_edit_simbol > 3)
									num_edit_simbol = 0;
							}
							else
							{
								params_KV2[max_count_params-1] = KV2*1000;
								InitMode(mode_ParamsZV);
							}
							break;	
							
						case mode_ParamsZV:
							Time_return_menu = TIME_RETURN_MENU;
							if (flag_edit_params)
							{
								num_edit_simbol++;
								if (num_edit_simbol > 1)
									num_edit_simbol = 0;
							}
							else						
							{
								params_ZV[max_count_params-1] = ZV;
								InitMode(mode_ParamsV);
							}
							break;
							
						case mode_ParamsV:
							Time_return_menu = TIME_RETURN_MENU;
							if (flag_edit_params)
							{
								num_edit_simbol++;
								if (num_edit_simbol > 2)
									num_edit_simbol = 0;
							}	
							else					
								InitMode(mode_ParamsT);
							break;
					
						case mode_ParamsT:
							InitTestMode();
							break;
					}
					break;
					
				case 2:
					//NumToLCDStr(22);
					break;
					
				default:
					break;
			}
			break;
			
		case BUTTON_ID_B3:
			switch (enState)
			{
				case 1:
					switch (m_enGuiMode)
					{
						case mode_ParamsZ:
						case mode_ParamsI:
						case mode_ParamsK:
						case mode_ParamsKV2:
						case mode_ParamsZV:
						
						Time_return_menu = TIME_RETURN_MENU;
						if (flag_edit_params)
						{
							N[num_edit_simbol]++;
							if (N[num_edit_simbol] > 9)
								N[num_edit_simbol] = 0;
							switch (m_enGuiMode)
							{
								case mode_ParamsZ:
									if ((num_edit_simbol == 1) && (N[num_edit_simbol] > 6))
										N[num_edit_simbol] = 2;
									break;
								case mode_ParamsI:
									if ((num_edit_simbol == 3) && (N[num_edit_simbol] > 4))
										N[num_edit_simbol] = 1;
									break;
								case mode_ParamsK:
									if ((num_edit_simbol == 3) && (N[num_edit_simbol] > 4))
										N[num_edit_simbol] = 2;
									break;
								case mode_ParamsKV2:
									if ((num_edit_simbol == 3) && (N[num_edit_simbol] > 0))
										N[num_edit_simbol] = 0;
									break;
								case mode_ParamsZV:
									if ((num_edit_simbol == 3) && (N[num_edit_simbol] > 7))
										N[num_edit_simbol] = 1;
									break;
								default:
									break;
							}
						}
						else
						{
							current_num_params++;
							if (current_num_params > (max_count_params-1))
								current_num_params = 0;
							switch (m_enGuiMode)
							{
								case mode_ParamsZ:
									Z = params_Z[current_num_params];
									break;
									
								case mode_ParamsI:
									I = (float)params_I[current_num_params]/1000;
									break;
							
								case mode_ParamsK:
									K = (float)params_K[current_num_params]/1000;
									break;
						
								case mode_ParamsKV2:
									KV2 = (float)params_KV2[current_num_params]/1000;
									break;
							
								case mode_ParamsZV:
									ZV = params_ZV[current_num_params];
									break;								
							}							
						}
						break;
						
						case mode_ParamsR:
							Time_return_menu = TIME_RETURN_MENU;
							if (flag_edit_params)
							{
								N[num_edit_simbol]++;
								if (N[num_edit_simbol] > 9)
									N[num_edit_simbol] = 0;
								if ((num_edit_simbol == 3) && (N[num_edit_simbol] > 1))
										N[num_edit_simbol] = 0;
							}
							else
							{					
								R = ((R / 5) * 5) + 5;
								if (R > 1000)
									R = 400;
							}
							break;
							
						case mode_ParamsV:
							Time_return_menu = TIME_RETURN_MENU;
							if (flag_edit_params)
							{
								N[num_edit_simbol]++;
								if (N[num_edit_simbol] > 9)
									N[num_edit_simbol] = 0;
							}
							else
							{							
								V = ((V / 5) * 5) + 5;
								if (V > 600)
									V = 0;
							}
							break;
						
						default:
							break;						
					
					}
					break;
					
				case 2:
					NumToLCDStr(32);
					break;
					
				default:
					break;
			}
			break;
			
		default:
			break;								
	}
}

void WaitDualKeyPres(void)
{
	flag_dual_press = 0;
}



void LoadAllParams(void)
{
	unsigned char data[18];
	unsigned char ReadOk = 0;
	unsigned int temp = 0;
	unsigned char CRC = 0;
	unsigned char i = 0;
	
	for (i = 0; i < 9; i++)
	{
		temp = *(unsigned int __far*)(SA2+i*2);	
		data[i*2] = (temp >> 8) & 0xFF;
		data[i*2+1] = temp & 0xFF;		
	}	
	
	for (i = 0; i < 17; i++)
		CRC = CRC + data[i];
	
	CRC = ~CRC;
	// Если контрольная сумма совпадает 
	if (CRC == data[17])
	{
		ReadOk = 1;
	}
	else
	{
		// Если ошибка чтения то считываем дублирающие данные
		for (i = 0; i < 9; i++)
		{
			temp = *(unsigned int __far*)(SA3+i*2);	
			data[i*2] = (temp >> 8) & 0xFF;
			data[i*2+1] = temp & 0xFF;		
		} 
		ReadOk= 0;
	}
	

	R = ((unsigned int)data[0] << 8) | ((unsigned int)data[1]);
	if (R > 1000)
		R = 600;
		
	Z = data[2];
	if (Z > 69)
		Z = 69;
	
	I = (float)(((unsigned int)data[3] << 8) | ((unsigned int)data[4]))/1000;
	if (I > 4.0)
		I = 1.0;
		
	K = (float)(((unsigned int)data[5] << 8) | ((unsigned int)data[6]))/1000;
	if (K > 4.0)
		K = 2.36;

	KV2 = (float)(((unsigned int)data[7] << 8) | ((unsigned int)data[8]))/1000;
	if (KV2 > 0.6)
		KV2 = 0.6;	
		
	ZV = data[9];
	if (ZV > 78)
		ZV = 78;
		
	V = ((unsigned int)data[10] << 8) | ((unsigned int)data[11]);
	if (V > 600)
		V = 600;
		
	//km = LoadProbeg();
	Motochasi = ((unsigned long)data[12] << 24) | 
	 		((unsigned long)data[13] << 16) | 
	  		((unsigned long)data[14] << 8) | 
	   		((unsigned long)data[15]);
	if (Motochasi == 0xFFFFFFFFUL)
		Motochasi = 0;
		

	if (ReadOk)
		//Сохраняем значения
		SaveAllParamsSecondary();
}

unsigned int SaveAllParams(void)
{
	unsigned int result = 0;
	unsigned char data[18];
	unsigned char CRC = 0;
	unsigned int temp = 0;
	unsigned char i = 0;

	// Сохранение R
	data[0] = (R >> 8) & 0xFF;
	data[1] =  R & 0xFF;
	
	//Сохранение Z
	data[2] = Z & 0xFF;
	
	// Сохранение I
	temp = (unsigned int)(I * 1000);
	data[3] = (temp >> 8) & 0xFF;
	data[4] =  temp & 0xFF;
	
	// Сохранение K
	temp = (unsigned int)(K * 1000);
	data[5] = (temp >> 8) & 0xFF;
	data[6] =  temp & 0xFF;	
	
	// Сохранение KV2
	temp = (unsigned int)(KV2 * 1000);
	data[7] = (temp >> 8) & 0xFF;
	data[8] =  temp & 0xFF;
	
	// Сохранение ZV
	data[9] = ZV & 0xFF;
	
	// Сохранение V
	data[10] = (V >> 8) & 0xFF;
	data[11] =  V & 0xFF;	
	
	//Сохранение моточасы 4 байта
	data[12] = ((unsigned long)Motochasi >> 24) & 0xFF; 	
	data[13] = ((unsigned long)Motochasi >> 16) & 0xFF; 
	data[14] = ((unsigned long)Motochasi >> 8) & 0xFF; 
	data[15] = ((unsigned long)Motochasi) & 0xFF; 
	
	// РЕзерв
	data[16] = 0;

	//CRC
	for (i = 0; i < 17; i++)
		CRC = CRC + data[i];
		
	data[17] = ~CRC;	

	Data_flash_SectorErase(SA2);					
	for (i = 0; i < 9; i++)
	{
		temp = ((unsigned int)data[2*i] << 8) | (unsigned int)data[2*i+1];
		Data_Flash_write(SA2+i*2, temp);
		while (temp != *(unsigned int __far*)(SA2+i*2));	
	}
		
	return result;
}

unsigned int SaveAllParamsSecondary(void)
{
	unsigned int result = 0;
	unsigned char data[18];
	unsigned char CRC = 0;
	unsigned int temp = 0;
	unsigned char i = 0;

	// Сохранение R
	data[0] = (R >> 8) & 0xFF;
	data[1] =  R & 0xFF;
	
	//Сохранение Z
	data[2] = Z & 0xFF;
	
	// Сохранение I
	temp = (unsigned int)(I * 1000);
	data[3] = (temp >> 8) & 0xFF;
	data[4] =  temp & 0xFF;
	
	// Сохранение K
	temp = (unsigned int)(K * 1000);
	data[5] = (temp >> 8) & 0xFF;
	data[6] =  temp & 0xFF;	
	
	// Сохранение KV2
	temp = (unsigned int)(KV2 * 1000);
	data[7] = (temp >> 8) & 0xFF;
	data[8] =  temp & 0xFF;
	
	// Сохранение ZV
	data[9] = ZV & 0xFF;
	
	// Сохранение V
	data[10] = (V >> 8) & 0xFF;
	data[11] =  V & 0xFF;	
	
	//Сохранение моточасы 4 байта
	data[12] = ((unsigned long)Motochasi >> 24) & 0xFF; 	
	data[13] = ((unsigned long)Motochasi >> 16) & 0xFF; 
	data[14] = ((unsigned long)Motochasi >> 8) & 0xFF; 
	data[15] = ((unsigned long)Motochasi) & 0xFF; 
	
	// РЕзерв
	data[16] = 0;

	//CRC
	for (i = 0; i < 17; i++)
		CRC = CRC + data[i];
		
	data[17] = ~CRC;		

	Data_flash_SectorErase(SA3);					
	for (i = 0; i < 9; i++)
	{
		temp = ((unsigned int)data[2*i] << 8) | (unsigned int)data[2*i+1];
		Data_Flash_write(SA3+i*2, temp);
		while (temp != *(unsigned int __far*)(SA3+i*2));	
	}
		
	return result;
}

