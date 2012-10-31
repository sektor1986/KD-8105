/************************************************************************/
/*               (C) Fujitsu Semiconductor Europe GmbH (FSEU)           */
/*                                                                      */
/* The following software deliverable is intended for and must only be  */
/* used for reference and in an evaluation laboratory environment.      */
/* It is provided on an as-is basis without charge and is subject to    */
/* alterations.                                                         */
/* It is the user’s obligation to fully test the software in its        */
/* environment and to ensure proper functionality, qualification and    */
/* compliance with component specifications.                            */
/*                                                                      */
/* In the event the software deliverable includes the use of open       */
/* source components, the provisions of the governing open source       */
/* license agreement shall apply with respect to such software          */
/* deliverable.                                                         */
/* FSEU does not warrant that the deliverables do not infringe any      */
/* third party intellectual property right (IPR). In the event that     */
/* the deliverables infringe a third party IPR it is the sole           */
/* responsibility of the customer to obtain necessary licenses to       */
/* continue the usage of the deliverable.                               */
/*                                                                      */
/* To the maximum extent permitted by applicable law FSEU disclaims all */
/* warranties, whether express or implied, in particular, but not       */
/* limited to, warranties of merchantability and fitness for a          */
/* particular purpose for which the deliverable is not designated.      */
/*                                                                      */
/* To the maximum extent permitted by applicable law, FSEU’s liability  */
/* is restricted to intentional misconduct and gross negligence.        */
/* FSEU is not liable for consequential damages.                        */
/*                                                                      */
/* (V1.5)                                                               */
/************************************************************************/

/** \file main.c
 **
 ** Main application
 ** See README.TXT for project description and disclaimer.
 **
 ** History:
 **   - 2006-10-06  1.00  JWa  First version
 *****************************************************************************/

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include "mcu.h"
#include "base_types.h"
#include "vectors.h"
#include "options.h"
#include "driver_update.h"
#include "timer.h"
//#include "icu.h"
#include "adc.h"
#include "lcd.h"
#include "smc.h"
#include "74hc595.h"
//#include "flash.h"
#include "button.h"
#include "backlight.h"
#include "rtc.h"
#include "can.h"
#include "j1939.h"
#include "uart.h"
#include "mb96670_smc_zpd/mb96670_smc_zpd.h"


#if (SMC_TYPE == SMC_TYPE_R200)
// --- ZPD values R200-----------------------------------
	#define STEP_BLINDING_TIME_US		1500
	#define STEP_SAMPLING_TIME_US		7000
	#define ADC_SAMPLE_COUNT			50
	#define THRESHOLD					30
	#define DIRECTION					true

#elif (SMC_TYPE == SMC_TYPE_WCHINA)
// White china
	#define STEP_BLINDING_TIME_US		1000
	#define STEP_SAMPLING_TIME_US		20000
	#define ADC_SAMPLE_COUNT			50
	#define THRESHOLD					500
	#define DIRECTION					true
#else
// VID23
	#define STEP_BLINDING_TIME_US		1000
	#define STEP_SAMPLING_TIME_US		20000
	#define ADC_SAMPLE_COUNT			50
	#define THRESHOLD					400
	#define DIRECTION					false
#endif /* SMC_TYPE */

#define MAX_STEP_COUNT				1000
//#define MIN_CONNECTION_CHECK_LEVEL	164
//#define MAX_CONNECTION_CHECK_LEVEL	203

#define MIN_CONNECTION_CHECK_LEVEL	50
#define MAX_CONNECTION_CHECK_LEVEL	180

#define START_STEP					1
#define POST_STALL_STEPS			0

/*********************@GLOBAL_VARIABLES_START*******************/

STRUCT_SMC_ZPD_SETTINGS gstcZPDSettings = {	{ 0, 0, 0, 0 }, THRESHOLD, MAX_STEP_COUNT, MAX_CONNECTION_CHECK_LEVEL, MIN_CONNECTION_CHECK_LEVEL, START_STEP, DIRECTION, POST_STALL_STEPS };
								
#define STEPPER_COUNT	2					
STRUCT_SMC_ZPD_STALL_DATA gastcStallData[STEPPER_COUNT];
unsigned char aucStartStep[STEPPER_COUNT] = {START_STEP, START_STEP};

unsigned char gaaucADSampleBuffer[300][STEPPER_COUNT];

unsigned long gulStepBlindingTimeUs = STEP_BLINDING_TIME_US;
unsigned long gulStepSamplingTimeUs = STEP_SAMPLING_TIME_US;
unsigned short gusADCSampleCount = ADC_SAMPLE_COUNT;

STRUCT_SMC_ZPD_INFO gstcInfo;

bool gbStepperRunning = false;
bool gbStallDetected = false;

unsigned char gaucUARTBuffer[250];
unsigned char gucLength;

bool gbOutputStallData = false;

void SMC_ZPD_StallDetected(void);
void StopStepper(void);

void ZPD_Init(void);
/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/

/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/
static void ButtonCallback(uint16_t u16ButtonId, en_button_state_t enState);
/**
 ******************************************************************************
 ** Main application to control the program flow
 *****************************************************************************/
void main(void)
{
    // Initialize all interrupt levels of resources
    Vectors_InitIrqLevels();
    // Allow all interrupt levels
    __set_il(7);
    // Enable interrupts
    __EI();
 
 	InitLCD();
 	Init74hc595();
 	#if ((SMC_TYPE != SMC_TYPE_R200) && (ZPD == ZPD_ENABLE))
 		ZPD_Init();	
 		//Ожидание окончания ZPD
 		while (m_enSmcMode == Zpd)
 		{
			WDTCP = 0x00;
 		}
 	#else 	
	 	m_enSmcMode = NormalDriving;
	#endif

	InitSMC(250);						// Таймер SMC 25*2=40mks				
	Timer_Init();
	InitADC();
//	InitRTC();
	
// Если двигатель R200 или ZPD не активно
	#if ((SMC_TYPE == SMC_TYPE_R200) || (ZPD == ZPD_DISABLE))	
		ZeroPosSMC();					// Возврат стрелок на 0
		Timer_Wait(TIMER_ID_MAIN, 2000, TRUE);  
	#endif
	
	ClearPosSMC();						// Установка нулевых позиций для моторов
	
	InitBacklight();	
	DriverInit();
	InitICU(); 


	Button_Init(ButtonCallback);
	CAN_Init();
	J1939_init();
	InitUsart0();
	
//	if (Button_GetCurrentButtonState(BUTTON_ID_B1) == StateLow)
//		SetModePass();	
    
    while(1)
    { 
    	WDTCP = 0x00;    	
    	Timer_Main();	      
    }   
}


static void ButtonCallback(uint16_t u16ButtonId, en_button_state_t enState)
{
	char result = 0;	
	
	if (enState == StateLow)
	{
		return;
	}
    
	if (enState == StateHigh)
	{
		switch (u16ButtonId)
		{
			case BUTTON_ID_B1:                     // Режим индикации, короткое нажатие
				ButoonPress(BUTTON_ID_B1, 1); 
				break;
				
			case BUTTON_ID_B2:                     // Выбор параметра, короткое нажитие
				ButoonPress(BUTTON_ID_B2, 1); 
				break;
				
			case BUTTON_ID_B3:                     // Значение параметра, короткое нажатие
				ButoonPress(BUTTON_ID_B3, 1); 
				break;								
        
			default:
				break;
		}	
    }    
    
	if (enState == StateLong)  
	{      
		switch (u16ButtonId)
		{
			case BUTTON_ID_B1:
				ButoonPress(BUTTON_ID_B1, 2);     // Режим индикации, длинное нажатие
				break;
        
			case BUTTON_ID_B2:                    // Выбор параметра, длиное нажатие
				ButoonPress(BUTTON_ID_B2, 2); 
				break;     
				
			case BUTTON_ID_B3:                   // Значение параметра, длинное нажатие
				ButoonPress(BUTTON_ID_B3, 1); 
				break;	
							   
			default:
				break;
		}
    }	
}

void ZPD_Init(void)
{
	ENUM_SMC_ZPD_ERRORS Error;
    // Initialize MB96300_SCM_ZPD
	Error = SMC_ZPD_Init((unsigned char*)gaaucADSampleBuffer, sizeof(gaaucADSampleBuffer), SMC_ZPD_StallDetected);
	SMC_ZPD_SetADCSampleCount(gusADCSampleCount);
	SMC_ZPD_SetStepBlindingTime(gulStepBlindingTimeUs);
	SMC_ZPD_SetStepSamplingTime(gulStepSamplingTimeUs);	
	
	gstcZPDSettings.aucADCChannel[0] = gstcZPDSettings.aucADCChannel[2] = 19;
	gstcZPDSettings.aucADCChannel[1] = gstcZPDSettings.aucADCChannel[3] = 17;
	gstcZPDSettings.ucStartStep = aucStartStep[0];
	SMC_ZPD_EnableSMC(0, &gstcZPDSettings);	// SMC0		
	
	gstcZPDSettings.aucADCChannel[0] = gstcZPDSettings.aucADCChannel[2] = 23;
	gstcZPDSettings.aucADCChannel[1] = gstcZPDSettings.aucADCChannel[3] = 21;
	gstcZPDSettings.ucStartStep = aucStartStep[1];
	SMC_ZPD_EnableSMC(1, &gstcZPDSettings);	// SMC1	
	
	gbStallDetected = false;
	gbStepperRunning = true;
	
	Error = SMC_ZPD_Start();
}

void SMC_ZPD_StallDetected(void)
{
	unsigned int s;
	
	gbStallDetected = true;
	StopStepper();

	for (s=0; s<STEPPER_COUNT; s++)
	{
		SMC_ZPD_GetStallData(s, &gastcStallData[s]);
		aucStartStep[s] = gastcStallData[s].ucCurrentStep;
	}
	
	gbOutputStallData = true;
}

void StopStepper(void)
{
	SMC_ZPD_Stop();
	gbStepperRunning = false;
}

