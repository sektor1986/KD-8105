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
/** \file vectors.c
 **
 ** Interrupt level (priority) setting and interrupt vector definition
 **
 ** History:
 **   - 2005-04-31  1.00  UMa  Initial Version
 **   - 2005-11-08  1.01  MSt  SWB Mondeb switch for ICR00 Register added
 **   - 2006-02-27  1.02  UMa  added comment in DefaultIRQHandler 
 **   - 2006-03-17  1.03  UMa  comment out ICR01
 **   - 2006-07-28  1.04  UMa  changed comment
 **   - 2006-10-06  1.05  UMa  changed DefaultIRQHandler
 **   - 2010-03-19  1.06  JWa  adapt to new file and coding rules
 *****************************************************************************/

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include "mcu.h"
#include "base_types.h"
#include "vectors.h"
#include "mb96670_smc_zpd/mb96670_smc_zpd.h"
#include "icu.h"
#include "backlight.h"
//#include "icu.h"

//#include "..."  // <<< include your header files with ISR prototypes here

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

#define MIN_ICR  12          // Vector table definition below starts with vector 11 but this is NMI with fixed priority
#define MAX_ICR  147
#define DEFAULT_ILM_MASK 7   // 7: interrupt disabled


// Interrupt vector definiton
//__interrupt void SMC_IRQ (void);
__interrupt void Timer_Isr_MainClockTimer(void);
//__interrupt void ADC_IRQ(void);
__interrupt void ISR_CAN0(void);
__interrupt void RX_USART0(void);
// Use the following statements to define the interrupt vector table
// i.e. add your interrupt handlers here (ensure to define the ISR prototype,
// e.g. by adding the appropriate header file above).
// All resource related vectors are predefined. Remaining software interrupts
// can be added here as well.
//
// Refer to the device specific datasheet for the available interrupts in your device!

#pragma	intvect	Vectors_Isr_DefaultHandler 11	///< Non-Maskable Interrupt
#pragma	intvect	Vectors_Isr_DefaultHandler 12	///< Delayed Interrupt
#pragma	intvect	Vectors_Isr_DefaultHandler 13	///< RC Timer
#pragma	intvect	Timer_Isr_MainClockTimer   14	///< Main Clock Timer
#pragma	intvect	Vectors_Isr_DefaultHandler 15	///< Sub Clock Timer
#pragma	intvect	Vectors_Isr_DefaultHandler 16	///< LVD
#pragma	intvect	EI0_IRQ                    17	///< External Interrupt 0
#pragma	intvect	EI1_IRQ                    18	///< External Interrupt 1
#pragma	intvect	EI2_IRQ                    19	///< External Interrupt 2
#pragma	intvect	EI3_IRQ                    20	///< External Interrupt 3
#pragma	intvect	Vectors_Isr_DefaultHandler 21	///< External Interrupt 4
#pragma	intvect	Vectors_Isr_DefaultHandler 22	///< External Interrupt 5
#pragma	intvect	Vectors_Isr_DefaultHandler 23	///< External Interrupt 6
#pragma	intvect	Vectors_Isr_DefaultHandler 24	///< External Interrupt 7
#pragma	intvect	Vectors_Isr_DefaultHandler 25	///< External Interrupt 8
#pragma	intvect	Vectors_Isr_DefaultHandler 26	///< External Interrupt 9
#pragma	intvect	Vectors_Isr_DefaultHandler 27	///< External Interrupt 10
#pragma	intvect	Vectors_Isr_DefaultHandler 28	///< External Interrupt 11
#pragma	intvect	Vectors_Isr_DefaultHandler 29	///< External Interrupt 12
#pragma	intvect	Vectors_Isr_DefaultHandler 30	///< External Interrupt 13
#pragma	intvect	Vectors_Isr_DefaultHandler 31	///< External Interrupt 14
#pragma	intvect	Vectors_Isr_DefaultHandler 32	///< External Interrupt 15
#pragma	intvect	ISR_CAN0                   33	///< CAN Controller 0
#pragma	intvect	Vectors_Isr_DefaultHandler 34	///< CAN Controller 1
#pragma	intvect	Vectors_Isr_DefaultHandler 35	///< CAN Controller 2
#pragma	intvect	Vectors_Isr_DefaultHandler 36	///< Reserved
#pragma	intvect	Vectors_Isr_DefaultHandler 37	///< Reserved
#pragma	intvect	Vectors_Isr_DefaultHandler 38	///< Programmable Pulse Generator 0
#pragma	intvect	Vectors_Isr_DefaultHandler 39	///< Programmable Pulse Generator 1
#pragma	intvect	Vectors_Isr_DefaultHandler 40	///< Programmable Pulse Generator 2
#pragma	intvect	Vectors_Isr_DefaultHandler 41	///< Programmable Pulse Generator 3
#pragma	intvect	Vectors_Isr_DefaultHandler 42	///< Programmable Pulse Generator 4
#pragma	intvect	Vectors_Isr_DefaultHandler 43	///< Programmable Pulse Generator 5
#pragma	intvect	Vectors_Isr_DefaultHandler 44	///< Programmable Pulse Generator 6
#pragma	intvect	Vectors_Isr_DefaultHandler 45	///< Programmable Pulse Generator 7
#pragma	intvect	Vectors_Isr_DefaultHandler 46	///< Programmable Pulse Generator 8
#pragma	intvect	Vectors_Isr_DefaultHandler 47	///< Programmable Pulse Generator 9
#pragma	intvect	Vectors_Isr_DefaultHandler 48	///< Programmable Pulse Generator 10
#pragma	intvect	Vectors_Isr_DefaultHandler 49	///< Programmable Pulse Generator 11
#pragma	intvect	Vectors_Isr_DefaultHandler 50	///< Programmable Pulse Generator 12
#pragma	intvect	Vectors_Isr_DefaultHandler 51	///< Programmable Pulse Generator 13
#pragma	intvect	Vectors_Isr_DefaultHandler 52	///< Programmable Pulse Generator 14
#pragma	intvect	Vectors_Isr_DefaultHandler 53	///< Programmable Pulse Generator 15
#pragma	intvect	Vectors_Isr_DefaultHandler 54	///< Programmable Pulse Generator 16
#pragma	intvect	Vectors_Isr_DefaultHandler 55	///< Programmable Pulse Generator 17
#pragma	intvect	Vectors_Isr_DefaultHandler 56	///< Programmable Pulse Generator 18
#pragma	intvect	Vectors_Isr_DefaultHandler 57	///< Programmable Pulse Generator 19
#pragma	intvect	Vectors_Isr_DefaultHandler 58	///< Reload Timer 0
#pragma	intvect	Backlight_IRQ              59	///< Reload Timer 1
#pragma	intvect	ISR_RLT2                   60	///< Reload Timer 2 SMC_IRQ  ISR_RLT2 
#pragma	intvect	Vectors_Isr_DefaultHandler 61	///< Reload Timer 3
#pragma	intvect	Vectors_Isr_DefaultHandler 62	///< Reload Timer 4
#pragma	intvect	Vectors_Isr_DefaultHandler 63	///< Reload Timer 5
#pragma	intvect	Vectors_Isr_DefaultHandler 64	///< Reload Timer 6 - dedicated for PPG
#pragma	intvect	Vectors_Isr_DefaultHandler 65	///< Input Capture Unit 0
#pragma	intvect	ICU1_IRQ                   66	///< Input Capture Unit 1
#pragma	intvect	Vectors_Isr_DefaultHandler 67	///< Input Capture Unit 2
#pragma	intvect	Vectors_Isr_DefaultHandler 68	///< Input Capture Unit 3
#pragma	intvect	Vectors_Isr_DefaultHandler 69	///< Input Capture Unit 4
#pragma	intvect	Vectors_Isr_DefaultHandler 70	///< Input Capture Unit 5
#pragma	intvect	Vectors_Isr_DefaultHandler 71	///< Input Capture Unit 6
#pragma	intvect	Vectors_Isr_DefaultHandler 72	///< Input Capture Unit 7
#pragma	intvect	Vectors_Isr_DefaultHandler 73	///< Input Capture Unit 8
#pragma	intvect	Vectors_Isr_DefaultHandler 74	///< Input Capture Unit 9
#pragma	intvect	Vectors_Isr_DefaultHandler 75	///< Input Capture Unit 10
#pragma	intvect	Vectors_Isr_DefaultHandler 76	///< Input Capture Unit 11
#pragma	intvect	Vectors_Isr_DefaultHandler 77	///< Output Compare Unit 0
#pragma	intvect	Vectors_Isr_DefaultHandler 78	///< Output Compare Unit 1
#pragma	intvect	Vectors_Isr_DefaultHandler 79	///< Output Compare Unit 2
#pragma	intvect	Vectors_Isr_DefaultHandler 80	///< Output Compare Unit 3
#pragma	intvect	Vectors_Isr_DefaultHandler 81	///< Output Compare Unit 4
#pragma	intvect	Vectors_Isr_DefaultHandler 82	///< Output Compare Unit 5
#pragma	intvect	Vectors_Isr_DefaultHandler 83	///< Output Compare Unit 6
#pragma	intvect	Vectors_Isr_DefaultHandler 84	///< Output Compare Unit 7
#pragma	intvect	Vectors_Isr_DefaultHandler 85	///< Output Compare Unit 8
#pragma	intvect	Vectors_Isr_DefaultHandler 86	///< Output Compare Unit 9
#pragma	intvect	Vectors_Isr_DefaultHandler 87	///< Output Compare Unit 10
#pragma	intvect	Vectors_Isr_DefaultHandler 88	///< Output Compare Unit 11
#pragma	intvect	FRT0_IRQ                   89	///< Free Running Timer 0
#pragma	intvect	Vectors_Isr_DefaultHandler 90	///< Free Running Timer 1
#pragma	intvect	Vectors_Isr_DefaultHandler 91	///< Free Running Timer 2
#pragma	intvect	Vectors_Isr_DefaultHandler 92	///< Free Running Timer 3
#pragma	intvect	Vectors_Isr_DefaultHandler 93	///< Real Timer Clock
#pragma	intvect	Vectors_Isr_DefaultHandler 94	///< Clock Calibration Unit 
#pragma	intvect	Vectors_Isr_DefaultHandler 95	///< Sound Generator 0
#pragma	intvect	Vectors_Isr_DefaultHandler 96	///< I2C interface 0
#pragma	intvect	Vectors_Isr_DefaultHandler 97	///< I2C interface 1
#pragma	intvect	ISR_ADC	                   98	///< A/D Converter ADC_IRQ    
#pragma	intvect	Vectors_Isr_DefaultHandler 99	///< Reserved
#pragma	intvect	Vectors_Isr_DefaultHandler 100	///< Reserved
#pragma	intvect	RX_USART0                  101	///< LIN USART 0 RX
#pragma	intvect	Vectors_Isr_DefaultHandler 102	///< LIN USART 0 TX
#pragma	intvect	Vectors_Isr_DefaultHandler 103	///< LIN USART 1 RX
#pragma	intvect	Vectors_Isr_DefaultHandler 104	///< LIN USART 1 TX
#pragma	intvect	Vectors_Isr_DefaultHandler 105	///< LIN USART 2 RX
#pragma	intvect	Vectors_Isr_DefaultHandler 106	///< LIN USART 2 TX
#pragma	intvect	Vectors_Isr_DefaultHandler 107	///< LIN USART 3 RX
#pragma	intvect	Vectors_Isr_DefaultHandler 108	///< LIN USART 3 TX
#pragma	intvect	Vectors_Isr_DefaultHandler 109	///< LIN USART 4 RX
#pragma	intvect	Vectors_Isr_DefaultHandler 110	///< LIN USART 4 TX
#pragma	intvect	Vectors_Isr_DefaultHandler 111	///< LIN USART 5 RX
#pragma	intvect	Vectors_Isr_DefaultHandler 112	///< LIN USART 5 TX
#pragma	intvect	Vectors_Isr_DefaultHandler 113	///< LIN USART 6 RX
#pragma	intvect	Vectors_Isr_DefaultHandler 114	///< LIN USART 6 TX
#pragma	intvect	Vectors_Isr_DefaultHandler 115	///< LIN USART 7 RX
#pragma	intvect	Vectors_Isr_DefaultHandler 116	///< LIN USART 7 TX
#pragma	intvect	Vectors_Isr_DefaultHandler 117	///< LIN USART 8 RX
#pragma	intvect	Vectors_Isr_DefaultHandler 118	///< LIN USART 8 TX
#pragma	intvect	Vectors_Isr_DefaultHandler 119	///< LIN USART 9 RX
#pragma	intvect	Vectors_Isr_DefaultHandler 120	///< LIN USART 9 TX
#pragma	intvect	Vectors_Isr_DefaultHandler 121	///< Sound Generator 1
#pragma	intvect	Vectors_Isr_DefaultHandler 122	///< Reserved
#pragma	intvect	Vectors_Isr_DefaultHandler 123	///< Reserved
#pragma	intvect	Vectors_Isr_DefaultHandler 124	///< Reserved
#pragma	intvect	Vectors_Isr_DefaultHandler 125	///< Reserved
#pragma	intvect	Vectors_Isr_DefaultHandler 126	///< Reserved
#pragma	intvect	Vectors_Isr_DefaultHandler 127	///< Reserved
#pragma	intvect	Vectors_Isr_DefaultHandler 128	///< Reserved
#pragma	intvect	Vectors_Isr_DefaultHandler 129	///< Reserved
#pragma	intvect	Vectors_Isr_DefaultHandler 130	///< Reserved
#pragma	intvect	Vectors_Isr_DefaultHandler 131	///< Reserved
#pragma	intvect	Vectors_Isr_DefaultHandler 132	///< Reserved
#pragma	intvect	Vectors_Isr_DefaultHandler 133	///< Dual operation Flash A
#pragma	intvect	Vectors_Isr_DefaultHandler 134	///< Dual operation Flash B
#pragma	intvect	Vectors_Isr_DefaultHandler 135	///< Reserved
#pragma	intvect	Vectors_Isr_DefaultHandler 136	///< Reserved
#pragma	intvect	Vectors_Isr_DefaultHandler 137	///< Quad Possition/Revolution counter 0
#pragma	intvect	Vectors_Isr_DefaultHandler 138	///< Quad Possition/Revolution counter 1
#pragma	intvect	Vectors_Isr_DefaultHandler 139	///< A/D Converter 0 - Range Comparator
#pragma	intvect	Vectors_Isr_DefaultHandler 140	///< A/D Converter 0 - Pulse detection
#pragma	intvect	Vectors_Isr_DefaultHandler 141	///< Reserved
#pragma	intvect	Vectors_Isr_DefaultHandler 142	///< Reserved
#pragma	intvect	Vectors_Isr_DefaultHandler 143	///< Reserved
#pragma	intvect	Vectors_Isr_DefaultHandler 144	///< Reserved
#pragma	intvect	Vectors_Isr_DefaultHandler 145	///< Reserved
#pragma	intvect	Vectors_Isr_DefaultHandler 146	///< Reserved
#pragma	intvect	Vectors_Isr_DefaultHandler 147	///< Reserved


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

/**
 ******************************************************************************
 ** This function pre-sets the priority level of all interrupts sources. 
 ** By default all interrupts are disabled by using DEFAULT_ILM_MASK (=7).
  *****************************************************************************/
void Vectors_InitIrqLevels(void)
{
  volatile uint8_t u8Irq;
  
  for (u8Irq = MIN_ICR; u8Irq <= MAX_ICR; u8Irq++) 
  {
    ICR = (u8Irq << 8) | DEFAULT_ILM_MASK;
  }
  ICR = (66 << 8) | 3;  // ICU1 
  ICR = (17 << 8) | 3;  // EI0   
  ICR = (18 << 8) | 3;  // EI1
  ICR = (19 << 8) | 3;  // EI2
  ICR = (20 << 8) | 3;  // EI3
  ICR = (89 << 8) | 3;	// FRT0 
  ICR = (59 << 8) | 3;	// BACKLIGHT   RLT2
  ICR = (60 << 8) | 2;	// SMC   RLT2
  ICR = (14 << 8) | 2;  // MC Timer
  ICR = (98 << 8) | 3;  // ADC
  ICR = (33 << 8) | 4;  // CAN0
  ICR = (101 << 8) | 4; // UART0_RX  

}

/**
 ******************************************************************************
 ** This function is a placeholder for all vector definitions. Either use
 ** your own placeholder or add necessary code here. 
 *****************************************************************************/
__interrupt void Vectors_Isr_DefaultHandler (void)
{
    // disable interrupts
    __DI();
    
    // halt system or wait for watchdog reset
    while(1)
    {
        __wait_nop();
    }
}
