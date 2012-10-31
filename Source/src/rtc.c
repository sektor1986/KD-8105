#include "mcu.h"
#include "rtc.h"

#define DividerMC 	1000000

void InitRTC(void)
{
    WTBR0 = (0xFFFF & DividerMC);  // Set Sub-Second Prescaler 
    WTBR1 = (DividerMC >> 16);

    WTCR_INTE0 = 0;		// No Interrupts
    WTCR_INTE1 = 0;
    WTCR_INTE2 = 0;
    WTCR_INTE3 = 0;
    WTCER_INTE4 = 0;
    WTCR_OE = 0;		// No Output
    WTCKSR = 0;			// Main Clock Source   

    WTSR = 0;			// Seconds: 0
    WTMR = 0;			// Minutes: 0
    WTHR = 0;			// Hours:   0

    WTCR_ST = 1;		// ... and go!
}