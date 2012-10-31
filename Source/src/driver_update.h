#ifndef __DRIVER_UPDATE_H
#define __DRIVER_UPDATE_H

typedef enum en_gui_mode
{
	Empty,
    mode_Motochasi,
    mode_Vom,
    mode_Fuel,
    mode_Vbort,
    mode_FuelRate,
    mode_ProbegMotochNaOst,
    mode_CoolantLevel,
    mode_ErrorSpeedL,
    mode_ErrorSpeedR,
    mode_ErrorFuel,  
    mode_ErrorCAN,
    mode_ParamsZ,
    mode_ParamsI,
    mode_ParamsR,
    mode_ParamsK,
    mode_ParamsKV2,
    mode_ParamsZV,
    mode_ParamsV,
    mode_ParamsT,
    AdminMode3,
    AdminMode4,
    AdminMode5,
    AdminMode6
} en_gui_mode_t;

void DriverInit(void);
void ButoonPress(unsigned int Num_button, unsigned char enState);
void SetModePass(void);
void UpdateSMCLCD(void);

extern float MetrInImp;
extern float ProbegM;


#endif