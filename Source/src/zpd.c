#include "fstdint.h"


/** port pin definition for a test signal */
typedef struct stc_port_pin
{
    boolean_t           bEnable;        ///< flag which indicates whether this signal is enabled or not
    volatile uint8_t*   pu8Port;        ///< pointer to port data register
    uint8_t             u8BitMask;      ///< mask for pin of port
} stc_port_pin_t;


/** test signals for debugging purposes */
typedef struct stc_test_signals
{
    stc_port_pin_t  stcStepBlindingTime;                            ///< test signal to mark step blinding time (common for all SMC units)
    stc_port_pin_t  stcStepSamplingTime;                            ///< test signal to mark step sampling time (common for all SMC units)
    stc_port_pin_t  astcAdcSampling[SMC_ZPD_MAX_STEPPER_UNITS];     ///< test signal to mark BEMF sampling (for each SMC unit)
    stc_port_pin_t  astcStallDetected[SMC_ZPD_MAX_STEPPER_UNITS];   ///< test signal to mark stall detected event (for each SMC unit)
} stc_test_signals_t;

/** ADC states */
typedef enum en_adc_states
{
    AdcStateStart,
    AdcStateConnectionCheck,
    AdcStateCalibration,
    AdcStateZpd
} en_adc_states_t;

/** ADC sampling control data */
typedef struct stc_adc
{
    en_adc_states_t eState;                 ///< state of ADC sampling
    uint8_t*        pu8SampleBuffer;        ///< pointer to ADC sample buffer (provided by the application)
    uint16_t        u16Index;               ///< index pointing to the next entry in pu8SampleBuffer
    uint16_t        u16StopIndex;           ///< stop index where to stop ADC sampling
    uint16_t        u16BufferSize;          ///< size of pu8SampleBuffer
    uint16_t        u16SampleCount;         ///< number of ADC samples to record
    uint8_t         u8CurSmcIndex;          ///< current SMC number that is sampled
    uint8_t         u8AdcFallbackChannel;   ///< ADC channel which is used in case of a time slot sampling for a disabled SMC unit
} stc_adc_t;

/** SMC ZPD operation modes */
typedef enum en_smc_mode
{
    NoOperation,    ///< Idle, no operation in progress (default after SmcZpd_Init)
    NormalDriving,  ///< normal driving i.e. micro-stepping is running
    Zpd             ///< ZPD operation is running
} en_smc_mode_t;

/** common ZPD control data */
typedef struct stc_zpd_control
{
    uint32_t    u32BlindingTimeUs;                  ///< blinding time in micro-seconds (time which must elapse after applying a new full step to delay the start of BEMF sampling)
    uint32_t    u32SamplingTimeUs;                  ///< sampling time in micro-seconds which starts after blinding time is elapsed (time where BEMF sampling is executed. blinding + sampling time = step width)
    uint8_t     ucConnectionCheckSteps;             ///< down-counter for connection check steps
    uint8_t     ucCalibrationSteps;                 ///< down-counter for calibration check steps
    boolean_t   bStartBlindingTime;                 ///< flag which indicates whether the RLT0 should be configured for blinding time or sampling time
    boolean_t   bStartADC;                          ///< flag which indicates whether the ADC sampling should be started or not
    func_ptr_t  pfnStallDetectedCallbackFunction;   ///< stall detected callback function pointer to application's routine
} stc_zpd_control_t;
