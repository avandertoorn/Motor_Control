#include "driverlib.h"
#include "device.h"
#include "my_board.h"
#include "math.h"

#define EPWM_TIMER_TBPRD            4000U
#define EPWM_MAX_PERCENT            1.0F   //Max on-time
#define EPWM_MIN_PERCENT            .000F     //Min on-time
#define EPWM_MIN_DUTY_PERCENT       .01F     //Min Duty cycle
#define STEP_RESOLUTION             4096U
#define MAGNETIC_REV_TO_MECH_REVS   7U
#define MIN_ADC                     20U
#define MAX_ADC                     4000U
#define ADC_RANGE                   MAX_ADC - MIN_ADC
#define EPWM8_CMP_RANGE             MAX_CMP - MIN_CMP

typedef struct
{
    uint32_t epwmModule;
    uint32_t epwmACmpReg;
    uint32_t epwmBCmpReg;
    int32_t sinValue; //for graphing
}epwmSineInfo;

epwmSineInfo epwm1Info;
epwmSineInfo epwm2Info;
epwmSineInfo epwm4Info;

int positionArray[STEP_RESOLUTION][3];

unsigned int EPWM_MAX_CMP;
float DUTY_SCALE_FACTOR;
float DUTY_OFFSET;
unsigned int adcResult;
unsigned int stepCount;
unsigned int eqepStepCount;
unsigned int arrayShift;
float dutyPercentage;
uint16_t spiResult;
bool initialized;

void initSineEPWM(epwmSineInfo *info);
void initEPWM7(void);
void initADC(void);
void initADCSOC(void);
void initEPWM6(void);
void initEQEP(void);
__interrupt void epwm1ISR(void);
__interrupt void adcA1ISR(void);
void updateCompare(epwmSineInfo *info1,epwmSineInfo *info2, epwmSineInfo *info3);
void updateCompareModule(epwmSineInfo *info, int compareValue);
int calcCompareValue(epwmSineInfo *info, uint16_t qepPosition);
void initSPIAMaster(void);
//
// Main
//
void main(void)
{
    Device_init();
    Device_initGPIO();
    Interrupt_initModule();
    Interrupt_initVectorTable();

    Interrupt_register(INT_EPWM1, &epwm1ISR);
    Interrupt_register(INT_ADCA1, &adcA1ISR);

    Board_init();
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    stepCount = 0U;
    eqepStepCount = 0U;

    epwm1Info.epwmModule = myEPWM1_BASE;
    epwm1Info.epwmACmpReg = myEPWM1_BASE + 0x6AU + EPWM_COUNTER_COMPARE_A + 0x1U;
    epwm1Info.epwmBCmpReg = myEPWM1_BASE + 0x6AU + EPWM_COUNTER_COMPARE_B + 0x1U;
    epwm2Info.epwmModule = myEPWM2_BASE;
    epwm2Info.epwmACmpReg = myEPWM2_BASE + 0x6AU + EPWM_COUNTER_COMPARE_A + 0x1U;
    epwm2Info.epwmBCmpReg = myEPWM2_BASE + 0x6AU + EPWM_COUNTER_COMPARE_B + 0x1U;
    epwm4Info.epwmModule = myEPWM4_BASE;
    epwm4Info.epwmACmpReg = myEPWM4_BASE + 0x6AU + EPWM_COUNTER_COMPARE_A + 0x1U;
    epwm4Info.epwmBCmpReg = myEPWM4_BASE + 0x6AU + EPWM_COUNTER_COMPARE_B + 0x1U;

    initSineEPWM(&epwm1Info); //Phase 1
    initSineEPWM(&epwm2Info); //Phase 2
    initSineEPWM(&epwm4Info); //Phase 3
    initEPWM7(); //For syncing oscilliscope
    initEQEP();
    initADC(); //ADC for motor rpm
    initEPWM6(); //for ADC
    initADCSOC();
    initSPIAMaster();

    EPWM_setCounterCompareValue(myEPWM1_BASE,
                                    EPWM_COUNTER_COMPARE_C,
                                    EPWM_TIMER_TBPRD / 2);
    EPWM_setInterruptSource(myEPWM1_BASE, EPWM_INT_TBCTR_U_CMPC);
    EPWM_enableInterrupt(myEPWM1_BASE);
    EPWM_setInterruptEventCount(myEPWM1_BASE, 1U);

    EPWM_setSyncOutPulseMode(myEPWM1_BASE, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);
    EPWM_setSyncOutPulseMode(myEPWM4_BASE, EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN);
    EPWM_setSyncOutPulseMode(myEPWM2_BASE, EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN);
    EPWM_setSyncOutPulseMode(myEPWM3_BASE, EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN);
    EPWM_setSyncOutPulseMode(myEPWM7_BASE, EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN);
    EPWM_enablePhaseShiftLoad(myEPWM4_BASE);
    EPWM_enablePhaseShiftLoad(myEPWM2_BASE);
    EPWM_enablePhaseShiftLoad(myEPWM3_BASE);
    EPWM_enablePhaseShiftLoad(EPWM6_BASE);
    EPWM_enablePhaseShiftLoad(myEPWM7_BASE);

    EPWM_MAX_CMP = (unsigned int)((float)EPWM_TIMER_TBPRD * EPWM_MAX_PERCENT);
    DUTY_SCALE_FACTOR = EPWM_MAX_PERCENT - EPWM_MIN_DUTY_PERCENT;
    DUTY_OFFSET = EPWM_MIN_DUTY_PERCENT - EPWM_MIN_PERCENT;
    arrayShift = 0;

    int i = 0;
    int phaseLow = 3;
    int upDown = 0;
    float lastPercentage = 0.0f;
    float topStart = (float)M_PI * 2.0f * (1.0f / 12.0f);
    float bottom = (float)M_PI * 2.0f * (3.0f / 12.0f);
    float topEnd = (float)M_PI * 2.0f * (5.0f / 12.0f);
    for(i = 0; i < STEP_RESOLUTION; i++)
    {
        float angle1 = ((float)i / (float)STEP_RESOLUTION * (float)M_PI * 2.0f) * (float)MAGNETIC_REV_TO_MECH_REVS;
        //float angle1Harmonic = angle1 * 3.0f;
        float angle2 = angle1 + (float)M_PI * 2.0f * (1.0f / 3.0f);
        //float angle2Harmonic = angle2 * 3.0f;
        float angle3 = angle1 + (float)M_PI * 2.0f * (2.0f / 3.0f);
        //float angle3Harmonic = angle3 * 3.0f;
        float dutyPercentage1 = sinf(angle1) * 1.154f;
        float dutyPercentage2 = sinf(angle2) * 1.154f;
        float dutyPercentage3 = sinf(angle3) * 1.154f;

        float adjustedAngle = angle1;
        float percentage = 0.0f;

        if(adjustedAngle < topStart)
        {
            adjustedAngle = adjustedAngle + (topEnd - topStart);
        }
        while(adjustedAngle > topEnd)
        {
            adjustedAngle = adjustedAngle - (topEnd - topStart);
        }

        if(adjustedAngle < bottom)
        {
            percentage = ((bottom - topStart) - (adjustedAngle - topStart)) / (bottom - topStart);
        }
        else
        {
            percentage = (adjustedAngle - bottom) / (topEnd - bottom);
        }

        float scaledPercentage = percentage * .576f;
        int offset = EPWM_MAX_CMP * scaledPercentage - EPWM_MAX_CMP * .288f;

        int phaseA = (int)(dutyPercentage1 * (float)EPWM_MAX_CMP) + offset;
        int phaseB = (int)(dutyPercentage2 * (float)EPWM_MAX_CMP) + offset;
        int phaseC = (int)(dutyPercentage3 * (float)EPWM_MAX_CMP) + offset;

        if(percentage < lastPercentage)
        {
            upDown = 1;
        }
        else
        {
            if(upDown == 1)
            {
                if(phaseLow == 1)
                {
                    phaseLow = 3;
                }
                else if(phaseLow == 2)
                {
                   phaseLow = 1;
                }
                else
                {
                    phaseLow = 2;
                }
            }
            upDown = 0;
        }
        lastPercentage = percentage;

        if(phaseLow == 1)
        {
            phaseA = -EPWM_MAX_CMP - 1;
        }
        else if(phaseLow == 2)
        {
            phaseB = -EPWM_MAX_CMP - 1;
        }
        else
        {
            phaseC = -EPWM_MAX_CMP - 1;
        }

        if(percentage < .5f)
        {
            if(upDown == 1)
            {
                int value = (int)(((1.0f - (percentage * 2.0f)) * (float)EPWM_MAX_CMP) * -1.0f);
                if(phaseLow == 1)
                {
                    phaseC = value;
                }
                else if(phaseLow == 2)
                {
                    phaseA = value;
                }
                else
                {
                    phaseB = value;
                }
            }
            else
            {
                int value = (int)(((1.0f - (percentage * 2.0f)) * (float)EPWM_MAX_CMP) * -1.0f);
                if(phaseLow == 1)
                {
                    phaseB = value;
                }
                else if(phaseLow == 2)
                {
                    phaseC = value;
                }
                else
                {
                    phaseA = value;
                }
            }
        }
        positionArray[i][0] = phaseA;
        positionArray[i][1] = phaseB;
        positionArray[i][2] = phaseC;
    }


    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    Interrupt_enable(INT_EPWM1);
    Interrupt_enable(INT_ADCA1);

    EPWM_enableADCTrigger(EPWM6_BASE, EPWM_SOC_A);
    EPWM_setTimeBaseCounterMode(EPWM6_BASE, EPWM_COUNTER_MODE_UP);

    EINT;
    ERTM;


    SPI_writeDataBlockingNonFIFO(SPIA_BASE, 0x0018);
    while(SPI_isBusy(SPIA_BASE));
    {

    }

    DEVICE_DELAY_US(50);

    SPI_writeDataBlockingNonFIFO(SPIA_BASE, 0x0021);

    spiResult = SPI_readDataNonBlocking(SPIA_BASE);
    while(SPI_isBusy(SPIA_BASE));
    {

    }

    DEVICE_DELAY_US(50);

    SPI_writeDataBlockingNonFIFO(SPIA_BASE, 0x0003);
    while(SPI_isBusy(SPIA_BASE));
    {

    }

    DEVICE_DELAY_US(50);

    SPI_writeDataBlockingNonFIFO(SPIA_BASE, 0x8004);

    spiResult = SPI_readDataNonBlocking(SPIA_BASE);
    while(SPI_isBusy(SPIA_BASE));
    {

    }

    DEVICE_DELAY_US(50);

    SPI_writeDataBlockingNonFIFO(SPIA_BASE, 0x0000);
    spiResult = SPI_readDataNonBlocking(SPIA_BASE);

    for(;;)
    {
        NOP;
    }
}

//
// End of File
//

void initSineEPWM(epwmSineInfo *info)
{
    unsigned int countShift = 0U;
    EPWM_setTimeBasePeriod(info->epwmModule, EPWM_TIMER_TBPRD);
    EPWM_setPhaseShift(info->epwmModule, countShift);
    EPWM_setTimeBaseCounter(info->epwmModule, 0U);

    EPWM_setCounterCompareValue(info->epwmModule,
                                EPWM_COUNTER_COMPARE_A,
                                0);
    EPWM_setCounterCompareValue(info->epwmModule,
                                EPWM_COUNTER_COMPARE_B,
                                0);

    EPWM_setTimeBaseCounterMode(info->epwmModule, EPWM_COUNTER_MODE_UP);

    EPWM_setClockPrescaler(info->epwmModule,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);

    EPWM_setCounterCompareShadowLoadMode(info->epwmModule,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_PERIOD);
    EPWM_setCounterCompareShadowLoadMode(info->epwmModule,
                                         EPWM_COUNTER_COMPARE_B,
                                         EPWM_COMP_LOAD_ON_CNTR_PERIOD);

    EPWM_setActionQualifierAction(info->epwmModule,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(info->epwmModule,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(info->epwmModule,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(info->epwmModule,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
}

void initEPWM6(void)
{
    EPWM_disableADCTrigger(EPWM6_BASE, EPWM_SOC_A);
    EPWM_setADCTriggerSource(EPWM6_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_U_CMPA);
    EPWM_setADCTriggerEventPrescale(EPWM6_BASE, EPWM_SOC_A, 1);
    EPWM_setCounterCompareValue(EPWM6_BASE, EPWM_COUNTER_COMPARE_A, 20);
    EPWM_setTimeBaseCounterMode(EPWM6_BASE, EPWM_COUNTER_MODE_UP);
    EPWM_setTimeBasePeriod(EPWM6_BASE, EPWM_TIMER_TBPRD);
    EPWM_setClockPrescaler(EPWM6_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_4);
    EPWM_setTimeBaseCounterMode(EPWM6_BASE, EPWM_COUNTER_MODE_STOP_FREEZE);
}

void initADC(void)
{
    ADC_setVREF(ADCA_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_2_0);
    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);
    ADC_enableConverter(ADCA_BASE);
    DEVICE_DELAY_US(1000);
}

void initADCSOC(void)
{
    ADC_setupSOC(ADCA_BASE, ADC_CH_ADCIN8, ADC_TRIGGER_EPWM6_SOCA,
                 ADC_CH_ADCIN8, 8);
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_CH_ADCIN8);
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
}



void initEPWM7(void)
{
    EPWM_setTimeBasePeriod(myEPWM7_BASE, EPWM_TIMER_TBPRD);
    EPWM_setPhaseShift(myEPWM7_BASE, 0U);
    EPWM_setTimeBaseCounter(myEPWM7_BASE, 0U);

    EPWM_setCounterCompareValue(myEPWM7_BASE,
                                EPWM_COUNTER_COMPARE_A,
                                (unsigned int)((float)EPWM_TIMER_TBPRD / 2.0f - 1.0f));
    EPWM_setCounterCompareValue(myEPWM7_BASE,
                                EPWM_COUNTER_COMPARE_B,
                                0U);

    EPWM_setTimeBaseCounterMode(myEPWM7_BASE, EPWM_COUNTER_MODE_UP);

    EPWM_setClockPrescaler(myEPWM7_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);

    EPWM_setCounterCompareShadowLoadMode(myEPWM7_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareShadowLoadMode(myEPWM7_BASE,
                                         EPWM_COUNTER_COMPARE_B,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    EPWM_setActionQualifierAction(myEPWM7_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(myEPWM7_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(myEPWM7_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(myEPWM7_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
}

void initEQEP(void)
{
    EQEP_setDecoderConfig(EQEP1_BASE, (EQEP_CONFIG_1X_RESOLUTION |
                                       EQEP_CONFIG_QUADRATURE |
                                       EQEP_CONFIG_NO_SWAP));
    EQEP_setEmulationMode(EQEP1_BASE, EQEP_EMULATIONMODE_RUNFREE);
    EQEP_setPositionCounterConfig(EQEP1_BASE, EQEP_POSITION_RESET_IDX,
                                  0x00000FFF);
    EQEP_enableUnitTimer(EQEP1_BASE, (DEVICE_SYSCLK_FREQ / 100));
    EQEP_setLatchMode(EQEP1_BASE, EQEP_LATCH_UNIT_TIME_OUT);
    EQEP_enableModule(EQEP1_BASE);
    EQEP_setCaptureConfig(EQEP1_BASE, EQEP_CAPTURE_CLK_DIV_64,
                          EQEP_UNIT_POS_EVNT_DIV_32);
    EQEP_enableCapture(EQEP1_BASE);
}

__interrupt void epwm1ISR(void)
{
    updateCompare(&epwm1Info, &epwm2Info, &epwm4Info);
    EPWM_clearEventTriggerInterruptFlag(myEPWM1_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}

__interrupt void adcA1ISR(void)
{
    unsigned int result = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER8) >> 3;
    adcResult = result;

     if(result < MIN_ADC)
     {
         if(dutyPercentage > 0.0f)
         {
             if(result < 10)
             {
                 result = 0;
             }
             else
             {
                 result = MIN_ADC;
             }
         }
         else
         {
             result = 0;
         }
     }
    if(result != 0)
    {
        dutyPercentage = (float)result / (float)(511) * DUTY_SCALE_FACTOR + DUTY_OFFSET;
    }
    else
    {
        dutyPercentage = 0.0f;
    }

    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    if(true == ADC_getInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    }
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

void updateCompare(epwmSineInfo *info1,epwmSineInfo *info2, epwmSineInfo *info3)
{
    uint16_t qepPosition = EQEP_getPosition(EQEP1_BASE);

    qepPosition = qepPosition + arrayShift;
    qepPosition = qepPosition & 4095;
    eqepStepCount = qepPosition;

    updateCompareModule(info1, positionArray[qepPosition][0]);
    updateCompareModule(info2, positionArray[qepPosition][1]);
    updateCompareModule(info3, positionArray[qepPosition][2]);
}

void updateCompareModule(epwmSineInfo *info, int compareValue)
{
    if(compareValue > 0)
    {
        compareValue = compareValue * dutyPercentage;
        HWREGH(info->epwmACmpReg) = compareValue;
        HWREGH(info->epwmBCmpReg) = 0;
        //EPWM_setCounterCompareValue(info->epwmModule,
        //                            EPWM_COUNTER_COMPARE_A,
        //                            (unsigned int)compareValue);
        //EPWM_setCounterCompareValue(info->epwmModule,
        //                            EPWM_COUNTER_COMPARE_B,
        //                            0);
    }
    else if(compareValue < 0)
    {
        HWREGH(info->epwmACmpReg) = 0;
        HWREGH(info->epwmBCmpReg) = -compareValue;
        //EPWM_setCounterCompareValue(info->epwmModule,
        //                            EPWM_COUNTER_COMPARE_A,
        //                            0);
        //EPWM_setCounterCompareValue(info->epwmModule,
       //                             EPWM_COUNTER_COMPARE_B,
       //                             EPWM_TIMER_TBPRD + 1);
    }
    else
    {
        HWREGH(info->epwmACmpReg) = 0;
        HWREGH(info->epwmBCmpReg) = 0;
        //EPWM_setCounterCompareValue(info->epwmModule,
        //                            EPWM_COUNTER_COMPARE_A,
        //                            0);
        //EPWM_setCounterCompareValue(info->epwmModule,
        //                            EPWM_COUNTER_COMPARE_B,
        //                             0);
    }
}

void initSPIAMaster(void)
{
    //
    // Must put SPI into reset before configuring it
    //
    SPI_disableModule(SPIA_BASE);

    //
    // SPI configuration. Use a 500kHz SPICLK and 16-bit word size.
    //
    SPI_setConfig(SPIA_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA0,
                  SPI_MODE_MASTER, 500000, 16);
    SPI_disableLoopback(SPIA_BASE);
    SPI_setEmulationMode(SPIA_BASE, SPI_EMULATION_FREE_RUN);

    //
    // Configuration complete. Enable the module.
    //
    SPI_enableModule(SPIA_BASE);
}
