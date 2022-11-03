/*
 * my_board.c
 *
 *  Created on: Aug 14, 2021
 *      Author: andre
 */

#include "my_board.h"

void Board_init()
{
    EALLOW;

    PinMux_init();

    EDIS;
}

void PinMux_init()
{
    //EPWM1 -> myEPWM1 Pinmux
    GPIO_setPinConfig(GPIO_0_EPWM1_A);
    GPIO_setPinConfig(GPIO_1_EPWM1_B);
    //EPWM2 -> myEPWM2 Pinmux
    GPIO_setPinConfig(GPIO_2_EPWM2_A);
    GPIO_setPinConfig(GPIO_3_EPWM2_B);
    //EPWM3 -> myEPWM3 Pinmux
    GPIO_setPinConfig(GPIO_4_EPWM3_A);
    GPIO_setPinConfig(GPIO_5_EPWM3_B);

    GPIO_setPinConfig(GPIO_6_EPWM4_A);
    GPIO_setPinConfig(GPIO_7_EPWM4_B);

    //EPWM7 -> myEPWM7 Pinmux
    GPIO_setPinConfig(GPIO_12_EPWM7_A);

    GPIO_setPinConfig(GPIO_35_EQEP1_A);
    GPIO_setPinConfig(GPIO_37_EQEP1_B);
    GPIO_setPinConfig(GPIO_59_EQEP1_INDEX);

    GPIO_setPinConfig(GPIO_8_SPIA_SIMO);
    GPIO_setPadConfig(8, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(8, GPIO_QUAL_ASYNC);

    //
    // GPIO10 is the SPISOMIA.
    //
    GPIO_setPinConfig(GPIO_10_SPIA_SOMI);
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(10, GPIO_QUAL_ASYNC);

    //
    // GPIO09 is the SPICLKA.
    //
    GPIO_setPinConfig(GPIO_9_SPIA_CLK);
    GPIO_setPadConfig(9, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(9, GPIO_QUAL_ASYNC);

    //
    // GPIO11 is the SPISTEA.
    //
    GPIO_setPinConfig(GPIO_11_SPIA_STE);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(11, GPIO_QUAL_ASYNC);
}


