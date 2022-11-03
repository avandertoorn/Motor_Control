/*
 * my_board.h
 *
 *  Created on: Aug 14, 2021
 *      Author: andre
 */

#ifndef DEVICE_MY_BOARD_H_
#define DEVICE_MY_BOARD_H_

//
// Included Files
//

#include "driverlib.h"
#include "device.h"

#define GPIO_PIN_EPWM1_A 0
#define GPIO_PIN_EPWM1_B 1
#define GPIO_PIN_EPWM2_A 2
#define GPIO_PIN_EPWM2_B 3
#define GPIO_PIN_EPWM3_A 4
#define GPIO_PIN_EPWM3_B 5
#define GPIO_PIN_EPWM4_A 6
#define GPIO_PIN_EPWM4_B 7
#define GPIO_PIN_EPWM7_A 12

#define myEPWM1_BASE EPWM1_BASE
#define myEPWM2_BASE EPWM2_BASE
#define myEPWM3_BASE EPWM3_BASE
#define myEPWM4_BASE EPWM4_BASE
#define myEPWM7_BASE EPWM7_BASE
#define myEQEP1_BASE EQEP1_BASE

void    Board_init();
void    PinMux_init();


#endif /* DEVICE_MY_BOARD_H_ */
