/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//****************************************************************************
//
// LP5024.h - Hardware abstraction layer for LP5024 LED Driver
//
//****************************************************************************
#ifndef LP50XX_H_
#define LP50XX_H_

#define HAL_LED_DRIVER_LP50XX_PERCENTAGE(percent)	((percent*255)/100)

#ifndef LED_DRIVER_LP50XX_SLAVE_ADDRESS_INDEPENDENT
#define LED_DRIVER_LP50XX_SLAVE_ADDRESS_INDEPENDENT	(0x28)
#endif

#ifndef LED_DRIVER_LP50XX_SLAVE_ADDRESS_BROADCAST
#define LED_DRIVER_LP50XX_SLAVE_ADDRESS_BROADCAST	(0x3c)
#endif

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
	HAL_LED_DRIVER_LP50XX_ERR_NONE,
	HAL_LED_DRIVER_LP50XX_ERR_COMMMUICATION
}HAL_LED_DRIVER_LP50XX_ERR;


//****************************************************************************
//
// Function Platform Implements
//
//****************************************************************************

uint8_t lp50xxPlatformWriteToRegister(uint8_t deviceAddr,
                                     uint8_t address,
                                     uint8_t value);

uint8_t lp50xxPlatformReadFromRegister(uint8_t deviceAddr,
                                      uint8_t address,
                                      uint8_t * readBuffer);


//****************************************************************************
//
// Function
//
//****************************************************************************


bool halLedDriverLp50xxBankInit(uint8_t bankAddr);
#define halLedDriverLp50xxInit()	halLedDriverLp50xxBankInit(LED_DRIVER_LP50XX_SLAVE_ADDRESS_INDEPENDENT)

bool halLedDriverLp50xxBankModeSelectStandby(uint8_t bankAddr); //Set Chip_EN=0 to enter STANDBY mode

bool halLedDriverLp50xxBankModeSelectNormal(uint8_t bankAddr); //Set Chip_EN=1 to enter NORMAL mode

#define LP50XX_DEVICE_CONFIG1_Global_Off 			(1<<0)
#define LP50XX_DEVICE_CONFIG1_Max_Current_Option	(1<<1)
#define LP50XX_DEVICE_CONFIG1_PWM_Dithering_EN		(1<<2)
#define LP50XX_DEVICE_CONFIG1_Auto_Incr_EN			(1<<3)
#define LP50XX_DEVICE_CONFIG1_Power_Save_EN			(1<<4)
#define LP50XX_DEVICE_CONFIG1_Log_Scale_EN			(1<<5)
bool halLedDriverLp50xxBankDeviceConfig1(uint8_t bankAddr, uint8_t config1);

/*Bank Select*/
bool halLedDriverLp50xxBankLedConfig0(uint8_t bankAddr, uint8_t config0);

bool halLedDriverLp50xxBankBrightnessSet(uint8_t bankAddr, uint8_t brightness);
#define halLedDriverLp50xxBrightnessSet(brightness)	halLedDriverLp50xxBankBrightnessSet(LED_DRIVER_LP50XX_SLAVE_ADDRESS_INDEPENDENT, brightness)

bool halLedDriverLp50xxBankColorSet(uint8_t bankAddr, uint8_t BANK_A_COLOR, uint8_t BANK_B_COLOR, uint8_t BANK_C_COLOR);

bool halLedDriverLp50xxBankOutputSet(uint8_t bankAddr, uint8_t outputNumber, uint8_t value);
#define halLedDriverLp50xxOutputSet(outputNumber, value)	halLedDriverLp50xxBankOutputSet(LED_DRIVER_LP50XX_SLAVE_ADDRESS_INDEPENDENT, outputNumber, value)

bool halLedDriverLp50xxBankOutputGet(uint8_t bankAddr, uint8_t outputNumber, uint8_t *value);
#define halLedDriverLp50xxOutputSet(outputNumber, value)	halLedDriverLp50xxBankOutputSet(LED_DRIVER_LP50XX_SLAVE_ADDRESS_INDEPENDENT, outputNumber, value)

bool halLedDriverLp50xxBankLedColorSet(uint8_t bankAddr, uint8_t ledNumber, uint8_t GS_Red, uint8_t GS_Green, uint8_t GS_Blue);

bool halLedDriverLp50xxBankLedBrightnessSet(uint8_t bankAddr, uint8_t ledNumber, uint8_t brightness);
#define halLedDriverLp50xxLedBrightnessSet(ledNumber, brightness)	halLedDriverLp50xxBankLedBrightnessSet(LED_DRIVER_LP50XX_SLAVE_ADDRESS_INDEPENDENT, ledNumber, brightness)


//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
#endif /* ADC14_H_ */
