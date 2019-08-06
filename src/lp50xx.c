/* --COPYRIGHT--,BSD
 * Copyright (c) 2019, vinhlq
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
// lp50xx.c - Hardware abstraction layer for LP5024 LED Driver
//
//****************************************************************************
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "lp50xx_platform.h"
#include "lp50xx.h"

//-----------------------------------------------------------------------------
//static local funcitons


bool halLedDriverLp50xxBankModeSelectStandby(uint8_t bankAddr)//Set Chip_EN=0 to enter STANDBY mode
{
	if(!bankAddr)
	{
		bankAddr = LED_DRIVER_LP50XX_SLAVE_ADDRESS_BROADCAST;
	}

	if(HAL_LED_DRIVER_LP50XX_ERR_NONE != lp50xxPlatformWriteToRegister(	bankAddr,
							0x0,
							0x0))
	{
		return false;
	}
	return true;
}

bool halLedDriverLp50xxBankModeSelectNormal(uint8_t bankAddr)//Set Chip_EN=1 to enter NORMAL mode
{
	if(!bankAddr)
	{
		bankAddr = LED_DRIVER_LP50XX_SLAVE_ADDRESS_BROADCAST;
	}

	if(HAL_LED_DRIVER_LP50XX_ERR_NONE != lp50xxPlatformWriteToRegister(	bankAddr,
																0x0,
																0x40))
	{
		return false;
	}
	return true;
}

/*Device Configure*/
bool halLedDriverLp50xxBankDeviceConfig1(uint8_t bankAddr, uint8_t config1)
{
	if(!bankAddr)
	{
		bankAddr = LED_DRIVER_LP50XX_SLAVE_ADDRESS_BROADCAST;
	}

    if(HAL_LED_DRIVER_LP50XX_ERR_NONE != lp50xxPlatformWriteToRegister(	bankAddr,
																0x01,
																config1))
    {
    	return false;
    }
    return true;
}

/*Bank Select*/
bool halLedDriverLp50xxBankLedConfig0(uint8_t bankAddr, uint8_t config0)
{
	if(!bankAddr)
	{
		bankAddr = LED_DRIVER_LP50XX_SLAVE_ADDRESS_BROADCAST;
	}

	if(HAL_LED_DRIVER_LP50XX_ERR_NONE != lp50xxPlatformWriteToRegister(	bankAddr,
							0x02,
							config0))
	{
		return false;
	}
	return true;
}

bool halLedDriverLp50xxBankBrightnessSet(uint8_t bankAddr, uint8_t brightness)
{
	if(!bankAddr)
	{
		bankAddr = LED_DRIVER_LP50XX_SLAVE_ADDRESS_BROADCAST;
	}

	if(HAL_LED_DRIVER_LP50XX_ERR_NONE != lp50xxPlatformWriteToRegister(	bankAddr,
																0x03,
																brightness))
	{
		return false;
	}
	return true;
}

bool halLedDriverLp50xxBankColorSet(uint8_t bankAddr, uint8_t BANK_A_COLOR, uint8_t BANK_B_COLOR, uint8_t BANK_C_COLOR)
{
	if(!bankAddr)
	{
		bankAddr = LED_DRIVER_LP50XX_SLAVE_ADDRESS_BROADCAST;
	}

    /*Set Bank Red Color Gray*/
	if(HAL_LED_DRIVER_LP50XX_ERR_NONE != lp50xxPlatformWriteToRegister(	bankAddr,
																0x04,
																BANK_A_COLOR))
	{
		return false;
	}

    /*Set Bank Green Color Gray*/
	if(HAL_LED_DRIVER_LP50XX_ERR_NONE != lp50xxPlatformWriteToRegister(	bankAddr,
																0x05,
																BANK_B_COLOR))
	{
		return false;
	}

    /*Set Bank Blue Color Gray*/
	if(HAL_LED_DRIVER_LP50XX_ERR_NONE != lp50xxPlatformWriteToRegister(	bankAddr,
																0x06,
																BANK_C_COLOR))
	{
		return false;
	}
	return true;
}

bool halLedDriverLp50xxBankOutputSet(uint8_t bankAddr, uint8_t outputNumber, uint8_t value)
{
	if(!bankAddr)
	{
		bankAddr = LED_DRIVER_LP50XX_SLAVE_ADDRESS_BROADCAST;
	}

	if(HAL_LED_DRIVER_LP50XX_ERR_NONE != lp50xxPlatformWriteToRegister(	bankAddr,
																outputNumber+0x0F,
																value))
	{
		return false;
	}
	return true;
}

bool halLedDriverLp50xxBankLedBrightnessSet(uint8_t bankAddr, uint8_t ledNumber, uint8_t brightness)
{
	if(!bankAddr)
	{
		bankAddr = LED_DRIVER_LP50XX_SLAVE_ADDRESS_BROADCAST;
	}

	if(HAL_LED_DRIVER_LP50XX_ERR_NONE != lp50xxPlatformWriteToRegister(	bankAddr,
																ledNumber+0x07,
																brightness))
	{
		return false;
	}
    return true;
}

bool halLedDriverLp50xxBankLedColorSet(uint8_t bankAddr, uint8_t ledNumber, uint8_t GS_Red, uint8_t GS_Green, uint8_t GS_Blue)
{
	if(!bankAddr)
	{
		bankAddr = LED_DRIVER_LP50XX_SLAVE_ADDRESS_BROADCAST;
	}

	if(HAL_LED_DRIVER_LP50XX_ERR_NONE != lp50xxPlatformWriteToRegister(	bankAddr,
																ledNumber*3+0x0F,
																GS_Red))
	{
		return false;
	}

	if(HAL_LED_DRIVER_LP50XX_ERR_NONE != lp50xxPlatformWriteToRegister(	bankAddr,
																ledNumber*3+0x10,
																GS_Green))
	{
		return false;
	}

	if(HAL_LED_DRIVER_LP50XX_ERR_NONE != lp50xxPlatformWriteToRegister(	bankAddr,
																ledNumber*3+0x11,
																GS_Blue))
	{
		return false;
	}
    return true;
}

bool halLedDriverLp50xxBankReset(uint8_t bankAddr)
{
	if(!bankAddr)
	{
		bankAddr = LED_DRIVER_LP50XX_SLAVE_ADDRESS_BROADCAST;
	}

	if(HAL_LED_DRIVER_LP50XX_ERR_NONE != lp50xxPlatformWriteToRegister(	bankAddr,
																0x27,
																0xff))
	{
		return false;
	}
	return true;
}

bool halLedDriverLp50xxBankOutputGet(uint8_t bankAddr, uint8_t outputNumber, uint8_t *value)
{
	if(!bankAddr)
	{
		bankAddr = LED_DRIVER_LP50XX_SLAVE_ADDRESS_BROADCAST;
	}

	if(HAL_LED_DRIVER_LP50XX_ERR_NONE != lp50xxPlatformReadFromRegister(bankAddr,
																outputNumber+0x0F,
																value))
	{
		return false;
	}
	return true;
}

bool halLedDriverLp50xxBankInit(uint8_t bankAddr)
{
	uint8_t config1 = 	LP50XX_DEVICE_CONFIG1_PWM_Dithering_EN |	\
						LP50XX_DEVICE_CONFIG1_Auto_Incr_EN	| \
						LP50XX_DEVICE_CONFIG1_Power_Save_EN	|	\
						LP50XX_DEVICE_CONFIG1_Log_Scale_EN;

	if(!halLedDriverLp50xxBankReset(bankAddr))
	{
		return false;
	}

	if(!halLedDriverLp50xxBankModeSelectNormal(bankAddr))
	{
		return false;
	}

	if(!halLedDriverLp50xxBankDeviceConfig1(bankAddr, config1))
	{
		return false;
	}

	if(!halLedDriverLp50xxBankLedConfig0(bankAddr, 0))
	{
		return false;
	}

//	if(!halLedDriverLp50xxBankLedBrightnessSet(bankAddr, 0x80))
//	{
//		return false;
//	}

//	if(!halLedDriverLp50xxBankBankBrightnessSet_U1(0xff))
//	{
//		return false;
//	}
//
//	if(!halLedDriverLp50xxBankBankBrightnessSet_U2(0xff))
//	{
//		return false;
//	}

//	for(uint8_t i = 0; i < 24; i++)
//	{
//		if(!halLedDriverLp50xxBankOutputSet(bankAddr, i, 0x20))
//		{
//			return false;
//		}
//	}
	return true;
}
