/*****************************************************************************
* File Name: CY8CMBR3xxx_HostFunctions.c
*
* Version 1.00
*
* Description:
*   This file contains the definitions of the low-level APIs. You may need to 
*   modify the content of these APIs to suit your host processor’s I2C 
*   implementation.
*
* Note:
*   These host-dependent Low Level APIs are provided as an example of
*   low level I2C read and write functions. This set of low level APIs are 
*   written for PSoC 4200/4100 devices and hence should be re-written
*   with equivalent host-dependent APIs from the respective IDEs, if the 
*   host design does not include PSoC 4200/4100 device.
* 
*   To use these APIs, the host should implement a working I2C communication
*   interface. This interface will be used by these APIs to communicate to the
*   CY8CMBR3xxx device.
*
*   For PSoC 4200/4100 devices, please ensure that you have created an instance 
*   of SCB component with I2C Master configuration. The component should be
*   named "SCB".
*
* Owner:
*   SRVS
*
* Related Document:
*   MBR3 Design Guide
*   MBR3 Device Datasheet
*
* Hardware Dependency:
*   PSoC 4200 (Update this as per the host used)
*
* Code Tested With:
*   PSoC Creator 3.0 CP7
*   CY3280-MBR3 Evaluation Kit
*   CY8CKIT-042 Pioneer Kit
*
******************************************************************************
* Copyright (2014), Cypress Semiconductor Corporation.
******************************************************************************
* This software is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and
* foreign), United States copyright laws and international treaty provisions.
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the
* Cypress Source Code and derivative works for the sole purpose of creating
* custom software in support of licensee product to be used only in conjunction
* with a Cypress integrated circuit as specified in the applicable agreement.
* Any reproduction, modification, translation, compilation, or representation of
* this software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH
* REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes without further notice to the
* materials described herein. Cypress does not assume any liability arising out
* of the application or use of any product or circuit described herein. Cypress
* does not authorize its products for use as critical components in life-support
* systems where a malfunction or failure may reasonably be expected to result in
* significant injury to the user. The inclusion of Cypress' product in a life-
* support systems application implies that the manufacturer assumes all risk of
* such use and in doing so indemnifies Cypress against all charges. Use may be
* limited by and subject to the applicable Cypress software license agreement.
*****************************************************************************/

/*******************************************************************************
* Included headers
*******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "driver/i2c.h"

#include "lp50xx.h"

/*******************************************************************************
* API Constants
*******************************************************************************/
 
/*******************************************************************************
*   Function Code
*******************************************************************************/

/**
 * @brief test code to write mpu6050
 *
 * 1. send data
 * ___________________________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write reg_address + ack | write data_len byte + ack  | stop |
 * --------|---------------------------|-------------------------|----------------------------|------|
 *
 * @param i2c_num I2C port number
 * @param reg_address slave reg address
 * @param data data to send
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
uint8_t lp50xxPlatformWriteToRegister(uint8_t deviceAddr,
                                     uint8_t address,
                                     uint8_t value)
{
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, deviceAddr << 1 | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, address, true);
  i2c_master_write_byte(cmd, value, true);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 * portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  if (ret != ESP_OK) {
	  return HAL_LED_DRIVER_LP50XX_ERR_COMMMUICATION;
  }
  return HAL_LED_DRIVER_LP50XX_ERR_NONE;
}

/**
 * @brief test code to read mpu6050
 *
 * 1. send reg address
 * ______________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write reg_address + ack | stop |
 * --------|---------------------------|-------------------------|------|
 *
 * 2. read data
 * ___________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | read data_len byte + ack(last nack)  | stop |
 * --------|---------------------------|--------------------------------------|------|
 *
 * @param i2c_num I2C port number
 * @param reg_address slave reg address
 * @param data data to read
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
uint8_t lp50xxPlatformReadFromRegister(uint8_t deviceAddr,
                                      uint8_t address,
                                      uint8_t * readBuffer)
{
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, deviceAddr << 1 | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, address, true);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 * portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  if (ret != ESP_OK) {
	  return HAL_LED_DRIVER_LP50XX_ERR_COMMMUICATION;
  }

  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, deviceAddr << 1 | I2C_MASTER_READ, true);
  i2c_master_read_byte(cmd, readBuffer, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  return HAL_LED_DRIVER_LP50XX_ERR_NONE;
}

/****************************End of File***************************************/
