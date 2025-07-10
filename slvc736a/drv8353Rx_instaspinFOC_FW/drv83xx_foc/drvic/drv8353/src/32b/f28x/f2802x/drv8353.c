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
//! \file   drivers/drvic/drv8305/src/32b/f28x/f2806x/drv8353.c
//! \brief  Contains the various functions related to the DRV8353 object
//!
//! (C) Copyright 2015, Texas Instruments, Inc.
//  MW_INSTALL_DIR
//  ${PROJECT_LOC}\..\..\..\..\..\..\..\..\..\..
//
// **************************************************************************
// the includes

#include <math.h>


// **************************************************************************
// drivers
#include "sw/drivers/drvic/drv8353/src/32b/f28x/f2802x/drv8353.h"


// **************************************************************************
// modules


// **************************************************************************
// platforms


// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


DRV8353_Handle DRV8353_init(void *pMemory,const size_t numBytes)
{
  DRV8353_Handle handle;

  if(numBytes < sizeof(DRV8353_Obj))
    return((DRV8353_Handle)NULL);

  // assign the handle
  handle = (DRV8353_Handle)pMemory;

  DRV8353_resetRxTimeout(handle);
  DRV8353_resetEnableTimeout(handle);

  return(handle);
} // end of DRV8353_init() function


void DRV8353_enable(DRV8353_Handle handle)
{
  DRV8353_Obj *obj = (DRV8353_Obj *)handle;
  static volatile uint16_t enableWaitTimeOut;
  uint16_t n = 0;

  // Enable the DRV8353
  GPIO_setHigh(obj->gpioHandle,obj->gpioNumber);


  enableWaitTimeOut = 0;
#ifdef DRV8353_SPI
  if(gDeviceVariant)
  {
  // Make sure the FAULT bit is not set during startup
  while(((DRV8353_readSpi(handle, Address_Status_0) & DRV8353_STATUS00_FAULT_BITS) != 0) && (enableWaitTimeOut < 1000))
  {
      if (++enableWaitTimeOut > 999)
      {
          obj->enableTimeOut = true;
      }
  }
  }
else
#endif
{
  while(enableWaitTimeOut < 1000)
   {
       if (++enableWaitTimeOut > 999)
       {
           obj->enableTimeOut = true;
       }
   }
}
  // Wait for the DRV8353 to go through start up sequence
  for(n=0;n<0xffff;n++)
    asm(" NOP");

  return;
} // end of DRV8353_enable() function

#ifdef DRV8353_SPI
DRV8353_CTRL03_PeakSourCurHS_e DRV8353_getPeakSourCurHS(DRV8353_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8353_readSpi(handle,Address_Control_3);

  // mask the bits
  data &= DRV8353_CTRL03_IDRIVEP_HS_BITS;

  return((DRV8353_CTRL03_PeakSourCurHS_e)data);
} // end of DRV8353_getPeakSourCurHS function


DRV8353_CTRL03_PeakSinkCurHS_e DRV8353_getPeakSinkCurHS(DRV8353_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8353_readSpi(handle,Address_Control_3);

  // mask the bits
  data &= DRV8353_CTRL03_IDRIVEN_HS_BITS;

  return((DRV8353_CTRL03_PeakSinkCurHS_e)data);
} // end of DRV8353_getPeakSinkCurHS function


DRV8353_CTRL04_PeakTime_e DRV8353_getPeakSourTime(DRV8353_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8353_readSpi(handle,Address_Control_4);

  // mask the bits
  data &= DRV8353_CTRL04_TDRIVE_BITS;

  return((DRV8353_CTRL04_PeakTime_e)data);
} // end of DRV8353_getPeakSourTime function


DRV8353_CTRL04_PeakSourCurLS_e DRV8353_getPeakSourCurLS(DRV8353_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8353_readSpi(handle,Address_Control_4);

  // mask the bits
  data &= DRV8353_CTRL04_IDRIVEP_LS_BITS;

  return((DRV8353_CTRL04_PeakSourCurLS_e)data);
} // end of DRV8353_getPeakSourCurLS function


DRV8353_CTRL04_PeakSinkCurLS_e DRV8353_getPeakSinkCurLS(DRV8353_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8353_readSpi(handle,Address_Control_4);

  // mask the bits
  data &= DRV8353_CTRL04_IDRIVEN_LS_BITS;

  return((DRV8353_CTRL04_PeakSinkCurLS_e)data);
} // end of DRV8353_getPeakSinkCurLS function
#endif
/*
DRV8353_CTRL04_PeakSinkTime_e DRV8353_getPeakSinkTime(DRV8353_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8353_readSpi(handle,Address_Control_4);

  // mask the bits
  data &= DRV8353_CTRL04_TDRIVE_BITS;

  return((DRV8353_CTRL04_PeakSinkTime_e)data);
} // end of DRV8353_getPeakSinkTime function
*/
#ifdef DRV8353_SPI
DRV8353_CTRL05_OcpDeg_e DRV8353_getVDSDeglitch(DRV8353_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8353_readSpi(handle,Address_Control_5);

  // mask the bits
  data &= DRV8353_CTRL05_OCP_DEG_BITS;

  return((DRV8353_CTRL05_OcpDeg_e)data);
} // end of DRV8353_getVDSDeglitch function

/*
DRV8305_CTRL07_VDSBlanking_e DRV8353_getVDSBlanking(DRV8353_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8353_readSpi(handle,Address_Control_7);

  // mask the bits
  data &= DRV8305_CTRL07_TBLANK_BITS;

  return((DRV8305_CTRL07_VDSBlanking_e)data);
} // end of DRV8305_getVDSBlanking function
*/

DRV8353_CTRL05_DeadTime_e DRV8353_getDeadTime(DRV8353_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8353_readSpi(handle,Address_Control_5);

  // mask the bits
  data &= DRV8353_CTRL05_DEAD_TIME_BITS;

  return((DRV8353_CTRL05_DeadTime_e)data);
} // end of DRV8353_getDeadTime function


DRV8353_CTRL02_PwmMode_e DRV8353_getPwmMode(DRV8353_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8353_readSpi(handle,Address_Control_2);

  // mask the bits
  data &= DRV8353_CTRL02_PWM_MODE_BITS;

  return((DRV8353_CTRL02_PwmMode_e)data);
} // end of DRV8353_getPwmMode function

/*
DRV8305_CTRL07_CommOption_e DRV8353_getCommOption(DRV8353_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_7);

  // mask the bits
  data &= DRV8305_CTRL07_COMM_OPT_BITS;

  return((DRV8305_CTRL07_CommOption_e)data);
} // end of DRV8305_getCommOption function


DRV8305_CTRL09_WatchDelay_e DRV8353_getWatchDelay(DRV8353_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_9);

  // mask the bits
  data &= DRV8305_CTRL09_WD_DLY_BITS;

  return((DRV8305_CTRL09_WatchDelay_e)data);
} // end of DRV8305_getWatchDelay function


DRV8305_CTRL0A_CSGain1_e DRV8353_getCSGain1(DRV8353_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_A);

  // mask the bits
  data &= DRV8305_CTRL0A_GAIN_CS1_BITS;

  return((DRV8305_CTRL0A_CSGain1_e)data);
} // end of DRV8305_getCSGain1 function


DRV8305_CTRL0A_CSGain2_e DRV8353_getCSGain2(DRV8353_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_A);

  // mask the bits
  data &= DRV8305_CTRL0A_GAIN_CS2_BITS;

  return((DRV8305_CTRL0A_CSGain2_e)data);
} // end of DRV8305_getCSGain2 function


DRV8305_CTRL0A_CSGain3_e DRV8353_getCSGain3(DRV8353_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_A);

  // mask the bits
  data &= DRV8305_CTRL0A_GAIN_CS3_BITS;

  return((DRV8305_CTRL0A_CSGain3_e)data);
} // end of DRV8305_getCSGain3 function


DRV8305_CTRL0A_CSBlank_e DRV8353_getCSBlank(DRV8353_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_A);

  // mask the bits
  data &= DRV8305_CTRL0A_CS_BLANK_BITS;

  return((DRV8305_CTRL0A_CSBlank_e)data);
} // end of DRV8305_getCSBlank function


DRV8305_CTRL0A_DcCalMode_e DRV8353_getDcCalMode(DRV8353_Handle handle,const DRV8305_ShuntAmpNumber_e ampNumber)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_A);

  // mask the bits
  if(ampNumber == ShuntAmpNumber_1)
    {
      data &= DRV8305_CTRL0A_DC_CAL_CH1_BITS;
    }
  else if(ampNumber == ShuntAmpNumber_2)
    {
      data &= DRV8305_CTRL0A_DC_CAL_CH2_BITS;
    }
  else if(ampNumber == ShuntAmpNumber_3)
    {
      data &= DRV8305_CTRL0A_DC_CAL_CH3_BITS;
    }

  return((DRV8305_CTRL0A_DcCalMode_e)data);
} // end of DRV8305_getDcCalMode() function


DRV8305_CTRL0B_VregUvLevel_e DRV8353_getVregUvLevel(DRV8353_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_B);

  // mask the bits
  data &= DRV8305_CTRL0B_VREG_UV_LEVEL_BITS;

  return((DRV8305_CTRL0B_VregUvLevel_e)data);
} // end of DRV8305_getVregUvLevel function


DRV8305_CTRL0B_SleepDelay_e DRV8353_getSleepDelay(DRV8353_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_B);

  // mask the bits
  data &= DRV8305_CTRL0B_SLP_DLY_BITS;

  return((DRV8305_CTRL0B_SleepDelay_e)data);
} // end of DRV8305_getSleepDelay function


DRV8305_CTRL0B_VrefScaling_e DRV8353_getVrefScaling(DRV8353_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_B);

  // mask the bits
  data &= DRV8305_CTRL0B_VREF_SCALING_BITS;

  return((DRV8305_CTRL0B_VrefScaling_e)data);
} // end of DRV8305_getVrefScaling function


DRV8305_CTRL0C_VDSMode_e DRV8353_getVDSMode(DRV8353_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_C);

  // mask the bits
  data &= DRV8305_CTRL0C_VDS_MODE_BITS;

  return((DRV8305_CTRL0C_VDSMode_e)data);
} // end of DRV8305_getVDSMode function


DRV8305_CTRL0C_VDSLevel_e DRV8353_getVDSLevel(DRV8353_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_C);

  // mask the bits
  data &= DRV8305_CTRL0C_VDS_LEVEL_BITS;

  return((DRV8305_CTRL0C_VDSLevel_e)data);
} // end of DRV8305_getVDSLevel function
*/

void DRV8353_setSpiHandle(DRV8353_Handle handle,SPI_Handle spiHandle)
{
  DRV8353_Obj *obj = (DRV8353_Obj *)handle;

  // initialize the serial peripheral interface object
  obj->spiHandle = spiHandle;

  return;
} // end of DRV8353_setSpiHandle() function
#endif

void DRV8353_setGpioHandle(DRV8353_Handle handle,GPIO_Handle gpioHandle)
{
  DRV8353_Obj *obj = (DRV8353_Obj *)handle;

  // initialize the gpio interface object
  obj->gpioHandle = gpioHandle;

  return;
} // end of DRV8353_setGpioHandle() function


void DRV8353_setGpioNumber(DRV8353_Handle handle,GPIO_Number_e gpioNumber)
{
  DRV8353_Obj *obj = (DRV8353_Obj *)handle;

  // initialize the gpio interface object
  obj->gpioNumber = gpioNumber;

  return;
} // end of DRV8353_setGpioNumber() function

/*
void DRV8305_setPeakSourCurHS(DRV8353_Handle handle,const DRV8305_CTRL05_PeakSourCurHS_e level)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_5);

  // clear the bits
  data &= (~DRV8305_CTRL05_IDRIVEP_HS_BITS);

  // set the bits
  data |= level;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_5,data);

  return;
} // end of DRV8305_setPeakSourCurHS() function


void DRV8305_setPeakSinkCurHS(DRV8353_Handle handle,const DRV8305_CTRL05_PeakSinkCurHS_e level)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_5);

  // clear the bits
  data &= (~DRV8305_CTRL05_IDRIVEN_HS_BITS);

  // set the bits
  data |= level;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_5,data);

  return;
} // end of DRV8305_setPeakSinkCurHS() function


void DRV8305_setPeakSourTime(DRV8353_Handle handle,const DRV8305_CTRL05_PeakSourTime_e time)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_5);

  // clear the bits
  data &= (~DRV8305_CTRL05_TDRIVEN_BITS);

  // set the bits
  data |= time;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_5,data);

  return;
} // end of DRV8305_setPeakSourTime() function


void DRV8305_setPeakSinkCurLS(DRV8353_Handle handle,const DRV8305_CTRL06_PeakSinkCurLS_e level)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_6);

  // clear the bits
  data &= (~DRV8305_CTRL06_IDRIVEP_LS_BITS);

  // set the bits
  data |= level;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_6,data);

  return;
} // end of DRV8305_setPeakSinkCurLS() function


void DRV8305_setPeakSinkTime(DRV8353_Handle handle,const DRV8305_CTRL06_PeakSinkTime_e time)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_6);

  // clear the bits
  data &= (~DRV8305_CTRL06_TDRIVEP_BITS);

  // set the bits
  data |= time;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_6,data);

  return;
} // end of DRV8305_setPeakSinkTime() function


void DRV8305_setVDSDeglitch(DRV8353_Handle handle,const DRV8305_CTRL07_VDSDeglitch_e time)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_7);

  // clear the bits
  data &= (~DRV8305_CTRL07_TVDS_BITS);

  // set the bits
  data |= time;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_7,data);

  return;
} // end of DRV8305_setVDSDeglitch() function


void DRV8305_setVDSBlanking(DRV8353_Handle handle,const DRV8305_CTRL07_VDSBlanking_e time)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_7);

  // clear the bits
  data &= (~DRV8305_CTRL07_TBLANK_BITS);

  // set the bits
  data |= time;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_7,data);

  return;
} // end of DRV8305_setVDSBlanking() function


void DRV8305_setDeadTime(DRV8353_Handle handle,const DRV8305_CTRL07_DeadTime_e time)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_7);

  // clear the bits
  data &= (~DRV8305_CTRL07_DEAD_TIME_BITS);

  // set the bits
  data |= time;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_7,data);

  return;
} // end of DRV8305_setDeadTime() function


void DRV8305_setPwmMode(DRV8353_Handle handle,const DRV8305_CTRL07_PwmMode_e mode)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_7);

  // clear the bits
  data &= (~DRV8305_CTRL07_PWM_MODE_BITS);

  // set the bits
  data |= mode;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_7,data);

  return;
} // end of DRV8305_setPwmMode() function


void DRV8305_setCommOption(DRV8353_Handle handle,const DRV8305_CTRL07_CommOption_e mode)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_7);

  // clear the bits
  data &= (~DRV8305_CTRL07_COMM_OPT_BITS);

  // set the bits
  data |= mode;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_7,data);

  return;
} // end of DRV8305_setCommOption() function


void DRV8305_setWatchDelay(DRV8353_Handle handle,const DRV8305_CTRL09_WatchDelay_e time)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_9);

  // clear the bits
  data &= (~DRV8305_CTRL09_WD_DLY_BITS);

  // set the bits
  data |= time;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_9,data);

  return;
} // end of DRV8305_setWatchDelay() function


void DRV8305_setCSGain1(DRV8353_Handle handle,const DRV8305_CTRL0A_CSGain1_e gain)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_A);

  // clear the bits
  data &= (~DRV8305_CTRL0A_GAIN_CS1_BITS);

  // set the bits
  data |= gain;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_A,data);

  return;
} // end of DRV8305_setCSGain1() function


void DRV8305_setCSGain2(DRV8353_Handle handle,const DRV8305_CTRL0A_CSGain2_e gain)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_A);

  // clear the bits
  data &= (~DRV8305_CTRL0A_GAIN_CS2_BITS);

  // set the bits
  data |= gain;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_A,data);

  return;
} // end of DRV8305_setCSGain2() function


void DRV8305_setCSGain3(DRV8353_Handle handle,const DRV8305_CTRL0A_CSGain3_e gain)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_A);

  // clear the bits
  data &= (~DRV8305_CTRL0A_GAIN_CS3_BITS);

  // set the bits
  data |= gain;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_A,data);

  return;
} // end of DRV8305_setCSGain3() function


void DRV8305_setCSBlank(DRV8353_Handle handle,const DRV8305_CTRL0A_CSBlank_e time)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_A);

  // clear the bits
  data &= (~DRV8305_CTRL0A_CS_BLANK_BITS);

  // set the bits
  data |= time;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_A,data);

  return;
} // end of DRV8305_setCSBlank() function


void DRV8305_setDcCalMode(DRV8353_Handle handle,const DRV8305_ShuntAmpNumber_e ampNumber,const DRV8305_CTRL0A_DcCalMode_e mode)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_A);

  // clear the bits
  if(ampNumber == ShuntAmpNumber_1)
    {
      data &= (~DRV8305_CTRL0A_DC_CAL_CH1_BITS);
    }
  else if(ampNumber == ShuntAmpNumber_2)
    {
      data &= (~DRV8305_CTRL0A_DC_CAL_CH2_BITS);
    }
  else if(ampNumber == ShuntAmpNumber_3)
    {
      data &= (~DRV8305_CTRL0A_DC_CAL_CH3_BITS);
    }

  // set the bits
  data |= mode;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_A,data);

  return;
} // end of DRV8305_setDcCalMode() function


void DRV8305_setVregUvLevel(DRV8353_Handle handle,const DRV8305_CTRL0B_VregUvLevel_e level)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_B);

  // clear the bits
  data &= (~DRV8305_CTRL0B_VREG_UV_LEVEL_BITS);

  // set the bits
  data |= level;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_B,data);

  return;
} // end of DRV8305_setVregUvLevel() function


void DRV8305_setSleepDelay(DRV8353_Handle handle,const DRV8305_CTRL0B_SleepDelay_e time)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_B);

  // clear the bits
  data &= (~DRV8305_CTRL0B_SLP_DLY_BITS);

  // set the bits
  data |= time;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_B,data);

  return;
} // end of DRV8305_setSleepDelay() function


void DRV8305_setVrefScaling(DRV8353_Handle handle,const DRV8305_CTRL0B_VrefScaling_e mode)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_B);

  // clear the bits
  data &= (~DRV8305_CTRL0B_VREF_SCALING_BITS);

  // set the bits
  data |= mode;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_B,data);

  return;
} // end of DRV8305_setVrefScaling() function


void DRV8305_setVDSMode(DRV8353_Handle handle,const DRV8305_CTRL0C_VDSMode_e mode)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_C);

  // clear the bits
  data &= (~DRV8305_CTRL0C_VDS_MODE_BITS);

  // set the bits
  data |= mode;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_C,data);

  return;
} // end of DRV8305_setVDSMode() function


void DRV8305_setVDSLevel(DRV8353_Handle handle,const DRV8305_CTRL0C_VDSLevel_e level)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_C);

  // clear the bits
  data &= (~DRV8305_CTRL0C_VDS_LEVEL_BITS);

  // set the bits
  data |= level;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_C,data);

  return;
} // end of DRV8305_setVDSLevel() function


bool DRV8305_isFault(DRV8353_Handle handle)
{
  DRV8305_Word_t readWord;
  bool status=false;


  // read the data
  readWord = DRV8305_readSpi(handle,Address_Status_1);

  if(readWord & DRV8305_STATUS01_FAULT_BITS)
    {
      status = true;
    }

  return(status);
} // end of DRV8305_isFault() function


void DRV8305_reset(DRV8353_Handle handle)
{
  uint16_t data;


  // read data
  data = DRV8305_readSpi(handle,Address_Control_9);

  // set the bits
  data |= DRV8305_CTRL09_CLR_FLTS_BITS;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_9,data);

  return;
}  // end of DRV8305_reset() function

*/
#ifdef DRV8353_SPI
void DRV8353_setupSpi(DRV8353_Handle handle, DRV_SPI_8353_Vars_t *Spi_8353_Vars)
{
  DRV8353_Address_e drvRegAddr;
  uint16_t drvDataNew;

  // Set Default Values
  // Manual Read/Write
  Spi_8353_Vars->ManReadAddr  = 0;
  Spi_8353_Vars->ManReadData  = 0;
  Spi_8353_Vars->ManReadCmd = false;
  Spi_8353_Vars->ManWriteAddr = 0;
  Spi_8353_Vars->ManWriteData = 0;
  Spi_8353_Vars->ManWriteCmd = false;

  // Read/Write
  Spi_8353_Vars->ReadCmd  = false;
  Spi_8353_Vars->WriteCmd = false;

  // Read registers for default values
  // Read Status Register 0
  drvRegAddr = Address_Status_0;
  drvDataNew = DRV8353_readSpi(handle,drvRegAddr);
  Spi_8353_Vars->Stat_Reg_00.VDS_LC         = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_VDS_LC_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_00.VDS_HC  = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_VDS_HC_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_00.VDS_LB  = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_VDS_LB_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_00.VDS_HB  = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_VDS_HB_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_00.VDS_LA   = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_VDS_LA_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_00.VDS_HA  = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_VDS_HA_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_00.OTSD   = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_OTSD_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_00.UVLO   = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_UVLO_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_00.GDF  = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_GDF_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_00.VDS_OCP = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_VDS_OCP_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_00.FAULT       = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_FAULT_BITS)?1:0;

  // Read Status Register 1
  drvRegAddr = Address_Status_1;
  drvDataNew = DRV8353_readSpi(handle,drvRegAddr);
  Spi_8353_Vars->Stat_Reg_01.VGS_LC   = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_VGS_LC_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_01.VGS_HC   = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_VGS_HC_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_01.VGS_LB   = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_VGS_LB_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_01.VGS_HB = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_VGS_HB_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_01.VGS_LA = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_VGS_LA_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_01.VGS_HA   = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_VGS_HA_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_01.GDUV   = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_GDUV_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_01.OTW   = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_OTW_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_01.SC_OC   = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_SC_OC_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_01.SB_OC   = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_SB_OC_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_01.SA_OC   = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_SA_OC_BITS)?1:0;

    // Read Control Register 2
  drvRegAddr = Address_Control_2;
  drvDataNew = DRV8353_readSpi(handle,drvRegAddr);
  Spi_8353_Vars->Ctrl_Reg_02.CLR_FLT  = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_CLR_FLT_BITS);
  Spi_8353_Vars->Ctrl_Reg_02.BRAKE    = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_BRAKE_BITS);
  Spi_8353_Vars->Ctrl_Reg_02.COAST    = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_COAST_BITS);
  Spi_8353_Vars->Ctrl_Reg_02.PWM1_DIR = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_PWM1_DIR_BITS);
  Spi_8353_Vars->Ctrl_Reg_02.PWM1_COM = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_PWM1_COM_BITS);
  Spi_8353_Vars->Ctrl_Reg_02.PWM_MODE = (DRV8353_CTRL02_PwmMode_e)(drvDataNew & (uint16_t)DRV8353_CTRL02_PWM_MODE_BITS);
  Spi_8353_Vars->Ctrl_Reg_02.OTW_REP  = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_OTW_REP_BITS);
  Spi_8353_Vars->Ctrl_Reg_02.DIS_GDF  = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_DIS_GDF_BITS);
  Spi_8353_Vars->Ctrl_Reg_02.DIS_CPUV = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_DIS_CPUV_BITS);
  Spi_8353_Vars->Ctrl_Reg_02.OCP_ACT = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_OCP_ACT)?1:0;

  // Read Control Register 3
  drvRegAddr = Address_Control_3;
  drvDataNew = DRV8353_readSpi(handle,drvRegAddr);
  Spi_8353_Vars->Ctrl_Reg_03.IDRIVEN_HS  = (DRV8353_CTRL03_PeakSinkCurHS_e)(drvDataNew & (uint16_t)DRV8353_CTRL03_IDRIVEN_HS_BITS);
  Spi_8353_Vars->Ctrl_Reg_03.IDRIVEP_HS  = (DRV8353_CTRL03_PeakSourCurHS_e)(drvDataNew & (uint16_t)DRV8353_CTRL03_IDRIVEP_HS_BITS);
  Spi_8353_Vars->Ctrl_Reg_03.LOCK        = (DRV8353_CTRL03_Lock_e)(drvDataNew & (uint16_t)DRV8353_CTRL03_LOCK_BITS);

  // Read Control Register 4
  drvRegAddr = Address_Control_4;
  drvDataNew = DRV8353_readSpi(handle,drvRegAddr);
  Spi_8353_Vars->Ctrl_Reg_04.IDRIVEN_LS  = (DRV8353_CTRL04_PeakSinkCurLS_e)(drvDataNew & (uint16_t)DRV8353_CTRL04_IDRIVEN_LS_BITS);
  Spi_8353_Vars->Ctrl_Reg_04.IDRIVEP_LS  = (DRV8353_CTRL04_PeakSourCurLS_e)(drvDataNew & (uint16_t)DRV8353_CTRL04_IDRIVEP_LS_BITS);
  Spi_8353_Vars->Ctrl_Reg_04.TDRIVE      = (DRV8353_CTRL04_PeakTime_e)(drvDataNew & (uint16_t)DRV8353_CTRL04_TDRIVE_BITS);
  Spi_8353_Vars->Ctrl_Reg_04.CBC         = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL04_CBC_BITS)?1:0;

  // Read Control Register 5
  drvRegAddr = Address_Control_5;
  drvDataNew = DRV8353_readSpi(handle,drvRegAddr);
  Spi_8353_Vars->Ctrl_Reg_05.VDS_LVL     = (DRV8353_CTRL05_VDSLVL_e)(drvDataNew & (uint16_t)DRV8353_CTRL05_VDS_LVL_BITS);
  Spi_8353_Vars->Ctrl_Reg_05.OCP_DEG     = (DRV8353_CTRL05_OcpDeg_e)(drvDataNew & (uint16_t)DRV8353_CTRL05_OCP_DEG_BITS);
  Spi_8353_Vars->Ctrl_Reg_05.OCP_MODE    = (DRV8353_CTRL05_OcpMode_e)(drvDataNew & (uint16_t)DRV8353_CTRL05_OCP_MODE_BITS);
  Spi_8353_Vars->Ctrl_Reg_05.DEAD_TIME   = (DRV8353_CTRL05_DeadTime_e)(drvDataNew & (uint16_t)DRV8353_CTRL05_DEAD_TIME_BITS);
  Spi_8353_Vars->Ctrl_Reg_05.TRETRY      = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL05_TRETRY_BITS);

  // Read Control Register 6
  drvRegAddr = Address_Control_6;
  drvDataNew = DRV8353_readSpi(handle,drvRegAddr);
  Spi_8353_Vars->Ctrl_Reg_06.SEN_LVL     = (DRV8353_CTRL06_SENLevel_e)(drvDataNew & (uint16_t)DRV8353_CTRL06_SEN_LVL_BITS);
  Spi_8353_Vars->Ctrl_Reg_06.CSA_CAL_C   = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL06_CSA_CAL_C_BITS);
  Spi_8353_Vars->Ctrl_Reg_06.CSA_CAL_B   = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL06_CSA_CAL_B_BITS);
  Spi_8353_Vars->Ctrl_Reg_06.CSA_CAL_A   = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL06_CSA_CAL_A_BITS);
  Spi_8353_Vars->Ctrl_Reg_06.DIS_SEN     = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL06_DIS_SEN_BITS);
  Spi_8353_Vars->Ctrl_Reg_06.CSA_GAIN    = (DRV8353_CTRL06_CSAGain_e)(drvDataNew & (uint16_t)DRV8353_CTRL06_CSA_GAIN_BITS);
  Spi_8353_Vars->Ctrl_Reg_06.LS_REF      = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL06_LS_REF_BITS);
  Spi_8353_Vars->Ctrl_Reg_06.VREF_DIV    = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL06_VREF_DIV_BITS);
  Spi_8353_Vars->Ctrl_Reg_06.CSA_FET     = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL06_CSA_FET_BITS);

  // Read Control Register 7
  drvRegAddr = Address_Control_7;
  drvDataNew = DRV8353_readSpi(handle,drvRegAddr);
  Spi_8353_Vars->Ctrl_Reg_07.CAL_MODE    = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_CAL_MODE_BITS);
  Spi_8353_Vars->Ctrl_Reg_07.RESERVED_1   = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED1_BITS);
  Spi_8353_Vars->Ctrl_Reg_07.RESERVED_2   = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED2_BITS);
  Spi_8353_Vars->Ctrl_Reg_07.RESERVED_3   = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED3_BITS);
  Spi_8353_Vars->Ctrl_Reg_07.RESERVED_4   = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED4_BITS);
  Spi_8353_Vars->Ctrl_Reg_07.RESERVED_5   = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED5_BITS);
  Spi_8353_Vars->Ctrl_Reg_07.RESERVED_6   = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED6_BITS);
  Spi_8353_Vars->Ctrl_Reg_07.RESERVED_7   = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED7_BITS);
  Spi_8353_Vars->Ctrl_Reg_07.RESERVED_8   = (bool)(drvDataNew & (uint16_t)DRV8335_CTRL07_RESERVED8_BITS);
  Spi_8353_Vars->Ctrl_Reg_07.RESERVED_9   = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED9_BITS);
  Spi_8353_Vars->Ctrl_Reg_07.RESERVED_10  = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED10_BITS);

  return;
} // end of DRV8353_setupSpi() function


uint16_t DRV8353_readSpi(DRV8353_Handle handle,const DRV8353_Address_e regAddr)
{
  DRV8353_Obj *obj = (DRV8353_Obj *)handle;
  uint16_t ctrlWord;
  uint16_t n;
  const uint16_t data = 0;
  volatile uint16_t readWord;
  static volatile uint16_t WaitTimeOut = 0;
  volatile SPI_FifoStatus_e RxFifoCnt = SPI_FifoStatus_Empty;

  // build the control word
  ctrlWord = (uint16_t)DRV8353_buildCtrlWord(CtrlMode_Read,regAddr,data);

  // reset the Rx fifo pointer to zero
  SPI_resetRxFifo(obj->spiHandle);
  SPI_enableRxFifo(obj->spiHandle);

  GPIO_setLow(obj->gpioHandle, GPIO_Number_7);

  // wait for registers to update
  for(n=0;n<0x06;n++)
    asm(" NOP");

  // write the command
  SPI_write(obj->spiHandle,ctrlWord);

  // wait for the response to populate the RX fifo, else a wait timeout will occur
  while((RxFifoCnt < SPI_FifoStatus_1_Word) && (WaitTimeOut < 0xffff))
  {
    RxFifoCnt = SPI_getRxFifoStatus(obj->spiHandle);
	if(++WaitTimeOut > 0xfffe)
	{
		obj->RxTimeOut = true;
	}
  }

  GPIO_setHigh(obj->gpioHandle, GPIO_Number_7);

  // Read the word
  readWord = SPI_readEmu(obj->spiHandle);

  return(readWord & DRV8353_DATA_MASK);
} // end of DRV8353_readSpi() function

//TODO:
void DRV8353_writeSpi(DRV8353_Handle handle,const DRV8353_Address_e regAddr,const uint16_t data)
{
  DRV8353_Obj *obj = (DRV8353_Obj *)handle;
  uint16_t ctrlWord;
  uint16_t n;

  // build the control word
  ctrlWord = (uint16_t)DRV8353_buildCtrlWord(CtrlMode_Write,regAddr,data);

  // reset the Rx fifo pointer to zero
  SPI_resetRxFifo(obj->spiHandle);
  SPI_enableRxFifo(obj->spiHandle);

  GPIO_setLow(obj->gpioHandle, GPIO_Number_7);

  // wait for GPIO
  for(n=0;n<0x06;n++)
    asm(" NOP");

  // write the command
  SPI_write(obj->spiHandle,ctrlWord);

  // wait for registers to update
  for(n=0;n<0xF0;n++)
    asm(" NOP");

  GPIO_setHigh(obj->gpioHandle, GPIO_Number_7);

  return;
}  // end of DRV8353_writeSpi() function

// TODO:
void DRV8353_writeData(DRV8353_Handle handle, DRV_SPI_8353_Vars_t *Spi_8353_Vars)
{
  DRV8353_Address_e drvRegAddr = 0;
  uint16_t drvDataNew = 0;

  Spi_8353_Vars->Ctrl_Reg_04.IDRIVEN_LS = 3;
  Spi_8353_Vars->Ctrl_Reg_04.IDRIVEP_LS = 3 << 4;
  Spi_8353_Vars->Ctrl_Reg_03.IDRIVEN_HS = 3;
  Spi_8353_Vars->Ctrl_Reg_03.IDRIVEP_HS = 3 << 4;

  if(Spi_8353_Vars->WriteCmd)
  {
      // Write Control Register 2
      drvRegAddr = Address_Control_2;
      drvDataNew = (Spi_8353_Vars->Ctrl_Reg_02.CLR_FLT << 0)  | \
                   (Spi_8353_Vars->Ctrl_Reg_02.BRAKE << 1)    | \
                   (Spi_8353_Vars->Ctrl_Reg_02.COAST <<2)     | \
                   (Spi_8353_Vars->Ctrl_Reg_02.PWM1_DIR << 3) | \
                   (Spi_8353_Vars->Ctrl_Reg_02.PWM1_COM << 4) | \
                   (Spi_8353_Vars->Ctrl_Reg_02.PWM_MODE)      | \
                   (Spi_8353_Vars->Ctrl_Reg_02.OTW_REP << 7)  | \
                   (Spi_8353_Vars->Ctrl_Reg_02.DIS_GDF << 8)  | \
                   (Spi_8353_Vars->Ctrl_Reg_02.DIS_CPUV <<9)  | \
                   (Spi_8353_Vars->Ctrl_Reg_02.OCP_ACT << 10);
      DRV8353_writeSpi(handle,drvRegAddr,drvDataNew);

      // Write Control Register 3
      drvRegAddr = Address_Control_3;
      drvDataNew = (Spi_8353_Vars->Ctrl_Reg_03.IDRIVEN_HS) | \
                   (Spi_8353_Vars->Ctrl_Reg_03.IDRIVEP_HS) | \
                   (Spi_8353_Vars->Ctrl_Reg_03.LOCK);
      DRV8353_writeSpi(handle,drvRegAddr,drvDataNew);

      // Write Control Register 4
      drvRegAddr = Address_Control_4;
      drvDataNew = (Spi_8353_Vars->Ctrl_Reg_04.IDRIVEN_LS) | \
                   (Spi_8353_Vars->Ctrl_Reg_04.IDRIVEP_LS) | \
                   (Spi_8353_Vars->Ctrl_Reg_04.TDRIVE) | \
                   (Spi_8353_Vars->Ctrl_Reg_04.CBC << 10);
      DRV8353_writeSpi(handle,drvRegAddr,drvDataNew);

      // Write Control Register 5
      drvRegAddr = Address_Control_5;
      drvDataNew = (Spi_8353_Vars->Ctrl_Reg_05.VDS_LVL)      | \
                   (Spi_8353_Vars->Ctrl_Reg_05.OCP_DEG)      | \
                   (Spi_8353_Vars->Ctrl_Reg_05.OCP_MODE)     | \
                   (Spi_8353_Vars->Ctrl_Reg_05.DEAD_TIME)    | \
                   (Spi_8353_Vars->Ctrl_Reg_05.TRETRY << 10);
      DRV8353_writeSpi(handle,drvRegAddr,drvDataNew);

      // Write Control Register 6
      drvRegAddr = Address_Control_6;
      drvDataNew = (Spi_8353_Vars->Ctrl_Reg_06.SEN_LVL)        | \
                   (Spi_8353_Vars->Ctrl_Reg_06.CSA_CAL_C << 2) | \
                   (Spi_8353_Vars->Ctrl_Reg_06.CSA_CAL_B << 3) | \
                   (Spi_8353_Vars->Ctrl_Reg_06.CSA_CAL_A << 4) | \
                   (Spi_8353_Vars->Ctrl_Reg_06.DIS_SEN << 5)   | \
                   (Spi_8353_Vars->Ctrl_Reg_06.CSA_GAIN)       | \
                   (Spi_8353_Vars->Ctrl_Reg_06.LS_REF << 8)    | \
                   (Spi_8353_Vars->Ctrl_Reg_06.VREF_DIV << 9)  | \
                   (Spi_8353_Vars->Ctrl_Reg_06.CSA_FET << 10);
      DRV8353_writeSpi(handle,drvRegAddr,drvDataNew);

      // Write Control Register 7
      drvRegAddr = Address_Control_7;
      drvDataNew = (Spi_8353_Vars->Ctrl_Reg_07.CAL_MODE) << 0   | \
                   (Spi_8353_Vars->Ctrl_Reg_07.RESERVED_1 << 1) | \
                   (Spi_8353_Vars->Ctrl_Reg_07.RESERVED_2 << 2) | \
                   (Spi_8353_Vars->Ctrl_Reg_07.RESERVED_3 << 3) | \
                   (Spi_8353_Vars->Ctrl_Reg_07.RESERVED_4 << 4) | \
                   (Spi_8353_Vars->Ctrl_Reg_07.RESERVED_5 << 5) | \
                   (Spi_8353_Vars->Ctrl_Reg_07.RESERVED_6 << 6) | \
                   (Spi_8353_Vars->Ctrl_Reg_07.RESERVED_7 << 7) | \
                   (Spi_8353_Vars->Ctrl_Reg_07.RESERVED_8 << 8) | \
                   (Spi_8353_Vars->Ctrl_Reg_07.RESERVED_9 << 9) | \
                   (Spi_8353_Vars->Ctrl_Reg_07.RESERVED_10 << 10) ;
    DRV8353_writeSpi(handle,drvRegAddr,drvDataNew);

    Spi_8353_Vars->WriteCmd = false;
  }

  // Manual write to the DRV8353
  if(Spi_8353_Vars->ManWriteCmd)
  {
	// Custom Write
	drvRegAddr = (DRV8353_Address_e)(Spi_8353_Vars->ManWriteAddr << 11);
	drvDataNew = Spi_8353_Vars->ManWriteData;
    DRV8353_writeSpi(handle,drvRegAddr,drvDataNew);
    drvDataNew = 0;
	Spi_8353_Vars->ManWriteCmd = false;
  }

  return;
}  // end of DRV8353_writeData() function


void DRV8353_readData(DRV8353_Handle handle, DRV_SPI_8353_Vars_t *Spi_8353_Vars)
{
  DRV8353_Address_e drvRegAddr;
  uint16_t drvDataNew;

  if(Spi_8353_Vars->ReadCmd)
  {
	  // Read registers for default values
	  // Read Status Register 0
	  drvRegAddr = Address_Status_0;
	  drvDataNew = DRV8353_readSpi(handle,drvRegAddr);
	  Spi_8353_Vars->Stat_Reg_00.VDS_LC         = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_VDS_LC_BITS)?1:0;
	  Spi_8353_Vars->Stat_Reg_00.VDS_HC  = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_VDS_HC_BITS)?1:0;
	  Spi_8353_Vars->Stat_Reg_00.VDS_LB  = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_VDS_LB_BITS)?1:0;
	  Spi_8353_Vars->Stat_Reg_00.VDS_HB  = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_VDS_HB_BITS)?1:0;
	  Spi_8353_Vars->Stat_Reg_00.VDS_LA   = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_VDS_LA_BITS)?1:0;
	  Spi_8353_Vars->Stat_Reg_00.VDS_HA  = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_VDS_HA_BITS)?1:0;
	  Spi_8353_Vars->Stat_Reg_00.OTSD   = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_OTSD_BITS)?1:0;
	  Spi_8353_Vars->Stat_Reg_00.UVLO   = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_UVLO_BITS)?1:0;
	  Spi_8353_Vars->Stat_Reg_00.GDF  = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_GDF_BITS)?1:0;
	  Spi_8353_Vars->Stat_Reg_00.VDS_OCP = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_VDS_OCP_BITS)?1:0;
	  Spi_8353_Vars->Stat_Reg_00.FAULT       = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_FAULT_BITS)?1:0;

	  // Read Status Register 1
	  drvRegAddr = Address_Status_1;
	  drvDataNew = DRV8353_readSpi(handle,drvRegAddr);
	  Spi_8353_Vars->Stat_Reg_01.VGS_LC   = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_VGS_LC_BITS)?1:0;
	  Spi_8353_Vars->Stat_Reg_01.VGS_HC   = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_VGS_HC_BITS)?1:0;
	  Spi_8353_Vars->Stat_Reg_01.VGS_LB   = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_VGS_LB_BITS)?1:0;
	  Spi_8353_Vars->Stat_Reg_01.VGS_HB = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_VGS_HB_BITS)?1:0;
	  Spi_8353_Vars->Stat_Reg_01.VGS_LA = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_VGS_LA_BITS)?1:0;
	  Spi_8353_Vars->Stat_Reg_01.VGS_HA   = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_VGS_HA_BITS)?1:0;
	  Spi_8353_Vars->Stat_Reg_01.GDUV   = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_GDUV_BITS)?1:0;
	  Spi_8353_Vars->Stat_Reg_01.OTW   = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_OTW_BITS)?1:0;
	  Spi_8353_Vars->Stat_Reg_01.SC_OC   = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_SC_OC_BITS)?1:0;
	  Spi_8353_Vars->Stat_Reg_01.SB_OC   = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_SB_OC_BITS)?1:0;
	  Spi_8353_Vars->Stat_Reg_01.SA_OC   = (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_SA_OC_BITS)?1:0;

		// Read Control Register 2
	  drvRegAddr = Address_Control_2;
	  drvDataNew = DRV8353_readSpi(handle,drvRegAddr);
	  Spi_8353_Vars->Ctrl_Reg_02.CLR_FLT  = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_CLR_FLT_BITS);
	  Spi_8353_Vars->Ctrl_Reg_02.BRAKE    = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_BRAKE_BITS);
	  Spi_8353_Vars->Ctrl_Reg_02.COAST    = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_COAST_BITS);
	  Spi_8353_Vars->Ctrl_Reg_02.PWM1_DIR = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_PWM1_DIR_BITS);
	  Spi_8353_Vars->Ctrl_Reg_02.PWM1_COM = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_PWM1_COM_BITS);
	  Spi_8353_Vars->Ctrl_Reg_02.PWM_MODE = (DRV8353_CTRL02_PwmMode_e)(drvDataNew & (uint16_t)DRV8353_CTRL02_PWM_MODE_BITS);
	  Spi_8353_Vars->Ctrl_Reg_02.OTW_REP  = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_OTW_REP_BITS);
	  Spi_8353_Vars->Ctrl_Reg_02.DIS_GDF  = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_DIS_GDF_BITS);
	  Spi_8353_Vars->Ctrl_Reg_02.DIS_CPUV = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_DIS_CPUV_BITS);
	  Spi_8353_Vars->Ctrl_Reg_02.OCP_ACT = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_OCP_ACT)?1:0;

	  // Read Control Register 3
	  drvRegAddr = Address_Control_3;
	  drvDataNew = DRV8353_readSpi(handle,drvRegAddr);
	  Spi_8353_Vars->Ctrl_Reg_03.IDRIVEN_HS  = (DRV8353_CTRL03_PeakSinkCurHS_e)(drvDataNew & (uint16_t)DRV8353_CTRL03_IDRIVEN_HS_BITS);
	  Spi_8353_Vars->Ctrl_Reg_03.IDRIVEP_HS  = (DRV8353_CTRL03_PeakSourCurHS_e)(drvDataNew & (uint16_t)DRV8353_CTRL03_IDRIVEP_HS_BITS);
	  Spi_8353_Vars->Ctrl_Reg_03.LOCK        = (DRV8353_CTRL03_Lock_e)(drvDataNew & (uint16_t)DRV8353_CTRL03_LOCK_BITS);

	  // Read Control Register 4
	  drvRegAddr = Address_Control_4;
	  drvDataNew = DRV8353_readSpi(handle,drvRegAddr);
	  Spi_8353_Vars->Ctrl_Reg_04.IDRIVEN_LS  = (DRV8353_CTRL04_PeakSinkCurLS_e)(drvDataNew & (uint16_t)DRV8353_CTRL04_IDRIVEN_LS_BITS);
	  Spi_8353_Vars->Ctrl_Reg_04.IDRIVEP_LS  = (DRV8353_CTRL04_PeakSourCurLS_e)(drvDataNew & (uint16_t)DRV8353_CTRL04_IDRIVEP_LS_BITS);
	  Spi_8353_Vars->Ctrl_Reg_04.TDRIVE      = (DRV8353_CTRL04_PeakTime_e)(drvDataNew & (uint16_t)DRV8353_CTRL04_TDRIVE_BITS);
	  Spi_8353_Vars->Ctrl_Reg_04.CBC         = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL04_CBC_BITS)?1:0;

	  // Read Control Register 5
	  drvRegAddr = Address_Control_5;
	  drvDataNew = DRV8353_readSpi(handle,drvRegAddr);
	  Spi_8353_Vars->Ctrl_Reg_05.VDS_LVL     = (DRV8353_CTRL05_VDSLVL_e)(drvDataNew & (uint16_t)DRV8353_CTRL05_VDS_LVL_BITS);
	  Spi_8353_Vars->Ctrl_Reg_05.OCP_DEG     = (DRV8353_CTRL05_OcpDeg_e)(drvDataNew & (uint16_t)DRV8353_CTRL05_OCP_DEG_BITS);
	  Spi_8353_Vars->Ctrl_Reg_05.OCP_MODE    = (DRV8353_CTRL05_OcpMode_e)(drvDataNew & (uint16_t)DRV8353_CTRL05_OCP_MODE_BITS);
	  Spi_8353_Vars->Ctrl_Reg_05.DEAD_TIME   = (DRV8353_CTRL05_DeadTime_e)(drvDataNew & (uint16_t)DRV8353_CTRL05_DEAD_TIME_BITS);
	  Spi_8353_Vars->Ctrl_Reg_05.TRETRY      = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL05_TRETRY_BITS);

	  // Read Control Register 6
	  drvRegAddr = Address_Control_6;
	  drvDataNew = DRV8353_readSpi(handle,drvRegAddr);
	  Spi_8353_Vars->Ctrl_Reg_06.SEN_LVL     = (DRV8353_CTRL06_SENLevel_e)(drvDataNew & (uint16_t)DRV8353_CTRL06_SEN_LVL_BITS);
	  Spi_8353_Vars->Ctrl_Reg_06.CSA_CAL_C   = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL06_CSA_CAL_C_BITS);
	  Spi_8353_Vars->Ctrl_Reg_06.CSA_CAL_B   = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL06_CSA_CAL_B_BITS);
	  Spi_8353_Vars->Ctrl_Reg_06.CSA_CAL_A   = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL06_CSA_CAL_A_BITS);
	  Spi_8353_Vars->Ctrl_Reg_06.DIS_SEN     = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL06_DIS_SEN_BITS);
	  Spi_8353_Vars->Ctrl_Reg_06.CSA_GAIN    = (DRV8353_CTRL06_CSAGain_e)(drvDataNew & (uint16_t)DRV8353_CTRL06_CSA_GAIN_BITS);
	  Spi_8353_Vars->Ctrl_Reg_06.LS_REF      = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL06_LS_REF_BITS);
	  Spi_8353_Vars->Ctrl_Reg_06.VREF_DIV    = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL06_VREF_DIV_BITS);
	  Spi_8353_Vars->Ctrl_Reg_06.CSA_FET     = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL06_CSA_FET_BITS);

	 // Read Control Register 7
	   drvRegAddr = Address_Control_7;
	   drvDataNew = DRV8353_readSpi(handle,drvRegAddr);
	   Spi_8353_Vars->Ctrl_Reg_07.CAL_MODE    = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_CAL_MODE_BITS);
	   Spi_8353_Vars->Ctrl_Reg_07.RESERVED_1   = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED1_BITS);
	   Spi_8353_Vars->Ctrl_Reg_07.RESERVED_2   = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED2_BITS);
	   Spi_8353_Vars->Ctrl_Reg_07.RESERVED_3   = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED3_BITS);
	   Spi_8353_Vars->Ctrl_Reg_07.RESERVED_4   = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED4_BITS);
	   Spi_8353_Vars->Ctrl_Reg_07.RESERVED_5   = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED5_BITS);
	   Spi_8353_Vars->Ctrl_Reg_07.RESERVED_6   = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED6_BITS);
	   Spi_8353_Vars->Ctrl_Reg_07.RESERVED_7   = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED7_BITS);
	   Spi_8353_Vars->Ctrl_Reg_07.RESERVED_8   = (bool)(drvDataNew & (uint16_t)DRV8335_CTRL07_RESERVED8_BITS);
	   Spi_8353_Vars->Ctrl_Reg_07.RESERVED_9   = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED9_BITS);
	   Spi_8353_Vars->Ctrl_Reg_07.RESERVED_10  = (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED10_BITS);

    Spi_8353_Vars->ReadCmd = false;
  }

  // Manual read from the DRV8353
  if(Spi_8353_Vars->ManReadCmd)
  {
	// Custom Read
	drvRegAddr = (DRV8353_Address_e)(Spi_8353_Vars->ManReadAddr << 11);
    drvDataNew = DRV8353_readSpi(handle,drvRegAddr);
	Spi_8353_Vars->ManReadData = drvDataNew;
	Spi_8353_Vars->ManReadCmd = false;
  }

    return;
}  // end of DRV8353_readData() function

#endif
// end of file
