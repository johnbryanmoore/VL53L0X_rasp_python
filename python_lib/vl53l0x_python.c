/*
MIT License

Copyright (c) 2017 John Bryan Moore

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <pthread.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <ctype.h>
#include <time.h>
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

#define VERSION_REQUIRED_MAJOR 1
#define VERSION_REQUIRED_MINOR 0
#define VERSION_REQUIRED_BUILD 2

#define VL53L0X_DEFAULT_ADDRESS 0x29

#define VL53L0X_GOOD_ACCURACY_MODE      0   // Good Accuracy mode
#define VL53L0X_BETTER_ACCURACY_MODE    1   // Better Accuracy mode
#define VL53L0X_BEST_ACCURACY_MODE      2   // Best Accuracy mode
#define VL53L0X_LONG_RANGE_MODE         3   // Longe Range mode
#define VL53L0X_HIGH_SPEED_MODE         4   // High Speed mode


static VL53L0X_Dev_t MyDevice;
static VL53L0X_Dev_t *pMyDevice = &MyDevice;
static VL53L0X_RangingMeasurementData_t    RangingMeasurementData;
static VL53L0X_RangingMeasurementData_t   *pRangingMeasurementData    = &RangingMeasurementData;

void print_pal_error(VL53L0X_Error Status)
{
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(Status, buf);
    printf("API Status: %i : %s\n", Status, buf);
}

VL53L0X_Error WaitMeasurementDataReady(VL53L0X_DEV Dev)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t NewDatReady=0;
    uint32_t LoopNb;

    // Wait until it finished
    // use timeout to avoid deadlock
    if (Status == VL53L0X_ERROR_NONE)
    {
        LoopNb = 0;
        do
        {
            Status = VL53L0X_GetMeasurementDataReady(Dev, &NewDatReady);
            if ((NewDatReady == 0x01) || Status != VL53L0X_ERROR_NONE)
            {
                break;
            }
            LoopNb = LoopNb + 1;
            VL53L0X_PollingDelay(Dev);
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP)
        {
            Status = VL53L0X_ERROR_TIME_OUT;
        }
    }

    return Status;
}

VL53L0X_Error WaitStopCompleted(VL53L0X_DEV Dev)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint32_t StopCompleted=0;
    uint32_t LoopNb;

    // Wait until it finished
    // use timeout to avoid deadlock
    if (Status == VL53L0X_ERROR_NONE)
    {
        LoopNb = 0;
        do
        {
            Status = VL53L0X_GetStopCompletedStatus(Dev, &StopCompleted);
            if ((StopCompleted == 0x00) || Status != VL53L0X_ERROR_NONE)
            {
                break;
            }
            LoopNb = LoopNb + 1;
            VL53L0X_PollingDelay(Dev);
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP)
        {
            Status = VL53L0X_ERROR_TIME_OUT;
        }
    }

    return Status;
}
    
/******************************************************************************
 * @brief   Start Ranging
 * @param   mode - ranging mode
 *              0 - Good Accuracy mode
 *              1 - Better Accuracy mode
 *              2 - Best Accuracy mode
 *              3 - Longe Range mode
 *              4 - High Speed mode
 * @note Mode Definitions
 *   Good Accuracy mode
 *       33 ms timing budget 1.2m range
 *   Better Accuracy mode
 *       66 ms timing budget 1.2m range
 *   Best Accuracy mode
 *       200 ms 1.2m range
 *   Long Range mode (indoor,darker conditions)
 *       33 ms timing budget 2m range
 *   High Speed Mode (decreased accuracy)
 *       20 ms timing budget 1.2m range
 *
 *****************************************************************************/
void startRanging(int mode)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    VL53L0X_Version_t                   Version;
    VL53L0X_Version_t                  *pVersion   = &Version;
    VL53L0X_DeviceInfo_t                DeviceInfo;
    int32_t status_int;

    printf ("VL53L0X Start Ranging\n\n");

    if (mode >= VL53L0X_GOOD_ACCURACY_MODE &&
            mode <= VL53L0X_HIGH_SPEED_MODE)
    {
        // Initialize Comms
        pMyDevice->I2cDevAddr      = VL53L0X_DEFAULT_ADDRESS;

        VL53L0X_init(pMyDevice);
        /*
         *  Get the version of the VL53L0X API running in the firmware
         */

        status_int = VL53L0X_GetVersion(pVersion);
        if (status_int == 0)
        {
            /*
             *  Verify the version of the VL53L0X API running in the firmrware
             */

            // Check the Api version. If it is not correct, put out a warning
            if( pVersion->major != VERSION_REQUIRED_MAJOR ||
                pVersion->minor != VERSION_REQUIRED_MINOR ||
                pVersion->build != VERSION_REQUIRED_BUILD )
            {
                printf("VL53L0X API Version Warning: Your firmware %d.%d.%d (revision %d). This requires %d.%d.%d.\n",
                    pVersion->major, pVersion->minor, pVersion->build, pVersion->revision,
                    VERSION_REQUIRED_MAJOR, VERSION_REQUIRED_MINOR, VERSION_REQUIRED_BUILD);
            }
            // End of implementation specific

            Status = VL53L0X_DataInit(&MyDevice); // Data initialization
            if(Status == VL53L0X_ERROR_NONE)
            {
                Status = VL53L0X_GetDeviceInfo(&MyDevice, &DeviceInfo);

                if(Status == VL53L0X_ERROR_NONE)
                {
                    printf("VL53L0X_GetDeviceInfo:\n");
                    printf("Device Name : %s\n", DeviceInfo.Name);
                    printf("Device Type : %s\n", DeviceInfo.Type);
                    printf("Device ID : %s\n", DeviceInfo.ProductId);
                    printf("ProductRevisionMajor : %d\n", DeviceInfo.ProductRevisionMajor);
                    printf("ProductRevisionMinor : %d\n", DeviceInfo.ProductRevisionMinor);

                    if ((DeviceInfo.ProductRevisionMajor != 1) && (DeviceInfo.ProductRevisionMinor != 1)) {
                        printf("Error expected cut 1.1 but found cut %d.%d\n",
                                DeviceInfo.ProductRevisionMajor, DeviceInfo.ProductRevisionMinor);
                        Status = VL53L0X_ERROR_NOT_SUPPORTED;
                    }
                }

                if(Status == VL53L0X_ERROR_NONE)
                {
                    Status = VL53L0X_StaticInit(pMyDevice); // Device Initialization
                    // StaticInit will set interrupt by default

                    if(Status == VL53L0X_ERROR_NONE)
                    {
                        Status = VL53L0X_PerformRefCalibration(pMyDevice,
                                &VhvSettings, &PhaseCal); // Device Initialization

                        if(Status == VL53L0X_ERROR_NONE)
                        {
                            Status = VL53L0X_PerformRefSpadManagement(pMyDevice,
                                    &refSpadCount, &isApertureSpads); // Device Initialization

                            if(Status == VL53L0X_ERROR_NONE)
                            {
                                // Setup in continuous ranging mode
                                Status = VL53L0X_SetDeviceMode(pMyDevice, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); 

                                if(Status == VL53L0X_ERROR_NONE)
                                {
                                    // Set accuracy mode
                                    switch (mode)
                                    {
                                        case VL53L0X_BEST_ACCURACY_MODE:
                                            printf("VL53L0X_BEST_ACCURACY_MODE\n");
                                            if (Status == VL53L0X_ERROR_NONE)
                                            {
                                                Status = VL53L0X_SetLimitCheckValue(pMyDevice,
                                                    VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                                                    (FixPoint1616_t)(0.25*65536));

                                                if (Status == VL53L0X_ERROR_NONE)
                                                {
                                                    Status = VL53L0X_SetLimitCheckValue(pMyDevice,
                                                        VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                                                        (FixPoint1616_t)(18*65536));

                                                    if (Status == VL53L0X_ERROR_NONE)
                                                    {
                                                        Status = 
                                                            VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pMyDevice, 200000);
                                                    } 
                                                }
                                            }
                                            break;

                                        case VL53L0X_LONG_RANGE_MODE:
                                            printf("VL53L0X_LONG_RANGE_MODE\n");
                                            if (Status == VL53L0X_ERROR_NONE)
                                            {
                                                Status = VL53L0X_SetLimitCheckValue(pMyDevice,
                                                            VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                                                            (FixPoint1616_t)(0.1*65536));
                                    
                                                if (Status == VL53L0X_ERROR_NONE)
                                                {
                                                    Status = VL53L0X_SetLimitCheckValue(pMyDevice,
                                                                VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                                                                (FixPoint1616_t)(60*65536));
                                        
                                                    if (Status == VL53L0X_ERROR_NONE)
                                                    {
                                                        Status = 
                                                            VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pMyDevice, 33000);
                                            
                                                        if (Status == VL53L0X_ERROR_NONE)
                                                        {
                                                            Status = VL53L0X_SetVcselPulsePeriod(pMyDevice, 
                                                                        VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
                                            
                                                            if (Status == VL53L0X_ERROR_NONE)
                                                            {
                                                                Status = VL53L0X_SetVcselPulsePeriod(pMyDevice, 
                                                                            VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                            break;

                                        case VL53L0X_HIGH_SPEED_MODE:
                                            printf("VL53L0X_HIGH_SPEED_MODE\n");
                                            if (Status == VL53L0X_ERROR_NONE)
                                            {
                                                Status = VL53L0X_SetLimitCheckValue(pMyDevice,
                                                            VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                                                            (FixPoint1616_t)(0.25*65536));

                                                if (Status == VL53L0X_ERROR_NONE)
                                                {
                                                    Status = VL53L0X_SetLimitCheckValue(pMyDevice,
                                                                VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                                                                (FixPoint1616_t)(32*65536));

                                                    if (Status == VL53L0X_ERROR_NONE)
                                                    {
                                                        Status = 
                                                            VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pMyDevice, 20000);
                                                    }
                                                }
                                            }
                                            break;

                                        case VL53L0X_BETTER_ACCURACY_MODE:
                                            printf("VL53L0X_BETTER_ACCURACY_MODE\n");
                                            if (Status == VL53L0X_ERROR_NONE)
                                            {
                                                Status = 
                                                    VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pMyDevice, 66000);
                                            }
                                            break;

                                        case VL53L0X_GOOD_ACCURACY_MODE:
                                        default:
                                            printf("VL53L0X_GOOD_ACCURACY_MODE\n");
                                            if (Status == VL53L0X_ERROR_NONE)
                                            {
                                                Status = 
                                                    VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pMyDevice, 33000);
                                            }
                                            break;
                                    }

                                    if(Status == VL53L0X_ERROR_NONE)
                                    {
                                        Status = VL53L0X_StartMeasurement(pMyDevice);
                                    }
                                    else
                                    {
                                        printf("Set Accuracy\n");
                                    }
                                }
                                else
                                {
                                    printf ("Call of VL53L0X_SetDeviceMode\n");
                                }
                            }
                            else
                            {
                                printf ("Call of VL53L0X_PerformRefSpadManagement\n");
                            }
                        }
                        else
                        {
                            printf ("Call of VL53L0X_PerformRefCalibration\n");
                        }
                    }
                    else
                    {
                        printf ("Call of VL53L0X_StaticInit\n");
                    }
                }
                else
                {
                    printf ("Invalid Device Info\n");
                }
            }
            else
            {
                printf ("Call of VL53L0X_DataInit\n");
            }
        }
        else
        {
            Status = VL53L0X_ERROR_CONTROL_INTERFACE;
            printf("Call of VL53L0X_GetVersion\n");
        }

        print_pal_error(Status);
    }
    else
    {
        printf("Invalid mode %d specified\n", mode);
    }
}

/******************************************************************************
 * @brief   Get current distance in mm
 * @return  Current distance in mm or -1 on error
 *****************************************************************************/
int32_t getDistance()
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t current_distance = -1;

    Status = WaitMeasurementDataReady(pMyDevice);

    if(Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_GetRangingMeasurementData(pMyDevice, pRangingMeasurementData);
        if(Status == VL53L0X_ERROR_NONE)
        {
            current_distance = pRangingMeasurementData->RangeMilliMeter;
        }

        // Clear the interrupt
        VL53L0X_ClearInterruptMask(pMyDevice, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
        // VL53L0X_PollingDelay(pMyDevice);
    }

    return current_distance;
}

/******************************************************************************
 * @brief   Stop Ranging
 *****************************************************************************/
void stopRanging()
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    printf ("Call of VL53L0X_StopMeasurement\n");
    Status = VL53L0X_StopMeasurement(pMyDevice);

    if(Status == VL53L0X_ERROR_NONE)
    {
        printf ("Wait Stop to be competed\n");
        Status = WaitStopCompleted(pMyDevice);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
	    Status = VL53L0X_ClearInterruptMask(pMyDevice,
		    VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
    }

    print_pal_error(Status);
}
