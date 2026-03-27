/******************************************************************************
 * File Name:   main.c
 *
 * Description: This is the source code for the Non-Blocking Flash Write using 
 * Polling PSOC 4 Application for ModusToolbox.
 *
 * Related Document: See README.md
 *
 *
 ******************************************************************************
 * (c) 2023-2026, Infineon Technologies AG, or an affiliate of Infineon
 * Technologies AG. All rights reserved.
 * This software, associated documentation and materials ("Software") is
 * owned by Infineon Technologies AG or one of its affiliates ("Infineon")
 * and is protected by and subject to worldwide patent protection, worldwide
 * copyright laws, and international treaty provisions. Therefore, you may use
 * this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software. If no license
 * agreement applies, then any use, reproduction, modification, translation, or
 * compilation of this Software is prohibited without the express written
 * permission of Infineon.
 *
 * Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
 * THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
 * SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
 * Infineon reserves the right to make changes to the Software without notice.
 * You are responsible for properly designing, programming, and testing the
 * functionality and safety of your intended application of the Software, as
 * well as complying with any legal requirements related to its use. Infineon
 * does not guarantee that the Software will be free from intrusion, data theft
 * or loss, or other breaches ("Security Breaches"), and Infineon shall have
 * no liability arising out of any Security Breaches. Unless otherwise
 * explicitly approved by Infineon, the Software may not be used in any
 * application where a failure of the Product or any consequences of the use
 * thereof can reasonably be expected to result in personal injury.
 *****************************************************************************/

/******************************************************************************
 * Include Header files
 *****************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"

/******************************************************************************
 * Macros and Constants
 *****************************************************************************/
/* Set this macro to 'true' to observe the failure case of flash write. A RAM
 * address is passed as row address argument to the flash write function instead
 * of a flash address causing the function to return error.
 */
#define MAKE_FLASH_WRITE_FAIL (false)

/* Check MCU datasheet to get flash size. 
 * Flash Row starts with 0, for e.x Row0, Row1...RowN-1.
 * Last Row = RowN - 1
 */
#define LAST_FLASH_ROW        (CY_FLASH_NUMBER_ROWS-1)

/* This array reserves space in the flash for one row of size
 * CY_FLASH_SIZEOF_ROW. Explicit initialization is required so that memory is
 * allocated in flash instead of RAM.
 */
#define CALCULATE_FLASH_ADDRESS(rowNum) \
        (CY_FLASH_BASE + ((rowNum) * CY_FLASH_SIZEOF_ROW))

/* Flash write delay in miliseconds */
#define FLASH_WRITE_DELAY_MS (10u)

/* CY_ALIGN ensures that the array address is an integer multiple of the row
 *  size so that the array occupies one complete row.
 */
CY_ALIGN(CY_FLASH_SIZEOF_ROW)

#if(MAKE_FLASH_WRITE_FAIL == false)
/* Make the address point to last user flash row */
const uint8_t *flash_data = (uint8_t *)CALCULATE_FLASH_ADDRESS(LAST_FLASH_ROW);
#else
/* Make the address point to some RAM location */
const uint8_t *flash_data = (uint8_t *)CY_SRAM_BASE;
#endif  /* #if(MAKE_FLASH_WRITE_FAIL == false) */

/******************************************************************************
 * Global Variables
 *****************************************************************************/

/******************************************************************************
 * Function Name: main
 ******************************************************************************
 * Summary:
 * System entrance point. This function performs.
 * 1. Initializes the BSP.
 * 2. Initializes the data into RAM that will be later written into flash.
 * 3. Writes the data into flash.
 * 4. Verifies the data written into flash by comparing it with the RAM data.
 * 5. Turns the LED1/LED6 ON if the flash write is successful otherwise turns
 *    the LED2/LED7 ON.
 * Parameters:
 *  void
 *
 * Return:
 *  int
 *
 *****************************************************************************/
int main(void)
{
    cy_rslt_t result;
    cy_en_flashdrv_status_t flash_write_status;
    uint32_t index;
    uint8_t ram_data[CY_FLASH_SIZEOF_ROW];
    bool error_flag= false;
    uint32_t i;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts. */
    __enable_irq();

    /* Initialize the data in RAM that will be written into flash */
    for(index = 0; index < CY_FLASH_SIZEOF_ROW; index++)
    {
        ram_data[index] = (uint8_t)index;
    }

    /* Non-blocking flash write */
    flash_write_status = Cy_Flash_StartWrite((uint32_t)flash_data, (const uint32_t *)ram_data);

    if(flash_write_status == CY_FLASH_DRV_OPERATION_STARTED)
    {
        /* The non-blocking write row API Cy_Flash_StartWrite() requires that
         * Cy_Flash_ResumeWrite() function be called 3 times to complete the write.
         * It is advised not to prolong calling this API for more than 25 ms.
         */
        for(i=0; i<3; i++)
        {
            Cy_SysLib_Delay(FLASH_WRITE_DELAY_MS);
            (void) Cy_Flash_ResumeWrite();
            Cy_SysLib_Delay(FLASH_WRITE_DELAY_MS);
        }

        /* Wait for the successful flash write */
        if(Cy_Flash_IsOperationComplete() != CY_FLASH_DRV_SUCCESS)
        {
            error_flag = true;
        }
        else
        {
            /* Verify the data written into flash by comparing it with the RAM data */
            if(memcmp(ram_data, flash_data, CY_FLASH_SIZEOF_ROW) != 0u)
            {
                error_flag = true;
            }
        }
    }
    else /* flash write operation did not start */
    {
        /* Flag error if the Cy_Flash_StartWrite API status is not as expected */
        error_flag = true;
    }

    if(error_flag)
    {
        /* Turn the Error; LED1/LED6 ON */
        Cy_GPIO_Clr(LED_ERROR_PORT, LED_ERROR_NUM);
    }
    else
    {
        /* Turn the Ok; LED2/LED7 ON */
        Cy_GPIO_Clr(LED_OK_PORT, LED_OK_NUM);
    }

    for(;;)
    {
        /* Put the CPU into Sleep mode to save power */
        Cy_SysPm_CpuEnterSleep();
    }
}

/* [] END OF FILE */
