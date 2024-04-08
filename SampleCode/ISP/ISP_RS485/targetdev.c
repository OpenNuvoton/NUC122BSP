/***************************************************************************//**
 * @file     targetdev.c
 * @brief    ISP support function source file
 * @version  0x31
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "targetdev.h"
#include "ISP_USER.h"

// Supports 32K/64K (APROM)
uint32_t GetApromSize()
{
    uint32_t size = 0x8000, data;
    int result;

    do {
        result = FMC_Read_User(size, &data);

        if (result < 0) {
            return size;
        } else {
            size *= 2;
        }
    } while (1);
}

void GetDataFlashInfo(uint32_t *addr, uint32_t *size)
{
    *addr = 0x1F000;
    *size = 4096;
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
