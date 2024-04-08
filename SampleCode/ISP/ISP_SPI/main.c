/**************************************************************************//**
 * @file     main.c
 * @brief    ISP tool main function
 * @version  2.0.0
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "targetdev.h"

#define PLLCON_SETTING  CLK_PLLCON_60MHz_HIRC
#define PLL_CLOCK       60035700

#define TEST_COUNT 16

uint32_t *_response_buff;
uint32_t spi_rcvbuf[TEST_COUNT];

void ProcessHardFault(void) {}
void SH_Return(void) {}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal RC 22.1184MHz clock */
    CLK->PWRCON |= (CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk));

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLK_S_Msk)) | CLK_CLKSEL0_HCLK_S_HIRC;
    CLK->CLKDIV = (CLK->CLKDIV & (~CLK_CLKDIV_HCLK_N_Msk)) | CLK_CLKDIV_HCLK(1);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCON = PLLCON_SETTING;

    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk));

    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLK_S_Msk)) | CLK_CLKSEL0_HCLK_S_PLL;
    CLK->CLKDIV = (CLK->CLKDIV & (~CLK_CLKDIV_HCLK_N_Msk)) | CLK_CLKDIV_HCLK(1);

    /* Update System Core Clock */
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For SYS_SysTickDelay()

    /* Enable SPI1 peripheral clock */
    CLK->APBCLK |= CLK_APBCLK_SPI1_EN_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Setup SPI1 multi-function pins */
    SYS->GPC_MFP &= ~(SYS_GPC_MFP_PC8_Msk | SYS_GPC_MFP_PC9_Msk | SYS_GPC_MFP_PC10_Msk | SYS_GPC_MFP_PC11_Msk);
    SYS->GPC_MFP |= SYS_GPC_MFP_PC8_SPI1_SS0 | SYS_GPC_MFP_PC9_SPI1_CLK | SYS_GPC_MFP_PC10_SPI1_MISO0 | SYS_GPC_MFP_PC11_SPI1_MOSI0;
    SYS->ALT_MFP &= ~(SYS_ALT_MFP_PC8_Msk);
    SYS->ALT_MFP |= SYS_ALT_MFP_PC8_SPI1_SS0;
}

void SPI_Init(void)
{
    /* Configure as a slave, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Configure SPI1 as a low level active device. */
    /* Default setting: slave selection signal is low level active. */
    SPI1->SSR = SPI_SS_ACTIVE_LOW | SPI_SSR_SS_LTRIG_Msk;
    /* Default setting: MSB first, disable unit transfer interrupt, SP_CYCLE = 0. */
    SPI1->CNTRL = SPI_SLAVE | ((32 & 0x1F) << SPI_CNTRL_TX_BIT_LEN_Pos) | (SPI_MODE_0);
    /* Set DIVIDER = 0 */
    SPI1->DIVIDER = 0UL;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    volatile uint32_t u32DataCount;
    _response_buff = (uint32_t *)response_buff; // in isp_user.c
    _response_buff[0] = 0x12345678;

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();
    SPI_Init();

    CLK->AHBCLK |= CLK_AHBCLK_ISP_EN_Msk;
    FMC->ISPCON |= FMC_ISPCON_ISPEN_Msk;

    /* Get APROM size, data flash size and address */
    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);

_ISP:
    u32DataCount = 0;

    SysTick->CTRL = 0UL;

    /* Check data count */
    while(u32DataCount < TEST_COUNT)
    {
        /* Write to TX register */
        SPI_WRITE_TX0(SPI1, _response_buff[u32DataCount]);
        /* Trigger SPI data transfer */
        SPI_TRIGGER(SPI1);
        /* Check SPI1 busy status */
        while(SPI_IS_BUSY(SPI1))
        {
            if(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
            {
                goto _ISP;
            }
        }

        /* Read RX register */
        spi_rcvbuf[u32DataCount] = SPI_READ_RX0(SPI1);
        u32DataCount++;

        SysTick->LOAD = 1000 * CyclesPerUs;
        SysTick->VAL  = (0x00);
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    }

    /* Disable SysTick counter */
    SysTick->CTRL = 0UL;

    if((u32DataCount == TEST_COUNT) && ((spi_rcvbuf[0] & 0xFFFFFF00) == 0x53504900))
    {
        spi_rcvbuf[0] &= 0x000000FF;
        ParseCmd((unsigned char *)spi_rcvbuf, 64);
    }

    goto _ISP;
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
