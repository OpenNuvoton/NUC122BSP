/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 1 $
 * $Date: 14/12/19 3:43p $
 * @brief    Use RTC alarm interrupt event to wake-up system.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC122.h"

#define HCLK_CLOCK           60000000

/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
extern int IsDebugFifoEmpty(void);
volatile uint8_t g_u8IsRTCAlarmINT = 0;

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    uint32_t u32TimeOutCnt;

    printf("\nSystem enter to power-down mode ...\n");

    /* To check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(IsDebugFifoEmpty() == 0)
        if(--u32TimeOutCnt == 0) break;

    SCB->SCR = 4;

    /* To program PWRCON register, it needs to disable register protection first. */
    CLK->PWRCON |= (CLK_PWRCON_PD_WAIT_CPU_Msk | CLK_PWRCON_PWR_DOWN_EN_Msk);

    __WFI();
}

/**
 * @brief       IRQ Handler for RTC Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The RTC_IRQHandler is default IRQ of RTC, declared in startup_NUC122.s.
 */
void RTC_IRQHandler(void)
{
    /* To check if RTC alarm interrupt occurred */
    if(RTC_GET_ALARM_INT_FLAG() == 1)
    {
        /* Clear RTC alarm interrupt flag */
        RTC_CLEAR_ALARM_INT_FLAG();

        g_u8IsRTCAlarmINT++;
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable IRC22M clock */
    CLK->PWRCON |= CLK_PWRCON_IRC22M_EN_Msk;

    /* Waiting for IRC22M clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_IRC22M_STB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable external 12 MHz XTAL, 32 kHz XTAL */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk | CLK_PWRCON_XTL32K_EN_Msk);

    /* Waiting for clock ready */
    while(!CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk | CLK_CLKSTATUS_XTL32K_STB_Msk));

    /* Set core clock as HCLK_CLOCK */
    CLK_SetCoreClock(HCLK_CLOCK);

    /* Enable peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(RTC_MODULE);

    /* Peripheral clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_PLL, CLK_CLKDIV_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD, TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD);
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    S_RTC_TIME_DATA_T sWriteRTC, sReadRTC;
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+----------------------------------------------------+\n");
    printf("|    RTC Power-down and Alarm Wake-up Sample Code    |\n");
    printf("+----------------------------------------------------+\n\n");

    /* Open RTC and start counting */
    sWriteRTC.u32Year       = 2014;
    sWriteRTC.u32Month      = 2;
    sWriteRTC.u32Day        = 6;
    sWriteRTC.u32DayOfWeek  = RTC_THURSDAY;
    sWriteRTC.u32Hour       = 15;
    sWriteRTC.u32Minute     = 30;
    sWriteRTC.u32Second     = 55;
    sWriteRTC.u32TimeScale  = RTC_CLOCK_24;
    if(RTC_Open(&sWriteRTC) != 0)
    {
        printf("\n RTC initial fail!!");
        printf("\n Please check h/w setting!!");
        goto lexit;
    }

    /* Set RTC alarm date/time */
    sWriteRTC.u32Year       = 2014;
    sWriteRTC.u32Month      = 2;
    sWriteRTC.u32Day        = 6;
    sWriteRTC.u32DayOfWeek  = RTC_THURSDAY;
    sWriteRTC.u32Hour       = 15;
    sWriteRTC.u32Minute     = 31;
    sWriteRTC.u32Second     = 05;
    RTC_SetAlarmDateAndTime(&sWriteRTC);

    /* Enable RTC alarm interrupt and wake-up function will be enabled also */
    NVIC_EnableIRQ(RTC_IRQn);
    RTC_EnableInt(RTC_RIER_AIER_Msk);

    printf("# Set RTC current date/time: 2014/02/06 15:30:55.\n");
    printf("# Set RTC alarm date/time:   2014/02/06 15:31:05.\n");
    printf("# Wait system waken-up by RTC alarm interrupt event.\n");

    g_u8IsRTCAlarmINT = 0;

    /* System enter to Power-down */
    /* To program PWRCON register, it needs to disable register protection first. */
    SYS_UnlockReg();
    PowerDownFunction();

    u32TimeOutCnt = RTC_TIMEOUT;
    while(g_u8IsRTCAlarmINT == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for RTC alarm interrupt time-out!");
            break;
        }
    }

    /* Read current RTC date/time */
    RTC_GetDateAndTime(&sReadRTC);
    printf("System has been waken-up and current date/time is:\n");
    printf("    %d/%02d/%02d %02d:%02d:%02d\n",
           sReadRTC.u32Year, sReadRTC.u32Month, sReadRTC.u32Day, sReadRTC.u32Hour, sReadRTC.u32Minute, sReadRTC.u32Second);

lexit:

    while(1);
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
