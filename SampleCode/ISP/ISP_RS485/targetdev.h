/***************************************************************************//**
 * @file     targetdev.h
 * @brief    ISP support function header file
 * @version  0x31
 *
 * @note
 * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "NUC122.h"
#include "uart_transfer.h"
#include "ISP_USER.h"

/* rename for uart_transfer.c */
#define UART_N					    UART1
#define UART_N_IRQHandler		    UART1_IRQHandler
#define UART_N_IRQn					UART1_IRQn

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
