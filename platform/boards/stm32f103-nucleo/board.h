
#pragma once
/*
===============================================================================
Header:        platform/boards/stm32f103-nucleo/board.h
Description:   Example board mapping for STM32F103 Nucleo (USART1 PA9/PA10)
Author:        Starter Repo
===============================================================================
*/
#include <stdint.h>
#include "stm32f1xx.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_dma.h"

typedef struct {
    USART_TypeDef*      usart;
    GPIO_TypeDef*       tx_port; uint32_t tx_pin;
    GPIO_TypeDef*       rx_port; uint32_t rx_pin;
    /* DMA channels (F1 uses channels rather than streams) */
    DMA_TypeDef*        dma;
    uint32_t            dma_rx_channel; /* LL_DMA_CHANNEL_5 for USART1_RX */
    uint32_t            dma_tx_channel; /* LL_DMA_CHANNEL_4 for USART1_TX */
    IRQn_Type           usart_irqn;
    IRQn_Type           dma_rx_irqn;
    IRQn_Type           dma_tx_irqn;
} board_uart_map_t;

const board_uart_map_t* board_uart_get(int logical_id);
void board_init_clocks(void);
void board_init_gpio(void);
void board_init_uart(const board_uart_map_t* m);
