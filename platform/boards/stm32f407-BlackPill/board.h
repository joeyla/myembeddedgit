
#pragma once
/*
===============================================================================
Header:        platform/boards/stm32f407-custom/board.h
Description:   Board mapping API (F407 custom). USART2 on PA2/PA3, DMA1 Stream5 RX.
Author:        Starter Repo
Created:       2025-08-28
===============================================================================
*/
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_bus.h"

typedef struct {
    USART_TypeDef*         usart;

    GPIO_TypeDef*          tx_port; uint32_t tx_pin; uint32_t tx_af;
    GPIO_TypeDef*          rx_port; uint32_t rx_pin; uint32_t rx_af;

    DMA_TypeDef*           dma;
    DMA_Stream_TypeDef*    dma_rx_stream; uint32_t dma_rx_channel;
    DMA_Stream_TypeDef*    dma_tx_stream; uint32_t dma_tx_channel;

    IRQn_Type              usart_irqn;
    IRQn_Type              dma_rx_irqn;
    IRQn_Type              dma_tx_irqn;
} board_uart_map_t;

const board_uart_map_t* board_uart_get(int logical_id);
void board_init_clocks(void);
void board_init_gpio(void);
void board_init_uart(const board_uart_map_t* m);
