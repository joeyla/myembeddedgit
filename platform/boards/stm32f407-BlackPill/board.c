
/*
===============================================================================
Module:        platform.board.stm32f407_custom
Description:   Board mapping for STM32F407: USART2 on PA2/PA3 + DMA1 Stream5 RX
Author:        Starter Repo
Created:       2025-08-28
===============================================================================
*/
/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_ll_rcc.h"

/* Defines -------------------------------------------------------------------*/
#define MODULE_ID  0x401
#ifdef MODULE_TAG
#undef MODULE_TAG
#endif
#define MODULE_TAG "platform.board.stm32f407_custom"

/* Private (static) data -----------------------------------------------------*/
static const board_uart_map_t uarts[] = {
    { /* logical 0: USART2 on PA2/PA3 (AF7) */
        .usart = USART2,
        .tx_port = GPIOA, .tx_pin = LL_GPIO_PIN_2, .tx_af = LL_GPIO_AF_7,
        .rx_port = GPIOA, .rx_pin = LL_GPIO_PIN_3, .rx_af = LL_GPIO_AF_7,
        .dma = DMA1,

        .dma_rx_stream = DMA1_Stream5, .dma_rx_channel = LL_DMA_CHANNEL_4, /* USART2_RX */
        .dma_tx_stream = DMA1_Stream6, .dma_tx_channel = LL_DMA_CHANNEL_4, /* USART2_TX */

        .usart_irqn = USART2_IRQn,
        .dma_rx_irqn = DMA1_Stream5_IRQn,
        .dma_tx_irqn = DMA1_Stream6_IRQn
    },
    { /* logical 1: USART3 on PD8/PD9 (AF7) */
        .usart = USART3,
        .tx_port = GPIOD, .tx_pin = LL_GPIO_PIN_8, .tx_af = LL_GPIO_AF_7,
        .rx_port = GPIOD, .rx_pin = LL_GPIO_PIN_9, .rx_af = LL_GPIO_AF_7,
        .dma = DMA1,

        .dma_rx_stream = DMA1_Stream1, .dma_rx_channel = LL_DMA_CHANNEL_4, /* USART3_RX */
        .dma_tx_stream = DMA1_Stream3, .dma_tx_channel = LL_DMA_CHANNEL_4, /* USART3_TX */

        .usart_irqn = USART3_IRQn,
        .dma_rx_irqn = DMA1_Stream1_IRQn,
        .dma_tx_irqn = DMA1_Stream3_IRQn
    }
};;

/* Public API ----------------------------------------------------------------*/
const board_uart_map_t* board_uart_get(int logical_id){
    if (logical_id < 0 || logical_id >= (int)(sizeof uarts/sizeof uarts[0])) return 0;
    return &uarts[logical_id];
}

void board_init_clocks(void){
    /* NOTE: System clock tree setup is project-specific.
       Provide your SystemClock_Config() elsewhere. */
    
}

void board_init_gpio(void){
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
}

void board_init_uart(const board_uart_map_t* m){
    /* GPIO */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

    LL_GPIO_SetPinMode(m->tx_port, m->tx_pin, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(m->tx_port, m->tx_pin, m->tx_af);
    LL_GPIO_SetPinSpeed(m->tx_port, m->tx_pin, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinOutputType(m->tx_port, m->tx_pin, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(m->tx_port, m->tx_pin, LL_GPIO_PULL_UP);

    LL_GPIO_SetPinMode(m->rx_port, m->rx_pin, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(m->rx_port, m->rx_pin, m->rx_af);
    LL_GPIO_SetPinSpeed(m->rx_port, m->rx_pin, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinOutputType(m->rx_port, m->rx_pin, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(m->rx_port, m->rx_pin, LL_GPIO_PULL_UP);

    /* Clocks */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
}
