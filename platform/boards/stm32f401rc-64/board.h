#pragma once
#include <stdint.h>
#include "stm32f4xx.h"          /* USART_TypeDef, GPIO_TypeDef, IRQn_Type */
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_dma.h"

/* Logical UART mapping (pins, DMA, IRQs) */
typedef struct {
    USART_TypeDef*         usart;

    GPIO_TypeDef*          tx_port; uint32_t tx_pin; uint32_t tx_af;
    GPIO_TypeDef*          rx_port; uint32_t rx_pin; uint32_t rx_af;

    DMA_TypeDef*           dma;
    DMA_Stream_TypeDef*    dma_rx_stream; 
    uint32_t dma_rx_channel;
    DMA_Stream_TypeDef*    dma_tx_stream; 
    uint32_t dma_tx_channel;
  
    IRQn_Type              usart_irqn;
    IRQn_Type              dma_rx_irqn;
    IRQn_Type              dma_tx_irqn;
} board_uart_map_t;

/* Board API expected by your app/driver */
#ifdef __cplusplus
extern "C" {
#endif

void board_init_clocks(void);
void board_init_gpio(void);
void board_init_uart(const board_uart_map_t* m);

unsigned board_get_uart_count(void);
const board_uart_map_t* board_get_uart_map(unsigned logical_id);

/* convenience alias some code calls */
static inline const board_uart_map_t* board_uart_get(unsigned logical_id) {
    return board_get_uart_map(logical_id);
}

#ifdef __cplusplus
}
#endif
