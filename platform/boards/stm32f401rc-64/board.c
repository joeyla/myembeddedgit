#include "board.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"

/* ---- Pin mux helper ---- */
static void gpio_set_af(GPIO_TypeDef* port, uint32_t pin_mask, uint32_t af)
{
    if (pin_mask & 0xFF00U) { LL_GPIO_SetAFPin_8_15(port, pin_mask, af); }
    else                    { LL_GPIO_SetAFPin_0_7 (port, pin_mask, af); }
    LL_GPIO_SetPinMode       (port, pin_mask, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinOutputType (port, pin_mask, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinSpeed      (port, pin_mask, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinPull       (port, pin_mask, LL_GPIO_PULL_UP);
}

/* ---- STM32F401RC LQFP-64 UART/DMA map ----
   USART1: PA9/PA10 AF7   DMA2 Stream7/2 Ch4 (TX/RX)
   USART2: PA2/PA3  AF7   DMA1 Stream6/5 Ch4 (TX/RX)
   USART6: PC6/PC7  AF8   DMA2 Stream6/1 Ch5 (TX/RX)
*/
static const board_uart_map_t uarts[] = {
    {   /* logical 0 -> USART1 */
        .usart = USART1,
        .tx_port = GPIOA, .tx_pin = LL_GPIO_PIN_9,  .tx_af = LL_GPIO_AF_7,
        .rx_port = GPIOA, .rx_pin = LL_GPIO_PIN_10, .rx_af = LL_GPIO_AF_7,
        .dma = DMA2,
        .dma_rx_stream = DMA2_Stream2, .dma_rx_channel = LL_DMA_CHANNEL_4,
        .dma_tx_stream = DMA2_Stream7, .dma_tx_channel = LL_DMA_CHANNEL_4,
        .usart_irqn = USART1_IRQn,
        .dma_rx_irqn = DMA2_Stream2_IRQn,
        .dma_tx_irqn = DMA2_Stream7_IRQn
    },
    {   /* logical 1 -> USART2 */
        .usart = USART2,
        .tx_port = GPIOA, .tx_pin = LL_GPIO_PIN_2, .tx_af = LL_GPIO_AF_7,
        .rx_port = GPIOA, .rx_pin = LL_GPIO_PIN_3, .rx_af = LL_GPIO_AF_7,
        .dma = DMA1,
        .dma_rx_stream = DMA1_Stream5, .dma_rx_channel = LL_DMA_CHANNEL_4,
        .dma_tx_stream = DMA1_Stream6, .dma_tx_channel = LL_DMA_CHANNEL_4,
        .usart_irqn = USART2_IRQn,
        .dma_rx_irqn = DMA1_Stream5_IRQn,
        .dma_tx_irqn = DMA1_Stream6_IRQn
    },
    {   /* logical 2 -> USART6 */
        .usart = USART6,
        .tx_port = GPIOC, .tx_pin = LL_GPIO_PIN_6, .tx_af = LL_GPIO_AF_8,
        .rx_port = GPIOC, .rx_pin = LL_GPIO_PIN_7, .rx_af = LL_GPIO_AF_8,
        .dma = DMA2,
        .dma_rx_stream = DMA2_Stream1, .dma_rx_channel = LL_DMA_CHANNEL_5,
        .dma_tx_stream = DMA2_Stream6, .dma_tx_channel = LL_DMA_CHANNEL_5,
        .usart_irqn = USART6_IRQn,
        .dma_rx_irqn = DMA2_Stream1_IRQn,
        .dma_tx_irqn = DMA2_Stream6_IRQn
    }
};

unsigned board_get_uart_count(void) {
    return (unsigned)(sizeof(uarts)/sizeof(uarts[0]));
}

const board_uart_map_t* board_get_uart_map(unsigned logical_id)
{
    return (logical_id < board_get_uart_count()) ? &uarts[logical_id] : 0;
}

/* ---- Clocks for GPIO/DMA/USART blocks we use ---- */
void board_init_clocks(void)
{
    /* GPIO */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB); /* optional */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

    /* DMA */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

    /* USARTs: 1 & 6 on APB2, 2 on APB1 */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART6);
}

/* ---- Pin mux for all defined UARTs ---- */
void board_init_gpio(void)
{
    for (unsigned i = 0; i < board_get_uart_count(); ++i) {
        const board_uart_map_t* m = &uarts[i];
        gpio_set_af(m->tx_port, m->tx_pin, m->tx_af);
        gpio_set_af(m->rx_port, m->rx_pin, m->rx_af);
    }
}

/* ---- Optional per-UART NVIC (if your driver calls this) ---- */
void board_init_uart(const board_uart_map_t* m)
{
    if (!m) return;
    NVIC_SetPriority(m->usart_irqn, 5);   NVIC_EnableIRQ(m->usart_irqn);
    NVIC_SetPriority(m->dma_rx_irqn, 6);  NVIC_EnableIRQ(m->dma_rx_irqn);
    NVIC_SetPriority(m->dma_tx_irqn, 6);  NVIC_EnableIRQ(m->dma_tx_irqn);
}
