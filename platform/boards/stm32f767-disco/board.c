
#include "board.h"
#define MODULE_ID 0x403
#define MODULE_TAG "platform.board.stm32f767_disco"

/* TODO: Fill with your actual USART/DMA mapping. The below is an example placeholder. */
static const board_uart_map_t uarts[] = {
    { USART3, GPIOD, LL_GPIO_PIN_8, LL_GPIO_AF_7, GPIOD, LL_GPIO_PIN_9, LL_GPIO_AF_7,
      DMA1, DMA1_Stream1, LL_DMA_CHANNEL_4, DMA1_Stream3, LL_DMA_CHANNEL_4,
      USART3_IRQn, DMA1_Stream1_IRQn, DMA1_Stream3_IRQn }
, 
    { USART6, GPIOG, LL_GPIO_PIN_14, LL_GPIO_AF_8, GPIOG, LL_GPIO_PIN_9, LL_GPIO_AF_8,
      DMA2, DMA2_Stream1, LL_DMA_CHANNEL_5, DMA2_Stream6, LL_DMA_CHANNEL_5,
      USART6_IRQn, DMA2_Stream1_IRQn, DMA2_Stream6_IRQn }
};

const board_uart_map_t* board_uart_get(int logical_id){ (void)logical_id; return &uarts[0]; }

void board_init_clocks(void){
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
}
void board_init_gpio(void){
    LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(GPIOD, LL_GPIO_PIN_8, LL_GPIO_AF_7);
    LL_GPIO_SetPinSpeed(GPIOD, LL_GPIO_PIN_8, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinPull(GPIOD, LL_GPIO_PIN_8, LL_GPIO_PULL_UP);

    LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(GPIOD, LL_GPIO_PIN_9, LL_GPIO_AF_7);
    LL_GPIO_SetPinSpeed(GPIOD, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinPull(GPIOD, LL_GPIO_PIN_9, LL_GPIO_PULL_UP);
}
void board_init_uart(const board_uart_map_t* m){ (void)m; }
