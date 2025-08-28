
#include "board.h"
#define MODULE_ID 0x402
#define MODULE_TAG "platform.board.stm32f103_nucleo"

static const board_uart_map_t uarts[] = {
    { USART1, GPIOA, LL_GPIO_PIN_9, GPIOA, LL_GPIO_PIN_10, DMA1, LL_DMA_CHANNEL_5, LL_DMA_CHANNEL_4, USART1_IRQn, DMA1_Channel5_IRQn, DMA1_Channel4_IRQn },
    { USART2, GPIOA, LL_GPIO_PIN_2, GPIOA, LL_GPIO_PIN_3,  DMA1, LL_DMA_CHANNEL_6, LL_DMA_CHANNEL_7, USART2_IRQn, DMA1_Channel6_IRQn, DMA1_Channel7_IRQn }
};

const board_uart_map_t* board_uart_get(int logical_id){ 
    if (logical_id < 0 || logical_id >= (int)(sizeof uarts/sizeof uarts[0])) return 0;
    return &uarts[logical_id]; 
}

void board_init_clocks(void){
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
}
void board_init_gpio(void){
    /* USART1 */
    LL_GPIO_InitTypeDef gpio = {0};
    gpio.Pin = LL_GPIO_PIN_9; gpio.Mode = LL_GPIO_MODE_ALTERNATE; gpio.Speed = LL_GPIO_SPEED_FREQ_HIGH; gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = LL_GPIO_PIN_10; gpio.Mode = LL_GPIO_MODE_FLOATING;
    LL_GPIO_Init(GPIOA, &gpio);
    /* USART2 */
    gpio.Pin = LL_GPIO_PIN_2; gpio.Mode = LL_GPIO_MODE_ALTERNATE; gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = LL_GPIO_PIN_3; gpio.Mode = LL_GPIO_MODE_FLOATING;
    LL_GPIO_Init(GPIOA, &gpio);
}
void board_init_uart(const board_uart_map_t* m){ (void)m; }
