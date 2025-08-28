
#include <stdint.h>
__attribute__((weak)) volatile uint8_t stm32f7_uart0_rx_buf[512] = {0};
__attribute__((weak)) const uint16_t   stm32f7_uart0_rx_size = 512;
__attribute__((weak)) volatile uint8_t stm32f7_uart0_tx_buf[512] = {0};
__attribute__((weak)) const uint16_t   stm32f7_uart0_tx_size = 512;

__attribute__((weak)) volatile uint8_t stm32f7_uart1_rx_buf[512] = {0};
__attribute__((weak)) const uint16_t   stm32f7_uart1_rx_size = 512;
__attribute__((weak)) volatile uint8_t stm32f7_uart1_tx_buf[512] = {0};
__attribute__((weak)) const uint16_t   stm32f7_uart1_tx_size = 512;
