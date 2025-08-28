
#include <stdint.h>
__attribute__((weak)) volatile uint8_t stm32f1_uart0_rx_buf[256] = {0};
__attribute__((weak)) const uint16_t   stm32f1_uart0_rx_size = 256;
__attribute__((weak)) volatile uint8_t stm32f1_uart0_tx_buf[256] = {0};
__attribute__((weak)) const uint16_t   stm32f1_uart0_tx_size = 256;

__attribute__((weak)) volatile uint8_t stm32f1_uart1_rx_buf[256] = {0};
__attribute__((weak)) const uint16_t   stm32f1_uart1_rx_size = 256;
__attribute__((weak)) volatile uint8_t stm32f1_uart1_tx_buf[256] = {0};
__attribute__((weak)) const uint16_t   stm32f1_uart1_tx_size = 256;
