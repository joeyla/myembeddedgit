#include <stdint.h>

/* after: F401 -> no special section; others keep .uartbufs */
#if defined(STM32F401xC) || defined(STM32F401xE)
  #define SEC_UARTBUF /* empty: place in default SRAM at 0x20000000 */
#else
  #define SEC_UARTBUF __attribute__((section(".uartbufs")))
#endif

/* Default sizes (power of two) — override by providing your own object file */
#ifndef STM32F4_UART0_RX_SIZE
#define STM32F4_UART0_RX_SIZE 1024u
#endif
#ifndef STM32F4_UART0_TX_SIZE
#define STM32F4_UART0_TX_SIZE 1024u
#endif

SEC_UARTBUF uint8_t stm32f4_uart0_rx_buf[STM32F4_UART0_RX_SIZE];
SEC_UARTBUF uint8_t stm32f4_uart0_tx_buf[STM32F4_UART0_TX_SIZE];

const uint16_t stm32f4_uart0_rx_size = (uint16_t)sizeof(stm32f4_uart0_rx_buf);
const uint16_t stm32f4_uart0_tx_size = (uint16_t)sizeof(stm32f4_uart0_tx_buf);

/* Optionally provide UART1 as well if you use logical_id=1 */
SEC_UARTBUF uint8_t stm32f4_uart1_rx_buf[1024];
SEC_UARTBUF uint8_t stm32f4_uart1_tx_buf[1024];
const uint16_t stm32f4_uart1_rx_size = (uint16_t)sizeof(stm32f4_uart1_rx_buf);
const uint16_t stm32f4_uart1_tx_size = (uint16_t)sizeof(stm32f4_uart1_tx_buf);

SEC_UARTBUF uint8_t stm32f4_uart2_rx_buf[1024];
SEC_UARTBUF uint8_t stm32f4_uart2_tx_buf[1024];
const uint16_t stm32f4_uart2_rx_size = (uint16_t)sizeof(stm32f4_uart2_rx_buf);
const uint16_t stm32f4_uart2_tx_size = (uint16_t)sizeof(stm32f4_uart2_tx_buf);
