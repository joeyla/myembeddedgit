#include <stdint.h>

/* Place in .uartbufs for CCM/fast RAM via scatter file */
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
  #define SEC(a) __attribute__((section(a)))
#else
  #define SEC(a) __attribute__((section(a)))
#endif

/* Default sizes (power of two) — override by providing your own object file */
#ifndef STM32F4_UART0_RX_SIZE
#define STM32F4_UART0_RX_SIZE 1024u
#endif
#ifndef STM32F4_UART0_TX_SIZE
#define STM32F4_UART0_TX_SIZE 1024u
#endif

SEC(".uartbufs") uint8_t stm32f4_uart0_rx_buf[STM32F4_UART0_RX_SIZE];
SEC(".uartbufs") uint8_t stm32f4_uart0_tx_buf[STM32F4_UART0_TX_SIZE];

const uint16_t stm32f4_uart0_rx_size = (uint16_t)sizeof(stm32f4_uart0_rx_buf);
const uint16_t stm32f4_uart0_tx_size = (uint16_t)sizeof(stm32f4_uart0_tx_buf);

/* Optionally provide UART1 as well if you use logical_id=1 */
SEC(".uartbufs") uint8_t stm32f4_uart1_rx_buf[1024];
SEC(".uartbufs") uint8_t stm32f4_uart1_tx_buf[1024];
const uint16_t stm32f4_uart1_rx_size = (uint16_t)sizeof(stm32f4_uart1_rx_buf);
const uint16_t stm32f4_uart1_tx_size = (uint16_t)sizeof(stm32f4_uart1_tx_buf);

