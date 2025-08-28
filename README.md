
# Embedded Starter Repo (v3)

Layered, portable structure with standardized file sections, module-based error scheme (`status_t`), and lightweight logging.


## v5 Additions
- STM32F4 USART2: RX via DMA circular, TX via DMA ring (non-blocking).
- FreeRTOS OS port and `apps/uart_echo_rtos` tasked example.
- Board stubs for STM32F103 Nucleo and STM32F767 Disco with suggested pin maps.
- Keil projects for bare-metal and RTOS variants.


New in v7: TX coalescing + uart_flush().


## Link-time ring size configuration
Each MCU driver references **weak default buffer symbols** you can override in your application to control TX/RX ring sizes (and placement). Define strong symbols with the same names in any of your app .c files:
```c
// Example: override F4 UART0 rings
volatile uint8_t stm32f4_uart0_rx_buf[1024];
const uint16_t   stm32f4_uart0_rx_size = 1024;

volatile uint8_t stm32f4_uart0_tx_buf[2048];
const uint16_t   stm32f4_uart0_tx_size = 2048;
```
> Sizes must be powers of two. The driver computes masks at runtime.

## New non-blocking flush
- `uart_try_flush(uart)` kicks TX if idle but will **not** wait. Use before sleep or when you want to minimize latency without blocking.
