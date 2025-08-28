
#pragma once
/*
===============================================================================
Header:        core/include/core/uart_iface.h
Description:   Portable UART interface with status_t error scheme
Author:        Starter Repo
Created:       2025-08-28
===============================================================================

SECTIONS
- Includes
- Extern C
- Defines
- Typedefs
- Macros
- API
*/

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "core/err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Typedefs ------------------------------------------------------------------*/
typedef enum {
    UART_OK = 0,
    UART_E_BUSY,
    UART_E_TIMEOUT,
    UART_E_IO,
    UART_E_CONFIG
} uart_status_t;

typedef void (*uart_rx_cb_t)(const uint8_t *data, uint16_t len, void *user);
typedef void (*uart_tx_done_cb_t)(void *user);

typedef struct {
    uint32_t baud;
    uint8_t  databits;   // 8 default
    uint8_t  stopbits;   // 1 or 2
    uint8_t  parity;     // 0:none, 1:odd, 2:even
} uart_config_t;

typedef struct uart_handle_s uart_handle_t;

/* API -----------------------------------------------------------------------*/
status_t uart_open(int logical_id, const uart_config_t *cfg, uart_handle_t** out);
status_t uart_close(uart_handle_t*);

status_t uart_write(uart_handle_t*, const uint8_t *buf, uint16_t len,
                    uart_tx_done_cb_t done, void *user);

status_t uart_read(uart_handle_t*, uint8_t *buf, uint16_t maxlen, uint16_t* out_len);
status_t uart_set_rx_callback(uart_handle_t*, uart_rx_cb_t cb, void *user);
status_t uart_set_baud(uart_handle_t*, uint32_t baud);

#ifdef __cplusplus
} /* extern "C" */
#endif

status_t uart_flush(uart_handle_t*, uint32_t timeout_ms);
/* Kick TX if idle but pending data exists; never waits. */
status_t uart_try_flush(uart_handle_t*);

/* Set minimum contiguous bytes before starting TX DMA when idle (coalescing). 0 disables coalescing. */
status_t uart_set_tx_coalesce_min(uart_handle_t*, uint16_t bytes);

