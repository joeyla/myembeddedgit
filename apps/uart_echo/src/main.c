
/*
===============================================================================
Module:        apps.uart_echo
Description:   Minimal echo app wiring UART via interface
Author:        Starter Repo
Created:       2025-08-28
===============================================================================
*/
/* Includes ------------------------------------------------------------------*/
#include "core/err.h"
#include "core/log.h"
#include "core/uart_iface.h"
#include "services/at_engine/at_engine.h"
#include "core/clock_iface.h"
#include "board.h"

/* Defines -------------------------------------------------------------------*/
#define MODULE_ID  0x601
#define MODULE_TAG "apps.uart_echo"

/* Public API ----------------------------------------------------------------*/
int main(void){
    board_init_clocks();
    board_init_gpio();

    uart_config_t cfg = { .baud=115200, .databits=8, .stopbits=1, .parity=0 };
    uart_handle_t* u = 0;
    status_t st = uart_open(0, &cfg, &u);
    if(st < 0){
        LOGE("uart_open failed"); log_status(st);
        for(;;);
    }

    at_engine_t eng; at_engine_init(&eng, u);
    at_engine_send(&eng, "UART echo demo ready\r\n");
    (void)uart_set_tx_coalesce_min(u, 32); /* coalesce tiny writes */
    (void)uart_flush(u, 100); /* ensure greeting is out */

    for(;;){
        uint8_t buf[64];
        uint16_t n = 0;
        st = uart_read(u, buf, sizeof buf, &n);
        if(st < 0){
            LOGE("uart_read failed"); log_status(st);
        } else if(n){
            (void)uart_write(u, buf, n, 0, 0);
            (void)uart_flush(u, 10); /* example: keep latency low for interactive echo */
        }
        clock_delay_ms(1);
    }
}
