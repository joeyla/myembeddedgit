
/*
===============================================================================
Module:        apps.uart_bench
Description:   Simple UART throughput/latency demonstration
===============================================================================
*/
#include <string.h>
#include "core/err.h"
#include "core/log.h"
#include "core/uart_iface.h"
#include "core/clock_iface.h"
#include "board.h"

#define MODULE_ID  0x603
#define MODULE_TAG "apps.uart_bench"

static uint8_t bigbuf[4096];

int main(void){
    board_init_clocks();
    board_init_gpio();

    uart_handle_t* u=0;
    uart_config_t cfg = { .baud=115200, .databits=8, .stopbits=1, .parity=0 };
    if(uart_open(0, &cfg, &u) < 0){ LOGE("uart_open failed"); for(;;); }
    (void)uart_set_tx_coalesce_min(u, 64);

    /* Prepare buffer */
    for(size_t i=0;i<sizeof(bigbuf);++i) bigbuf[i] = (uint8_t)('A' + (i%26));

    /* Throughput test: send big buffer N times and time it */
    const int iters = 10;
    uint32_t start = clock_millis();
    for(int i=0;i<iters;i++){
        (void)uart_write(u, bigbuf, sizeof(bigbuf), 0, 0);
        (void)uart_flush(u, 1000);
    }
    uint32_t elapsed = clock_millis() - start;
    /* Pseudo print (replace with your logger/ITM) */
    log_printf(LOG_INFO, MODULE_TAG, "TX %d x %u bytes in %lu ms", iters, (unsigned)sizeof(bigbuf), (unsigned long)elapsed);

    /* Latency-ish demo: small writes */
    for(int i=0;i<50;i++){
        (void)uart_write(u, (const uint8_t*)".", 1, 0, 0);
        (void)uart_try_flush(u); /* nudge out */
        clock_delay_ms(1);
    }

    for(;;){}
}
