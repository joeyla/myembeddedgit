
/*
===============================================================================
Module:        apps.uart_echo_rtos
Description:   FreeRTOS-based echo task demo
Author:        Starter Repo
===============================================================================
*/
#include "core/err.h"
#include "core/log.h"
#include "core/uart_iface.h"
#include "platform/os/freertos/include/os_port.h"
#include "board.h"
#include "FreeRTOS.h"
#include "task.h"

#define MODULE_ID  0x602
#define MODULE_TAG "apps.uart_echo_rtos"

static void vEchoTask(void* arg){
    (void)arg;
    uart_handle_t* u = 0;
    uart_config_t cfg = { .baud=115200, .databits=8, .stopbits=1, .parity=0 };
    if(uart_open(0, &cfg, &u) < 0){
        LOGE("uart_open failed");
        vTaskDelete(NULL);
    }
    const char* hello = "UART echo (RTOS) ready\r\n";
    (void)uart_set_tx_coalesce_min(u, 32);
    (void)uart_write(u, (const uint8_t*)hello, (uint16_t)strlen(hello), 0, 0);
    (void)uart_flush(u, 100);

    uint8_t buf[128];
    for(;;){
        uint16_t n=0;
        (void)uart_read(u, buf, sizeof buf, &n);
        if(n){
            (void)uart_write(u, buf, n, 0, 0);
            (void)uart_flush(u, 5);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

int main(void){
    board_init_clocks();
    board_init_gpio();

    xTaskCreate(vEchoTask, "echo", 512, NULL, tskIDLE_PRIORITY+1, NULL);
    vTaskStartScheduler();
    for(;;);
}
