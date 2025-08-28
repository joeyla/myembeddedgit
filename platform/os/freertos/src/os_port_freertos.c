
/*
===============================================================================
Module:        platform.os.freertos
Description:   FreeRTOS implementation for critical sections (task-aware)
Author:        Starter Repo
===============================================================================
*/
#include "platform/os/freertos/include/os_port.h"

#define MODULE_ID  0x502
#define MODULE_TAG "platform.os.freertos"

void os_enter_critical(void){ taskENTER_CRITICAL(); }
void os_exit_critical(void){ taskEXIT_CRITICAL(); }
