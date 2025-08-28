
/*
===============================================================================
Module:        platform.os.baremetal
Description:   Bare-metal critical section stubs
Author:        Starter Repo
Created:       2025-08-28
===============================================================================
*/
/* Includes ------------------------------------------------------------------*/
#include "core/os_iface.h"

/* Defines -------------------------------------------------------------------*/
#define MODULE_ID  0x501
#define MODULE_TAG "platform.os.baremetal"

/* Public API ----------------------------------------------------------------*/
static unsigned key;
void os_enter_critical(void){ key++; /* TODO: __disable_irq() */ }
void os_exit_critical(void){ if(key) key--; /* TODO: __enable_irq() */ }
