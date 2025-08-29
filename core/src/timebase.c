
/*
===============================================================================
Module:        core.timebase
Description:   Weak stubs for timebase
Author:        Starter Repo
Created:       2025-08-28
===============================================================================
*/
/* Includes ------------------------------------------------------------------*/
#include "core/clock_iface.h"
#include "core/err.h"

/* Defines -------------------------------------------------------------------*/
#define MODULE_ID  0x102
#ifdef MODULE_TAG
#undef MODULE_TAG
#endif
#define MODULE_TAG "core.timebase"

/* Public API ----------------------------------------------------------------*/
__attribute__((weak)) uint32_t clock_millis(void){ return 0; }
__attribute__((weak)) void     clock_delay_ms(uint32_t ms){ (void)ms; }
/* End of file ---------------------------------------------------------------*/
