
#pragma once
/*
===============================================================================
Header:        core/include/core/clock_iface.h
Description:   Abstract timebase
Author:        Starter Repo
Created:       2025-08-28
===============================================================================
*/
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/* API -----------------------------------------------------------------------*/
uint32_t clock_millis(void);
void     clock_delay_ms(uint32_t ms);

#ifdef __cplusplus
} /* extern "C" */
#endif
