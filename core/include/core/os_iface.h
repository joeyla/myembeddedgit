
#pragma once
/*
===============================================================================
Header:        core/include/core/os_iface.h
Description:   Minimal critical section abstraction
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
void os_enter_critical(void);
void os_exit_critical(void);

#ifdef __cplusplus
} /* extern "C" */
#endif
