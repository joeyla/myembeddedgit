
#pragma once
/*
===============================================================================
Header:        services/at_engine/include/at_engine.h
Description:   Minimal AT engine stub
Author:        Starter Repo
Created:       2025-08-28
===============================================================================
*/
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "core/uart_iface.h"

#ifdef __cplusplus
extern "C" {
#endif

/* portable inline for ARMCC/ARMCLANG/GCC */
#ifndef STATIC_INLINE
#  if defined(__CC_ARM) || defined(__ARMCC_VERSION)
#    define STATIC_INLINE static __inline
#  else
#    define STATIC_INLINE static inline
#  endif
#endif

typedef struct {
    uart_handle_t* uart;
} at_engine_t;

STATIC_INLINE void at_engine_init(at_engine_t* e, uart_handle_t* u){
    e->uart = u;
}
STATIC_INLINE void at_engine_send(at_engine_t* e, const char* s){
    const uint8_t* p = (const uint8_t*)s;
    if(!e || !e->uart || !s) return;
    while(*p){
        (void)uart_write(e->uart, p, 1, 0, 0);
        ++p;
    }
}

#ifdef __cplusplus
} /* extern "C" */
#endif
