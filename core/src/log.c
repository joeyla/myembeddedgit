
/*
===============================================================================
Module:        core.src.log
Description:   Default weak logging to semihosting/printf
Author:        Starter Repo
Created:       2025-08-28
===============================================================================

SECTIONS
- Includes
- Defines
- Typedefs
- Private (static) data
- Private helpers
- Public API
- End of file
*/

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdarg.h>
#include "core/log.h"

/* Defines -------------------------------------------------------------------*/
#define MODULE_ID  0x103
#define MODULE_TAG "core.log"

/* Typedefs ------------------------------------------------------------------*/

/* Private (static) data -----------------------------------------------------*/

/* Private helpers -----------------------------------------------------------*/

/* Public API ----------------------------------------------------------------*/
__attribute__((weak)) void log_vprintf(log_level_t level, const char* module_tag, const char* fmt, va_list ap){
    (void)level;
    printf("[%s] ", module_tag ? module_tag : "mod");
    vprintf(fmt, ap);
    printf("\r\n");
}

void log_printf(log_level_t level, const char* module_tag, const char* fmt, ...){
    va_list ap;
    va_start(ap, fmt);
    log_vprintf(level, module_tag, fmt, ap);
    va_end(ap);
}

void log_status(status_t st){
    unsigned sev = ERR_SEVERITY(st);
    unsigned mod = ERR_MODULE(st);
    unsigned code= ERR_CODE(st);
    printf("[status] sev=%u mod=0x%03X code=0x%04X%s\r\n",
           sev, mod, code, (st<0) ? " (ERR)" : "");
}

/* End of file ---------------------------------------------------------------*/
