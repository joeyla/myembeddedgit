
#pragma once
/*
===============================================================================
Header:        core/include/core/log.h
Description:   Tiny logging facade with module tag and severity
Author:        Starter Repo
Created:       2025-08-28
===============================================================================

SECTIONS
- Includes
- Extern C
- Defines
- Typedefs
- Macros
- API
*/

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdarg.h>
#include "core/err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Defines -------------------------------------------------------------------*/
typedef enum {
    LOG_DEBUG = 0,
    LOG_INFO  = 1,
    LOG_WARN  = 2,
    LOG_ERROR = 3
} log_level_t;

/* API -----------------------------------------------------------------------*/
void log_vprintf(log_level_t level, const char* module_tag, const char* fmt, va_list ap);
void log_printf(log_level_t level, const char* module_tag, const char* fmt, ...);

#ifndef MODULE_TAG
#define MODULE_TAG "unknown"
#endif

#define LOGD(fmt, ...) log_printf(LOG_DEBUG, MODULE_TAG, fmt, ##__VA_ARGS__)
#define LOGI(fmt, ...) log_printf(LOG_INFO,  MODULE_TAG, fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) log_printf(LOG_WARN,  MODULE_TAG, fmt, ##__VA_ARGS__)
#define LOGE(fmt, ...) log_printf(LOG_ERROR, MODULE_TAG, fmt, ##__VA_ARGS__)

void log_status(status_t st);

#ifdef __cplusplus
} /* extern "C" */
#endif
