
#pragma once
/*
===============================================================================
Module:        core/err.h
Description:   status_t + 32-bit encoded error (sev|module|code)
Author:        Starter Repo
Created:       2025-08-28
===============================================================================

SECTIONS
- Includes
- Defines
- Typedefs
- Macros
- API
- Notes
*/

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Defines -------------------------------------------------------------------*/
#define ERR_OK                ((status_t)0)
#define ERR_SEVERITY_SHIFT    (28u)
#define ERR_MODULE_SHIFT      (16u)
#define ERR_CODE_SHIFT        (0u)

/* Typedefs ------------------------------------------------------------------*/
typedef int32_t status_t;

typedef enum {
    ERR_SEV_OK   = 0,
    ERR_SEV_INFO = 1,
    ERR_SEV_WARN = 2,
    ERR_SEV_ERR  = 3
} err_severity_t;

/* Macros --------------------------------------------------------------------*/
#define ERR_MAKE(sev, module, code)       ((status_t)((((uint32_t)(sev)    & 0xFu)   << ERR_SEVERITY_SHIFT) |                 (((uint32_t)(module) & 0xFFFu) << ERR_MODULE_SHIFT)   |                 (((uint32_t)(code)   & 0xFFFFu)<< ERR_CODE_SHIFT))    |      ((sev)==ERR_SEV_ERR ? (int32_t)0x80000000 : 0))

#define ERR_SEVERITY(x)   ((uint32_t)((((uint32_t)(x)) >> ERR_SEVERITY_SHIFT) & 0xFu))
#define ERR_MODULE(x)     ((uint32_t)((((uint32_t)(x)) >> ERR_MODULE_SHIFT) & 0xFFFu))
#define ERR_CODE(x)       ((uint32_t)((((uint32_t)(x)) >> ERR_CODE_SHIFT) & 0xFFFFu))

#define ERR_INFO(module, code)  ERR_MAKE(ERR_SEV_INFO, (module), (code))
#define ERR_WARN(module, code)  ERR_MAKE(ERR_SEV_WARN, (module), (code))
#define ERR_ERR(module, code)   ERR_MAKE(ERR_SEV_ERR,  (module), (code))

#define RETURN_IF_ERROR(st) do{ if((st) < 0) return (st); }while(0)
#define GOTO_IF_ERROR(st, label) do{ if((st) < 0) goto label; }while(0)

/* API -----------------------------------------------------------------------*/
#ifndef MODULE_TAG
#define MODULE_TAG "unknown"
#endif

/* Notes ---------------------------------------------------------------------*/
