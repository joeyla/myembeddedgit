
#pragma once
/*
===============================================================================
Header:        core/include/core/assert.h
Description:   Minimal assert hook
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
#include <stdlib.h>
#ifdef __cplusplus
extern "C" {
#endif

/* Defines -------------------------------------------------------------------*/
#ifndef CORE_ASSERT
#define CORE_ASSERT(x) do{ if(!(x)){ abort(); } }while(0)
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
