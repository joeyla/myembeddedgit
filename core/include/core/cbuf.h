
#pragma once
/*
===============================================================================
Header:        core/include/core/cbuf.h
Description:   Lock-free single-producer/single-consumer ring buffer
Author:        Starter Repo
Created:       2025-08-28
===============================================================================
*/
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Typedefs ------------------------------------------------------------------*/
typedef struct {
    uint8_t* buf;
    uint16_t size;
    uint16_t head;
    uint16_t tail;
} cbuf_t;

/* API -----------------------------------------------------------------------*/
void cbuf_init(cbuf_t* c, uint8_t* storage, uint16_t size);
uint16_t cbuf_write(cbuf_t* c, const uint8_t* data, uint16_t len);
uint16_t cbuf_read(cbuf_t* c, uint8_t* out, uint16_t maxlen);
uint16_t cbuf_count(const cbuf_t* c);
bool cbuf_empty(const cbuf_t* c);

#ifdef __cplusplus
} /* extern "C" */
#endif
