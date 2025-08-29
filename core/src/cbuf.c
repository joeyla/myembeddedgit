
/*
===============================================================================
Module:        core.cbuf
Description:   Ring buffer
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
#include "core/err.h"
#include "core/cbuf.h"

/* Defines -------------------------------------------------------------------*/
#define MODULE_ID  0x101
#ifdef MODULE_TAG
#undef MODULE_TAG
#endif
#define MODULE_TAG "core.cbuf"

/* Typedefs ------------------------------------------------------------------*/

/* Private (static) data -----------------------------------------------------*/

/* Private helpers -----------------------------------------------------------*/
static inline uint16_t _inc(uint16_t v, uint16_t size){ return (uint16_t)((v+1u) % size); }

/* Public API ----------------------------------------------------------------*/
void cbuf_init(cbuf_t* c, uint8_t* storage, uint16_t size){
    c->buf = storage; c->size = size; c->head = c->tail = 0;
}

uint16_t cbuf_count(const cbuf_t* c){ return (uint16_t)((c->head + c->size - c->tail) % c->size); }
bool cbuf_empty(const cbuf_t* c){ return c->head == c->tail; }

uint16_t cbuf_write(cbuf_t* c, const uint8_t* data, uint16_t len){
    uint16_t wrote = 0;
    while(wrote < len){
        uint16_t nhead = _inc(c->head, c->size);
        if(nhead == c->tail) break; // full
        c->buf[c->head] = data[wrote++];
        c->head = nhead;
    }
    return wrote;
}

uint16_t cbuf_read(cbuf_t* c, uint8_t* out, uint16_t maxlen){
    uint16_t rd = 0;
    while(rd < maxlen && c->tail != c->head){
        out[rd++] = c->buf[c->tail];
        c->tail = _inc(c->tail, c->size);
    }
    return rd;
}

/* End of file ---------------------------------------------------------------*/
