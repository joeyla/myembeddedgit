
/*
===============================================================================
Module:        platform.mcu.stm32f4.uart
Description:   STM32F4 UART with RX DMA circular + TX DMA ring + per-write callbacks
Author:        Starter Repo
===============================================================================
*/
/* Includes ------------------------------------------------------------------*/
#include "core/err.h"
#include "core/log.h"
#include "core/os_iface.h"
#include "core/uart_iface.h"
#include "core/clock_iface.h"   // add this near the top
#include "board.h"

#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_utils.h"

/* portable inline for ARMCC/ARMCLANG/GCC */
#ifndef STATIC_INLINE
#  if defined(__CC_ARM) || defined(__ARMCC_VERSION)
#    define STATIC_INLINE static __inline
#  else
#    define STATIC_INLINE static inline
#  endif
#endif

/* Defines -------------------------------------------------------------------*/
#define MODULE_ID   0x301
#ifdef MODULE_TAG
#undef MODULE_TAG
#endif
#define MODULE_TAG  "platform.mcu.stm32f4.uart"

#define RX_DMA_BUF_SIZE 512u  /* must be power of two */
#define TX_RING_SIZE    512u  /* must be power of two */
#define TX_REQ_Q_SIZE   8u
#define TX_COALESCE_MIN  32u

/* Typedefs ------------------------------------------------------------------*/
typedef struct {
    uint16_t remaining;
    uart_tx_done_cb_t cb;
    void* user;
} tx_req_t;

struct uart_handle_s {
    int logical_id;
    const board_uart_map_t* map;

    /* RX via DMA circular buffer */
    volatile uint8_t* rx_buf; volatile uint16_t rx_tail; volatile uint16_t rx_size; volatile uint16_t rx_mask; /* software tail (consumed) */

    /* TX via DMA from software ring */
    volatile uint8_t* tx_ring; volatile uint16_t tx_head; volatile uint16_t tx_tail; volatile uint16_t tx_size; volatile uint16_t tx_mask;
    volatile uint8_t  tx_busy;
    volatile uint16_t tx_active_len; /* currently programmed DMA length */
    volatile uint16_t tx_coalesce_min; /* runtime threshold */

    /* Per-write completion queue */
    tx_req_t tx_req_q[TX_REQ_Q_SIZE];
    volatile uint8_t  tx_req_head;
    volatile uint8_t  tx_req_tail;

    /* Optional callbacks */
    uart_rx_cb_t rx_cb; void* rx_user;
};

/* Private (static) data -----------------------------------------------------*/

    /* Link-time configurable buffers (override these symbols to change sizes/placement) */
extern volatile uint8_t stm32f4_uart0_rx_buf[];
extern const uint16_t stm32f4_uart0_rx_size;

extern volatile uint8_t stm32f4_uart1_rx_buf[];
extern const uint16_t   stm32f4_uart1_rx_size;

extern volatile uint8_t stm32f4_uart0_tx_buf[];
extern const uint16_t stm32f4_uart0_tx_size;

extern volatile uint8_t stm32f4_uart1_tx_buf[];
extern const uint16_t   stm32f4_uart1_tx_size;
    
static struct uart_handle_s handles[3];
static void usart_apply_cfg(USART_TypeDef* U, const uart_config_t* cfg);

/* Private helpers -----------------------------------------------------------*/
STATIC_INLINE uint16_t tx_ring_count(struct uart_handle_s* h){
    return (uint16_t)((h->tx_head + TX_RING_SIZE - h->tx_tail) & h->tx_mask);
}
STATIC_INLINE uint16_t tx_ring_space(struct uart_handle_s* h){
    return (uint16_t)(TX_RING_SIZE - 1u - tx_ring_count(h));
}

/* F4: USART data register address (portable across pack versions) */
static uint32_t usart_dr_addr(USART_TypeDef* U) {
    return (uint32_t)&U->DR;  /* F4 uses DR, not RDR/TDR */
}

/* map LL prescaler enum to divisor */
static uint32_t apb_div_from_presc(uint32_t presc) {
    switch (presc) {
        case LL_RCC_APB1_DIV_1:  return 1;
        case LL_RCC_APB1_DIV_2:  return 2;
        case LL_RCC_APB1_DIV_4:  return 4;
        case LL_RCC_APB1_DIV_8:  return 8;
        case LL_RCC_APB1_DIV_16: return 16;
        /* APB2 uses the same DIV constants */
        default: return 1;
    }
}

static uint32_t get_pclk_freq(USART_TypeDef* U) {
    uint32_t hclk  = SystemCoreClock;  /* require system_stm32f4xx.c */
    uint32_t presc = (U == USART1 || U == USART6)
                     ? LL_RCC_GetAPB2Prescaler()
                     : LL_RCC_GetAPB1Prescaler();
    return hclk / apb_div_from_presc(presc);
}

static void usart_apply_cfg(USART_TypeDef* U, const uart_config_t* cfg)
{
    LL_USART_Disable(U);

    LL_USART_SetTransferDirection(U, LL_USART_DIRECTION_TX_RX);
    LL_USART_ConfigCharacter(U, LL_USART_DATAWIDTH_8B,
                                (cfg->parity==1)?LL_USART_PARITY_ODD:(cfg->parity==2)?LL_USART_PARITY_EVEN:LL_USART_PARITY_NONE,
                                (cfg->stopbits==2)?LL_USART_STOPBITS_2:LL_USART_STOPBITS_1);
    LL_USART_SetOverSampling(U, LL_USART_OVERSAMPLING_16);
    //uint32_t pclk = (U == USART1 || U == USART6) ? LL_RCC_GetAPB2ClockFreq() : LL_RCC_GetAPB1ClockFreq();
    uint32_t pclk = get_pclk_freq(U);
    LL_USART_SetBaudRate(U, pclk, LL_USART_OVERSAMPLING_16, cfg->baud);

    LL_USART_Enable(U);
}

/* Map a DMA_Stream_TypeDef* to its LL stream index */
static uint32_t dma_stream_index(DMA_Stream_TypeDef* s){
#define MAP(ctrl,n) if (s == ctrl##_Stream##n) return LL_DMA_STREAM_##n
    MAP(DMA1,0); MAP(DMA1,1); MAP(DMA1,2); MAP(DMA1,3);
    MAP(DMA1,4); MAP(DMA1,5); MAP(DMA1,6); MAP(DMA1,7);
    MAP(DMA2,0); MAP(DMA2,1); MAP(DMA2,2); MAP(DMA2,3);
    MAP(DMA2,4); MAP(DMA2,5); MAP(DMA2,6); MAP(DMA2,7);
#undef MAP
    return LL_DMA_STREAM_0; /* fallback */
}

static void dma_rx_start(struct uart_handle_s* h){
    const board_uart_map_t* m = h->map;

    uint32_t rxst = dma_stream_index(m->dma_rx_stream);

    LL_DMA_DisableStream              (m->dma, rxst);
    LL_DMA_SetChannelSelection        (m->dma, rxst, m->dma_rx_channel);
    LL_DMA_SetDataTransferDirection   (m->dma, rxst, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetStreamPriorityLevel     (m->dma, rxst, LL_DMA_PRIORITY_HIGH);
    LL_DMA_SetMode                    (m->dma, rxst, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode           (m->dma, rxst, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode           (m->dma, rxst, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize              (m->dma, rxst, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize              (m->dma, rxst, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_SetPeriphAddress           (m->dma, rxst, usart_dr_addr(m->usart));
    LL_DMA_SetMemoryAddress           (m->dma, rxst, (uint32_t)h->rx_buf);
    LL_DMA_SetDataLength              (m->dma, rxst, h->rx_size);
    LL_DMA_EnableIT_HT                (m->dma, rxst);
    LL_DMA_EnableIT_TC                (m->dma, rxst);

    LL_USART_EnableDMAReq_RX(m->usart);
    LL_DMA_EnableStream(m->dma, (uint32_t)m->dma_rx_stream);
}

static uint16_t dma_rx_available(struct uart_handle_s* h){
    uint32_t rxst = dma_stream_index(h->map->dma_rx_stream);
    uint16_t ndtr = (uint16_t)LL_DMA_GetDataLength(h->map->dma, rxst);
    uint16_t head = (uint16_t)((h->rx_size - ndtr) & h->rx_mask);
    uint16_t tail = h->rx_tail;
    if (head >= tail) return (uint16_t)(head - tail);
    return (uint16_t)(RX_DMA_BUF_SIZE - tail + head);
}

static void tx_req_push(struct uart_handle_s* h, uint16_t len, uart_tx_done_cb_t cb, void* user){
    uint8_t next = (uint8_t)((h->tx_req_head + 1u) % TX_REQ_Q_SIZE);
    if(next == h->tx_req_tail){
        /* overflow: drop callback; data copy prevented by caller space check */
        return;
    }
    h->tx_req_q[h->tx_req_head].remaining = len;
    h->tx_req_q[h->tx_req_head].cb = cb;
    h->tx_req_q[h->tx_req_head].user = user;
    h->tx_req_head = next;
}

static void tx_req_consume(struct uart_handle_s* h, uint16_t sent){
    while(sent && h->tx_req_tail != h->tx_req_head){
        tx_req_t* r = &h->tx_req_q[h->tx_req_tail];
        if(sent >= r->remaining){
            sent = (uint16_t)(sent - r->remaining);
            uart_tx_done_cb_t cb = r->cb; void* user = r->user;
            h->tx_req_tail = (uint8_t)((h->tx_req_tail + 1u) % TX_REQ_Q_SIZE);
            if(cb) cb(user);
        }else{
            r->remaining = (uint16_t)(r->remaining - sent);
            sent = 0;
        }
    }
}

static void tx_dma_start_locked(struct uart_handle_s* h){
    if(h->tx_busy) return;
    uint16_t tail = h->tx_tail;
    uint16_t head = h->tx_head;
    if(tail == head) return;

    uint16_t contiguous = (head >= tail) ? (head - tail) : (TX_RING_SIZE - tail);
    /* Coalesce: if idle and segment is small and not wrapping, wait for more data */
    if(h->tx_coalesce_min && contiguous < h->tx_coalesce_min && head >= tail){
        return;
    }
    h->tx_active_len = contiguous;

    const board_uart_map_t* m = h->map;
    uint32_t txst = dma_stream_index(m->dma_tx_stream);

    LL_DMA_DisableStream              (m->dma, txst);
    LL_DMA_SetChannelSelection        (m->dma, txst, m->dma_tx_channel);
    LL_DMA_SetDataTransferDirection   (m->dma, txst, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetStreamPriorityLevel     (m->dma, txst, LL_DMA_PRIORITY_HIGH);
    LL_DMA_SetMode                    (m->dma, txst, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode           (m->dma, txst, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode           (m->dma, txst, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize              (m->dma, txst, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize              (m->dma, txst, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_SetPeriphAddress           (m->dma, txst, usart_dr_addr(m->usart));
    LL_DMA_SetMemoryAddress           (m->dma, txst, (uint32_t)&h->tx_ring[tail]);
    LL_DMA_SetDataLength              (m->dma, txst, contiguous);
    LL_DMA_EnableIT_TC                (m->dma, txst);
    LL_DMA_EnableStream               (m->dma, txst);
    
    h->tx_busy = 1u;
}

/* Public API ----------------------------------------------------------------*/
status_t uart_open(int logical_id, const uart_config_t *cfg, uart_handle_t** out){
    if(!out) return ERR_ERR(MODULE_ID, 0x0001);
    const board_uart_map_t* map = board_uart_get(logical_id);
    if(!map) return ERR_ERR(MODULE_ID, 0x0002);
    board_init_uart(map);

    struct uart_handle_s* h = &handles[logical_id % 3];
    *h = (struct uart_handle_s){0};
    h->logical_id = logical_id;
    h->map = map;
    /* Bind link-time buffers */
    if(logical_id == 0){
        h->rx_buf = stm32f4_uart0_rx_buf; h->rx_size = stm32f4_uart0_rx_size; h->rx_mask = (uint16_t)(h->rx_size - 1u);
        h->tx_ring = stm32f4_uart0_tx_buf; h->tx_size = stm32f4_uart0_tx_size; h->tx_mask = (uint16_t)(h->tx_size - 1u);
    } else {
        h->rx_buf = stm32f4_uart1_rx_buf; h->rx_size = stm32f4_uart1_rx_size; h->rx_mask = (uint16_t)(h->rx_size - 1u);
        h->tx_ring = stm32f4_uart1_tx_buf; h->tx_size = stm32f4_uart1_tx_size; h->tx_mask = (uint16_t)(h->tx_size - 1u);
    }


    h->tx_coalesce_min = 32;
    usart_apply_cfg(map->usart, cfg);
    dma_rx_start(h);

    NVIC_SetPriority(map->dma_rx_irqn, 5);
    NVIC_EnableIRQ(map->dma_rx_irqn);
    NVIC_SetPriority(DMA1_Stream6_IRQn, 5);
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);

    *out = (uart_handle_t*)h;
    return ERR_OK;
}

status_t uart_close(uart_handle_t* h_){
    struct uart_handle_s* h = (struct uart_handle_s*)h_;
    if(!h) return ERR_ERR(MODULE_ID, 0x0003);
    LL_DMA_DisableStream(h->map->dma, (uint32_t)h->map->dma_rx_stream);
    LL_USART_DisableDMAReq_RX(h->map->usart);
    LL_DMA_DisableStream(DMA1, (uint32_t)DMA1_Stream6);
    LL_USART_DisableDMAReq_TX(h->map->usart);
    return ERR_OK;
}

status_t uart_write(uart_handle_t* h_, const uint8_t *buf, uint16_t len,
                    uart_tx_done_cb_t done, void *user){
    struct uart_handle_s* h = (struct uart_handle_s*)h_;
    if(!h || !buf || !len) return ERR_ERR(MODULE_ID, 0x0010);

    os_enter_critical();
    uint16_t space = tx_ring_space(h);
    if(len > space){
        os_exit_critical();
        return ERR_ERR(MODULE_ID, 0x0011); /* no space */
    }
    /* Copy into ring */
    uint16_t head = h->tx_head;
    uint16_t first = (uint16_t)((TX_RING_SIZE - head));
    if(first > len) first = len;
    for(uint16_t i=0;i<first;i++) h->tx_ring[head+i] = buf[i];
    uint16_t rem = (uint16_t)(len - first);
    if(rem){
        for(uint16_t i=0;i<rem;i++) h->tx_ring[i] = buf[first+i];
    }
    h->tx_head = (uint16_t)((head + len) & h->tx_mask);

    /* Queue completion */
    tx_req_push(h, len, done, user);

    /* Kick DMA if idle */
    tx_dma_start_locked(h);
    os_exit_critical();

    return ERR_OK;
}

status_t uart_read(uart_handle_t* h_, uint8_t *buf, uint16_t maxlen, uint16_t* out_len){
    struct uart_handle_s* h = (struct uart_handle_s*)h_;
    if(!h || !buf || !out_len) return ERR_ERR(MODULE_ID, 0x0012);

    uint16_t avail = dma_rx_available(h);
    uint16_t to_copy = (avail < maxlen) ? avail : maxlen;
    uint16_t tail = h->rx_tail;

    for(uint16_t i=0;i<to_copy;i++){
        buf[i] = h->rx_buf[tail];
        tail = (uint16_t)((tail + 1u) & h->rx_mask);
    }
    h->rx_tail = tail;
    *out_len = to_copy;
    return ERR_OK;
}

status_t uart_set_rx_callback(uart_handle_t* h_, uart_rx_cb_t cb, void *user){
    struct uart_handle_s* h = (struct uart_handle_s*)h_;
    if(!h) return ERR_ERR(MODULE_ID, 0x0013);
    h->rx_cb = cb; h->rx_user = user;
    return ERR_OK;
}

status_t uart_set_baud(uart_handle_t* h_, uint32_t baud){
    uart_config_t cfg = { .baud=baud, .databits=8, .stopbits=1, .parity=0 };
    struct uart_handle_s* h = (struct uart_handle_s*)h_;
    if(!h) return ERR_ERR(MODULE_ID, 0x0014);
    
    h->tx_coalesce_min = 32;
    usart_apply_cfg(h->map->usart, &cfg);
    return ERR_OK;
}

/* IRQ Handlers --------------------------------------------------------------*/
void DMA1_Stream5_IRQHandler(void){
    if(LL_DMA_IsActiveFlag_HT5(DMA1)){ LL_DMA_ClearFlag_HT5(DMA1); }
    if(LL_DMA_IsActiveFlag_TC5(DMA1)){ LL_DMA_ClearFlag_TC5(DMA1); }
}

void DMA1_Stream6_IRQHandler(void){
    /* Find handle using this TX stream (simple 0..2 scan) */
    struct uart_handle_s* h = 0;
    for(int i=0;i<3;i++){
        if(handles[i].map && handles[i].map->usart == USART2){ h = &handles[i]; break; }
    }
    if(!h) return;

    if(LL_DMA_IsActiveFlag_TC6(DMA1)){
        LL_DMA_ClearFlag_TC6(DMA1);
        os_enter_critical();
        /* Advance tail by active len */
        h->tx_tail = (uint16_t)((h->tx_tail + h->tx_active_len) & h->tx_mask);
        uint16_t sent = h->tx_active_len;
        h->tx_active_len = 0;
        h->tx_busy = 0;
        /* Complete requests covered by 'sent' */
        tx_req_consume(h, sent);
        /* Start next chunk if any */
        tx_dma_start_locked(h);
        os_exit_critical();
    }
}


status_t uart_flush(uart_handle_t* h_, uint32_t timeout_ms){
    struct uart_handle_s* h = (struct uart_handle_s*)h_;
    if(!h) return ERR_ERR(MODULE_ID, 0x0020);
    uint32_t start = clock_millis();
    for(;;){
        os_enter_critical();
        /* If idle but data pending and segment was small, force start now */
        if(!h->tx_busy){ tx_dma_start_locked(h); }
        uint16_t cnt = (uint16_t)((h->tx_head + TX_RING_SIZE - h->tx_tail) & h->tx_mask);
        uint8_t busy = h->tx_busy;
        os_exit_critical();
        if(cnt==0 && !busy) return ERR_OK;
        if(timeout_ms==0) return ERR_ERR(MODULE_ID, 0x0021);
        if((clock_millis() - start) > timeout_ms) return ERR_ERR(MODULE_ID, 0x0022);
    }
}


status_t uart_set_tx_coalesce_min(uart_handle_t* h_, uint16_t bytes){
    struct uart_handle_s* h = (struct uart_handle_s*)h_;
    if(!h) return ERR_ERR(MODULE_ID, 0x0023);
    h->tx_coalesce_min = bytes;
    return ERR_OK;
}


status_t uart_try_flush(uart_handle_t* h_){
    struct uart_handle_s* h = (struct uart_handle_s*)h_;
    if(!h) return ERR_ERR(MODULE_ID, 0x0024);
    os_enter_critical();
    if(!h->tx_busy){ tx_dma_start_locked(h); }
    os_exit_critical();
    return ERR_OK;
}
