/*
+------------------------------------------------------------------------------+
| IRISS Inc.
| -----------------------------------------------------------------------------
| Copyright 2023-2025 (c) IRISS Inc. All rights reserved.
+------------------------------------------------------------------------------+
*/
/**
 *  @file    slm_at_uart.c
 *  @brief   Async UART transport for the SLM AT command processor.
 *           Literal port of h745-zephyr-gateway/src/slm_at_uart.c.
 *
 *  RX: hardware double-buffered DMA into two 512-byte buffers. The UART
 *      callback copies each RX chunk into a 4 KB ring buffer. A delayable
 *      workqueue item drains the ring buffer into a single message slot
 *      that the AT processor can poll via slm_at_wait_for_message().
 *
 *  TX: writes go into a TX ring buffer guarded by a mutex. A semaphore
 *      tracks whether a uart_tx() is currently in flight; UART_TX_DONE
 *      either re-starts TX with whatever else is queued, or releases the
 *      semaphore.
 */
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(slm_at_uart, CONFIG_MAIN_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <string.h>
#include "slm_at_uart.h"

#define CONFIG_SLM_AT_ASYNC_UART_RX_ENABLE_TIMEOUT   100
#define CONFIG_SLM_AT_ASYNC_UART_RX_RECOVERY_TIMEOUT 1000
#define UART_SLM_AT_TX_BUF_LEN                       2048
#define UART_SLM_AT_BUF_LEN                          512
#define UART_SLM_AT_MSG_NEXT_BUF_TIMEOUT             200
#define UART_SLM_AT_RINGBUF_SIZE                     4096

struct slm_driver_context {
    const struct device *dev;
    uint8_t buf[UART_SLM_AT_BUF_LEN];
    uint8_t buf2[UART_SLM_AT_BUF_LEN];

    struct k_work_delayable uart_recovery_work;
    struct k_work_delayable uart_rx_work;

    struct ring_buf rx_ringbuf;
    uint8_t         rx_buf[UART_SLM_AT_RINGBUF_SIZE];

    struct ring_buf tx_ringbuf;
    uint8_t         tx_buf[UART_SLM_AT_TX_BUF_LEN];
};

static struct slm_driver_context slm_dev;

static bool rx_retry_pending;
static bool uart_recovery_pending;
static uint8_t *next_buf;

static data_msg_t g_data_msg;
static K_SEM_DEFINE(data_msg_sem, 0, 1);

static K_SEM_DEFINE(tx_done_sem, 0, 1);
static K_MUTEX_DEFINE(mutex_tx_put);

static int slm_at_tx_start(void)
{
    uint8_t *buf;
    size_t ret;
    int err;

    ret = ring_buf_get_claim(&slm_dev.tx_ringbuf, &buf,
                             ring_buf_capacity_get(&slm_dev.tx_ringbuf));
    err = uart_tx(slm_dev.dev, buf, ret, SYS_FOREVER_US);
    if (err) {
        LOG_ERR("UART TX error: %d", err);
        (void)ring_buf_get_finish(&slm_dev.tx_ringbuf, 0);
        return err;
    }
    return 0;
}

static void slm_at_uart_callback(const struct device *dev,
                                 struct uart_event *evt, void *user_data)
{
    struct slm_driver_context *context = user_data;
    uint8_t *p;
    int err, ret, len, space_left;

    switch (evt->type) {
    case UART_TX_DONE:
        err = ring_buf_get_finish(&slm_dev.tx_ringbuf, evt->data.tx.len);
        if (err) {
            LOG_ERR("UART_TX_DONE failure: %d", err);
        }
        if (ring_buf_is_empty(&slm_dev.tx_ringbuf) == false) {
            (void)slm_at_tx_start();
        } else {
            k_sem_give(&tx_done_sem);
        }
        break;

    case UART_TX_ABORTED:
        err = ring_buf_get_finish(&slm_dev.tx_ringbuf, evt->data.tx.len);
        if (err) {
            LOG_ERR("UART_TX_ABORTED failure: %d", err);
        }
        if (ring_buf_is_empty(&slm_dev.tx_ringbuf) == false) {
            (void)slm_at_tx_start();
        } else {
            k_sem_give(&tx_done_sem);
        }
        break;

    case UART_RX_RDY:
        len = evt->data.rx.len;
        p   = evt->data.rx.buf + evt->data.rx.offset;

        ret = ring_buf_put(&context->rx_ringbuf, p, len);
        if (ret < evt->data.rx.len) {
            LOG_WRN("Rx buffer doesn't have enough space. "
                    "Bytes pending: %d, written only: %d. "
                    "Disabling RX for now.",
                    evt->data.rx.len, ret);
            if (!rx_retry_pending) {
                uart_rx_disable(dev);
                rx_retry_pending = true;
            }
        }

        space_left = ring_buf_space_get(&context->rx_ringbuf);
        if (!rx_retry_pending &&
            space_left < (sizeof(context->rx_buf) / 8)) {
            uart_rx_disable(dev);
            rx_retry_pending = true;
            LOG_WRN("%d written to RX buf, but only %d space left. "
                    "Disabling RX for now.",
                    ret, space_left);
        }

        k_work_schedule(&context->uart_rx_work,
                        K_MSEC(UART_SLM_AT_MSG_NEXT_BUF_TIMEOUT));
        break;

    case UART_RX_BUF_REQUEST:
        if (next_buf) {
            err = uart_rx_buf_rsp(dev, next_buf, UART_SLM_AT_BUF_LEN);
            if (err) {
                LOG_ERR("uart_rx_buf_rsp() err: %d", err);
            }
        }
        break;

    case UART_RX_BUF_RELEASED:
        next_buf = evt->data.rx_buf.buf;
        break;

    case UART_RX_DISABLED:
        if (rx_retry_pending && !uart_recovery_pending) {
            k_work_schedule(&context->uart_recovery_work,
                            K_MSEC(CONFIG_SLM_AT_ASYNC_UART_RX_RECOVERY_TIMEOUT));
            rx_retry_pending      = false;
            uart_recovery_pending = true;
        }
        break;

    case UART_RX_STOPPED:
        if (evt->data.rx_stop.reason != 0) {
            rx_retry_pending = true;
        }
        break;

    default:
        break;
    }
}

static int slm_at_async_uart_rx_enable(struct slm_driver_context *context)
{
    int err;

    next_buf = context->buf2;

    err = uart_callback_set(context->dev, slm_at_uart_callback,
                            (void *)context);
    if (err) {
        LOG_ERR("Failed to set uart callback, err %d", err);
    }

    err = uart_rx_enable(context->dev, context->buf, UART_SLM_AT_BUF_LEN,
                         CONFIG_SLM_AT_ASYNC_UART_RX_ENABLE_TIMEOUT * USEC_PER_MSEC);
    if (err) {
        LOG_ERR("uart_rx_enable() failed, err %d", err);
    }
    rx_retry_pending = false;
    return err;
}

static void slm_at_uart_recovery(struct k_work *work)
{
    struct k_work_delayable   *dwork = k_work_delayable_from_work(work);
    struct slm_driver_context *slm   =
        CONTAINER_OF(dwork, struct slm_driver_context, uart_recovery_work);
    int ret;

    ret = ring_buf_space_get(&slm->rx_ringbuf);
    if (ret >= (sizeof(slm->rx_buf) / 2)) {
        ret = slm_at_async_uart_rx_enable(slm);
        if (ret) {
            LOG_ERR("slm_async_uart_rx_enable() failed, err %d", ret);
        } else {
            LOG_WRN("UART RX recovered");
        }
        uart_recovery_pending = false;
    } else {
        LOG_ERR("Rx buffer still doesn't have enough room %d to be re-enabled",
                ret);
        k_work_schedule(&slm->uart_recovery_work,
                        K_MSEC(CONFIG_SLM_AT_ASYNC_UART_RX_RECOVERY_TIMEOUT));
    }
}

static void slm_at_consume_ringbuf(struct k_work *work)
{
    struct k_work_delayable   *dwork = k_work_delayable_from_work(work);
    struct slm_driver_context *slm   =
        CONTAINER_OF(dwork, struct slm_driver_context, uart_rx_work);
    size_t len;

    if (k_sem_take(&data_msg_sem, K_SECONDS(1)) == 0) {
        if (!g_data_msg.message_ready) {
            len = ring_buf_get(&slm->rx_ringbuf, (uint8_t *)g_data_msg.buf, sizeof(g_data_msg.buf));
            if (len != 0) {
                g_data_msg.size = len;
                g_data_msg.message_ready = true;
            }
        }
        k_sem_give(&data_msg_sem);
    }

    if (!ring_buf_is_empty(&slm->rx_ringbuf)) {
        k_work_schedule(&slm->uart_rx_work, K_MSEC(SLM_UART_MIN_10_MS_DELAY_TIME));
    }
}

int slm_at_uart_init(void)
{
    slm_dev.dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_slm_uart));
    if (!device_is_ready(slm_dev.dev)) {
        LOG_ERR("SLM AT UART not ready");
        return -ENODEV;
    }
    LOG_INF("Initializing SLM AT to use %s", slm_dev.dev->name);

    memset(&g_data_msg, 0, sizeof(g_data_msg));

    ring_buf_init(&slm_dev.rx_ringbuf, sizeof(slm_dev.rx_buf), slm_dev.rx_buf);
    ring_buf_init(&slm_dev.tx_ringbuf, sizeof(slm_dev.tx_buf), slm_dev.tx_buf);

    k_work_init_delayable(&slm_dev.uart_rx_work,       slm_at_consume_ringbuf);
    k_work_init_delayable(&slm_dev.uart_recovery_work, slm_at_uart_recovery);

    slm_at_async_uart_rx_enable(&slm_dev);

    k_sem_give(&tx_done_sem);
    k_sem_give(&data_msg_sem);

    return 0;
}

int slm_at_tx_write(const uint8_t *data, size_t len, bool print_full_debug)
{
    ARG_UNUSED(print_full_debug);

    size_t ret;
    size_t sent = 0;
    int    err;

    k_mutex_lock(&mutex_tx_put, K_FOREVER);
    while (sent < len) {
        ret = ring_buf_put(&slm_dev.tx_ringbuf, data + sent, len - sent);
        if (ret) {
            sent += ret;
        } else {
            /* TX ring buf full — block until current TX drains, then push. */
            k_sem_take(&tx_done_sem, K_FOREVER);
            err = slm_at_tx_start();
            if (err) {
                LOG_ERR("TX buf overflow, %d dropped. Unable to send: %d",
                        len - sent, err);
                k_sem_give(&tx_done_sem);
                k_mutex_unlock(&mutex_tx_put);
                return err;
            }
        }
    }
    k_mutex_unlock(&mutex_tx_put);

    if (k_sem_take(&tx_done_sem, K_NO_WAIT) == 0) {
        err = slm_at_tx_start();
        if (err == 1) {
            k_sem_give(&tx_done_sem);
            return 0;
        } else if (err) {
            LOG_ERR("TX start failed: %d", err);
            k_sem_give(&tx_done_sem);
            return err;
        }
    }
    /* else: TX already in progress; UART_TX_DONE will drain the rest. */

    return 0;
}

bool slm_at_wait_for_message(uint32_t w_time)
{
    /* Fast path: caller wants a non-blocking peek. Never sleep. */
    if (w_time == 0) {
        return g_data_msg.message_ready;
    }

    uint32_t ticks = w_time / SLM_UART_MIN_10_MS_DELAY_TIME;
    for (uint32_t i = 0; i <= ticks; i++) {
        if (g_data_msg.message_ready) {
            return true;
        }
        k_msleep(SLM_UART_MIN_10_MS_DELAY_TIME);
    }
    return g_data_msg.message_ready;
}

void slm_at_get_message_copy(char *pcopy_str)
{
    size_t size = 0;
    if (k_sem_take(&data_msg_sem, K_SECONDS(1)) == 0) {
        size = g_data_msg.size;
        if (size > (SLM_UART_AT_COMMAND_LEN - 1)) {
            size = SLM_UART_AT_COMMAND_LEN - 1;
        }
        memcpy(pcopy_str, g_data_msg.buf, size);
        pcopy_str[size]          = 0x00;
        g_data_msg.message_ready = false;
        k_sem_give(&data_msg_sem);
    }
    pcopy_str[size] = 0x00;
}
