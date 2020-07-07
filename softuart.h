/*
 * Change Logs:
 * Date           Author       Notes
 * 2020-07-05     Slyant       the first version
 */
#ifndef SOFTUART_SOFTUART_H_
#define SOFTUART_SOFTUART_H_

#include <rtthread.h>

#define BAUD_RATE_2400                  2400
#define BAUD_RATE_4800                  4800
#define BAUD_RATE_9600                  9600
#define BAUD_RATE_19200                 19200

#define DATA_BITS_5                     5
#define DATA_BITS_6                     6
#define DATA_BITS_7                     7
#define DATA_BITS_8                     8
#define DATA_BITS_9                     9

#define STOP_BITS_1                     0
#define STOP_BITS_2                     1
#define STOP_BITS_3                     2
#define STOP_BITS_4                     3

#ifndef RT_SERIAL_RB_BUFSZ
#define RT_SERIAL_RB_BUFSZ              64
#endif

/* Default config for serial_configure structure */
#define SOFTUART_CONFIG_DEFAULT           \
{                                          \
    BAUD_RATE_9600,     /* 9600 bits/s */  \
    DATA_BITS_8,      /* 8 databits */     \
    STOP_BITS_1,      /* 1 stopbit */      \
    RT_SERIAL_RB_BUFSZ, /* Buffer size */  \
}

struct softuart_configure
{
    rt_uint32_t baud_rate;

    rt_uint32_t data_bits   :4;
    rt_uint32_t stop_bits   :2;
    rt_uint32_t bufsz       :16;
};

struct softuart_device
{
    struct rt_device            parent;

    const struct softuart_ops   *ops;
    struct softuart_configure   config;

    void *serial_rx;
};
typedef struct softuart_device *softuart_device_t;

struct softuart_ops
{
    rt_err_t (*configure)(softuart_device_t serial, struct softuart_configure *cfg);
    rt_err_t (*control)(softuart_device_t serial, int cmd, void *arg);

    int (*putc)(softuart_device_t serial, char c);
    int (*getc)(softuart_device_t serial);
};

struct softuart_rx_fifo
{
    /* software fifo */
    rt_uint8_t *buffer;

    rt_uint16_t put_index, get_index;

    rt_bool_t is_full;
};

void softuart_isr(softuart_device_t serial, int event);

rt_err_t softuart_register(softuart_device_t    serial,
                           const char           *name,
                           rt_uint32_t          flag,
                           void                 *data);

#endif /* SOFTUART_SOFTUART_H_ */
