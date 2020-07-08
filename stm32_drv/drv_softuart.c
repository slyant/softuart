/*
 * Change Logs:
 * Date           Author       Notes
 * 2020-07-05     Slyant       the first version
 */
#include <rtdevice.h>
#include "string.h"
#include "stdlib.h"
#include "drv_common.h"
#include "board.h"
#include "drv_softuart.h"

#ifdef PKG_USING_SOFTUART
#include "softuart.h"

#ifdef PKG_USING_SOFTUART_STM32_DRV

//#define DRV_DEBUG
#define LOG_TAG              "drv.softuart"
#include <drv_log.h>

struct softuart_hw_config
{
    const char *name;
    const char *tx_pin_name;
    const char *rx_pin_name;
};

struct drv_softuart
{
    struct softuart_hw_config *config;
    struct softuart_device serial;
};

static struct softuart_hw_config softuart_config[] =
{
#ifdef BSP_USING_SOFTUART1
        SOFTUART1_CONFIG,
#endif
#ifdef BSP_USING_SOFTUART2
        SOFTUART2_CONFIG,
#endif
#ifdef BSP_USING_SOFTUART3
        SOFTUART3_CONFIG,
#endif
#ifdef BSP_USING_SOFTUART4
        SOFTUART4_CONFIG,
#endif
};

static int up_char(char * c)
{
    if ((*c >= 'a') && (*c <= 'z'))
    {
        *c = *c - 32;
    }
    return 0;
}
static rt_base_t get_pin_by_name(const char* pin_name)
{
    char port_name = pin_name[1];
    up_char(&port_name);
    int pin_num = atoi((char*) &pin_name[2]);
    int name_num = port_name - 'A';
    return (name_num * 16 + pin_num);
}

static void bit_usdelay(struct softuart_configure *config)
{
    rt_hw_us_delay(1000000 / config->baud_rate);
}

static void tx_send_byte(struct drv_softuart *uart, char byte)
{
    rt_uint8_t i = 8;
    rt_pin_write(get_pin_by_name(uart->config->tx_pin_name), 0);   //start bit
    bit_usdelay(&uart->serial.config);
    while(i--)
    {
        rt_pin_write(get_pin_by_name(uart->config->tx_pin_name), byte & 0x01);
        bit_usdelay(&uart->serial.config);
        byte = byte >> 1;
    }
    rt_pin_write(get_pin_by_name(uart->config->tx_pin_name), 1);   //stop bit
    bit_usdelay(&uart->serial.config);
}

static rt_uint8_t rx_read_byte(struct drv_softuart *uart)
{
    rt_uint8_t i = 8;
    rt_uint8_t read_byte = 0;
    bit_usdelay(&uart->serial.config);  //start bit
    while(i--)
    {
        read_byte >>= 1;
        if(rt_pin_read(get_pin_by_name(uart->config->rx_pin_name)) == 1) read_byte |= 0x80;
        bit_usdelay(&uart->serial.config);
    }

    return read_byte;
}

static void rx_pin_irq(void *args)
{
    struct drv_softuart *uart = (struct drv_softuart *)args;
    if(rt_pin_read(get_pin_by_name(uart->config->rx_pin_name)) == 0)
    {
        rt_pin_irq_enable(get_pin_by_name(uart->config->rx_pin_name), PIN_IRQ_DISABLE);
        softuart_isr(&uart->serial, RT_SERIAL_EVENT_RX_IND);
        rt_pin_irq_enable(get_pin_by_name(uart->config->rx_pin_name), PIN_IRQ_ENABLE);
    }
}

static rt_err_t stm32_gpio_configure(struct drv_softuart *uart)
{
    struct softuart_hw_config *config = uart->config;

    rt_pin_mode(get_pin_by_name(config->tx_pin_name), PIN_MODE_OUTPUT);
    rt_pin_write(get_pin_by_name(config->tx_pin_name), 1);

    rt_pin_mode(get_pin_by_name(config->rx_pin_name), PIN_MODE_INPUT_PULLUP);
    rt_pin_write(get_pin_by_name(config->rx_pin_name), 1);

    return RT_EOK;
}

static struct drv_softuart softuart_obj[sizeof(softuart_config) / sizeof(softuart_config[0])] = {0};

static rt_err_t stm32_configure(softuart_device_t serial, struct softuart_configure *cfg)
{
    struct drv_softuart *uart;
    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    uart = rt_container_of(serial, struct drv_softuart, serial);
    stm32_gpio_configure(uart);

    return RT_EOK;
}

static rt_err_t stm32_control(softuart_device_t serial, int cmd, void *arg)
{
    struct drv_softuart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = rt_container_of(serial, struct drv_softuart, serial);

    switch (cmd)
    {
    /* disable interrupt */
    case RT_DEVICE_CTRL_CLR_INT:
        rt_pin_irq_enable(get_pin_by_name(uart->config->rx_pin_name), PIN_IRQ_DISABLE);
        rt_pin_detach_irq(get_pin_by_name(uart->config->rx_pin_name));
        break;
    /* enable interrupt */
    case RT_DEVICE_CTRL_SET_INT:
        rt_pin_attach_irq(get_pin_by_name(uart->config->rx_pin_name), PIN_IRQ_MODE_FALLING, rx_pin_irq, uart);
        rt_pin_irq_enable(get_pin_by_name(uart->config->rx_pin_name), PIN_IRQ_ENABLE);
        break;
    }
    return RT_EOK;
}

static int stm32_putc(softuart_device_t serial, char c)
{
    struct drv_softuart *uart;
    RT_ASSERT(serial != RT_NULL);

    uart = rt_container_of(serial, struct drv_softuart, serial);
    tx_send_byte(uart, c);

    return 1;
}

static int stm32_getc(softuart_device_t serial)
{
    int ch;
    struct drv_softuart *uart;
    RT_ASSERT(serial != RT_NULL);
    uart = rt_container_of(serial, struct drv_softuart, serial);
    ch = rx_read_byte(uart);

    return ch;
}

static const struct softuart_ops _softuart_ops =
{
    .configure = stm32_configure,
    .control = stm32_control,
    .putc = stm32_putc,
    .getc = stm32_getc
};

int softusart_init(void)
{
    rt_size_t obj_num = sizeof(softuart_obj) / sizeof(struct drv_softuart);
    struct softuart_configure config = SOFTUART_CONFIG_DEFAULT;
    rt_err_t result = 0;

    for (int i = 0; i < obj_num; i++)
    {
        softuart_obj[i].config = &softuart_config[i];
        softuart_obj[i].serial.ops    = &_softuart_ops;
        softuart_obj[i].serial.config = config;
        /* register UART device */
        result = softuart_register(&softuart_obj[i].serial, softuart_obj[i].config->name,
                                       RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                                       NULL);
        RT_ASSERT(result == RT_EOK);
    }

    return result;
}
INIT_BOARD_EXPORT(softusart_init);
#endif
#endif
