/*
 * Change Logs:
 * Date           Author       Notes
 * 2020-07-05     Slyant       the first version
 */
#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <softuart.h>

#define DBG_TAG    "SUART"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>

rt_inline int _softuart_poll_tx(softuart_device_t serial, const rt_uint8_t *data, int length)
{
    int size;
    RT_ASSERT(serial != RT_NULL);

    size = length;
    while (length)
    {
        serial->ops->putc(serial, *data);

        ++data;
        --length;
    }

    return size - length;
}

/* RT-Thread Device Interface */
/*
 * This function initializes serial device.
 */
static rt_err_t softuart_init(struct rt_device *dev)
{
    rt_err_t result = RT_EOK;
    softuart_device_t serial;

    RT_ASSERT(dev != RT_NULL);
    serial = (softuart_device_t) dev;

    /* initialize rx/tx */
    serial->serial_rx = RT_NULL;

    /* apply configuration */
    if (serial->ops->configure)
        result = serial->ops->configure(serial, &serial->config);

    return result;
}

static rt_err_t softuart_open(struct rt_device *dev, rt_uint16_t oflag)
{
    softuart_device_t serial;

    RT_ASSERT(dev != RT_NULL);
    serial = (softuart_device_t) dev;

    /* initialize the Rx/Tx structure according to open flag */
    if (serial->serial_rx == RT_NULL)
    {
        struct softuart_rx_fifo* rx_fifo;

        rx_fifo = (struct softuart_rx_fifo*) rt_malloc(sizeof(struct softuart_rx_fifo) + serial->config.bufsz);
        if (rx_fifo == RT_NULL)
            return -RT_ENOMEM;

        rx_fifo->buffer = (rt_uint8_t*) (rx_fifo + 1);
        rt_memset(rx_fifo->buffer, 0, serial->config.bufsz);
        rx_fifo->put_index = 0;
        rx_fifo->get_index = 0;
        rx_fifo->is_full = RT_FALSE;

        serial->serial_rx = rx_fifo;
        /* configure low level device */
        serial->ops->control(serial, RT_DEVICE_CTRL_SET_INT, 0);
    }

    return RT_EOK;
}

static rt_err_t softuart_close(struct rt_device *dev)
{
    softuart_device_t serial;

    RT_ASSERT(dev != RT_NULL);
    serial = (softuart_device_t) dev;

    /* this device has more reference count */
    if (dev->ref_count > 1)
        return RT_EOK;

    struct softuart_rx_fifo* rx_fifo;

    rx_fifo = (struct softuart_rx_fifo*) serial->serial_rx;
    if (rx_fifo == RT_NULL)
        return -RT_EEMPTY;

    rt_free(rx_fifo);
    serial->serial_rx = RT_NULL;
    /* configure low level device */
    serial->ops->control(serial, RT_DEVICE_CTRL_CLR_INT, 0);

    return RT_EOK;
}

static rt_size_t softuart_read(struct rt_device *dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    softuart_device_t serial;

    RT_ASSERT(dev != RT_NULL);
    if (size == 0)
        return 0;

    serial = (softuart_device_t) dev;

    rt_base_t level;
    struct softuart_rx_fifo* rx_fifo;

    /* interrupt mode receive */
    rx_fifo = (struct softuart_rx_fifo*) serial->serial_rx;
    if (rx_fifo == RT_NULL)
        return -RT_EEMPTY;

    rt_size_t rx_length, count;
    /* get rx length */
    level = rt_hw_interrupt_disable();
    if (rx_fifo->put_index == rx_fifo->get_index)
    {
        count = 0;
    }
    else if (rx_fifo->get_index < rx_fifo->put_index)
    {
        rx_length = rx_fifo->put_index - rx_fifo->get_index;
        count = size > rx_length ? rx_length : size;
        rt_memcpy(buffer, &rx_fifo->buffer[rx_fifo->get_index], count);
        rx_fifo->get_index += count;
    }
    else
    {
        rx_length = (serial->config.bufsz - (rx_fifo->get_index - rx_fifo->put_index));
        count = size > rx_length ? rx_length : size;
        if (serial->config.bufsz - rx_fifo->get_index >= count)
        {
            rt_memcpy(buffer, &rx_fifo->buffer[rx_fifo->get_index], count);
        }
        else
        {
            rt_memcpy(buffer, &rx_fifo->buffer[rx_fifo->get_index], serial->config.bufsz - rx_fifo->get_index);
            rt_memcpy(buffer + (serial->config.bufsz - rx_fifo->get_index), &rx_fifo->buffer[0],
                    count - (serial->config.bufsz - rx_fifo->get_index));
        }
        rx_fifo->get_index = (count - (serial->config.bufsz - rx_fifo->get_index));
    }
    rt_hw_interrupt_enable(level);

    return count;
}

static rt_size_t softuart_write(struct rt_device *dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    softuart_device_t serial;

    RT_ASSERT(dev != RT_NULL);
    if (size == 0)
        return 0;

    serial = (softuart_device_t) dev;

    return _softuart_poll_tx(serial, (const rt_uint8_t *) buffer, size);
}

static rt_err_t softuart_control(struct rt_device *dev, int cmd, void *args)
{
    rt_err_t ret = RT_EOK;
    softuart_device_t serial;

    RT_ASSERT(dev != RT_NULL);
    serial = (softuart_device_t) dev;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CONFIG:
        if (args)
        {
            struct softuart_configure *pconfig = (struct softuart_configure *) args;
            if (pconfig->bufsz != serial->config.bufsz && serial->parent.ref_count)
            {
                /*can not change buffer size*/
                return RT_EBUSY;
            }
            /* set serial configure */
            serial->config = *pconfig;
            if (serial->parent.ref_count)
            {
                /* serial device has been opened, to configure it */
                serial->ops->configure(serial, (struct softuart_configure *) args);
            }
        }
        break;
    case RT_DEVICE_CTRL_SET_INT:
        serial->ops->control(serial, RT_DEVICE_CTRL_SET_INT, 0);
        break;
    case RT_DEVICE_CTRL_CLR_INT:
        serial->ops->control(serial, RT_DEVICE_CTRL_CLR_INT, 0);
        break;
    default:
        break;
    }

    return ret;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops serial_ops =
{
    softuart_init,
    softuart_open,
    softuart_close,
    softuart_read,
    softuart_write,
    softuart_control
};
#endif

/*
 * serial register
 */
rt_err_t softuart_register(softuart_device_t serial, const char *name, rt_uint32_t flag, void *data)
{
    rt_err_t ret;
    struct rt_device *device;
    RT_ASSERT(serial != RT_NULL);

    device = &(serial->parent);

    device->type = RT_Device_Class_Char;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;

#ifdef RT_USING_DEVICE_OPS
    device->ops = &serial_ops;
#else
    device->init = softuart_init;
    device->open = softuart_open;
    device->close = softuart_close;
    device->read = softuart_read;
    device->write = softuart_write;
    device->control = softuart_control;
#endif
    device->user_data = data;

    /* register a character device */
    ret = rt_device_register(device, name, flag);

    return ret;
}

/* ISR for serial interrupt */
void softuart_isr(softuart_device_t serial, int event)
{
    switch (event & 0xff)
    {
    case RT_SERIAL_EVENT_RX_IND:
    {
        int ch = -1;
        rt_base_t level;
        struct softuart_rx_fifo* rx_fifo;

        /* interrupt mode receive */
        rx_fifo = (struct softuart_rx_fifo*) serial->serial_rx;
        if (rx_fifo == RT_NULL)
            return;

        ch = serial->ops->getc(serial);

        /* disable interrupt */
        level = rt_hw_interrupt_disable();

        rx_fifo->buffer[rx_fifo->put_index] = ch;
        rx_fifo->put_index += 1;
        if (rx_fifo->put_index >= serial->config.bufsz)
            rx_fifo->put_index = 0;

        /* if the next position is read index, discard this 'read char' */
        if (rx_fifo->put_index == rx_fifo->get_index)
        {
            rx_fifo->get_index += 1;
            rx_fifo->is_full = RT_TRUE;
            if (rx_fifo->get_index >= serial->config.bufsz)
                rx_fifo->get_index = 0;
        }

        /* enable interrupt */
        rt_hw_interrupt_enable(level);

        /* invoke callback */
        if (serial->parent.rx_indicate != RT_NULL)
        {
            rt_size_t rx_length;

            /* get rx length */
            level = rt_hw_interrupt_disable();
            rx_length =
                    (rx_fifo->put_index >= rx_fifo->get_index) ?
                            (rx_fifo->put_index - rx_fifo->get_index) :
                            (serial->config.bufsz - (rx_fifo->get_index - rx_fifo->put_index));
            rt_hw_interrupt_enable(level);

            if (rx_length)
            {
                serial->parent.rx_indicate(&serial->parent, rx_length);
            }
        }
        break;
    }
    }
}
