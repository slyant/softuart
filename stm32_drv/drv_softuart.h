/*
 * Change Logs:
 * Date           Author       Notes
 * 2020-07-05     Slyant       the first version
 */
#ifndef DRIVERS_DRV_SOFTUART_H_
#define DRIVERS_DRV_SOFTUART_H_

#include <rtthread.h>
#include <board.h>

#if defined(BSP_USING_SOFTUART1)
#ifndef SOFTUART1_CONFIG
#define SOFTUART1_CONFIG                                             \
    {                                                                \
        .name = "suart1",                                            \
        .tx_pin_name = BSP_SOFTUART1_TX_PIN,                         \
        .rx_pin_name = BSP_SOFTUART1_RX_PIN,                         \
    }
#endif /* SOFTUART1_CONFIG */
#endif
#if defined(BSP_USING_SOFTUART2)
#ifndef SOFTUART2_CONFIG
#define SOFTUART2_CONFIG                                             \
    {                                                                \
        .name = "suart2",                                            \
        .tx_pin_name = BSP_SOFTUART2_TX_PIN,                         \
        .rx_pin_name = BSP_SOFTUART2_RX_PIN,                         \
    }
#endif /* SOFTUART2_CONFIG */
#endif
#if defined(BSP_USING_SOFTUART3)
#ifndef SOFTUART3_CONFIG
#define SOFTUART3_CONFIG                                             \
    {                                                                \
        .name = "suart3",                                            \
        .tx_pin_name = BSP_SOFTUART3_TX_PIN,                         \
        .rx_pin_name = BSP_SOFTUART3_RX_PIN,                         \
    }
#endif /* SOFTUART3_CONFIG */
#endif
#if defined(BSP_USING_SOFTUART4)
#ifndef SOFTUART4_CONFIG
#define SOFTUART4_CONFIG                                             \
    {                                                                \
        .name = "suart4",                                            \
        .tx_pin_name = BSP_SOFTUART4_TX_PIN,                         \
        .rx_pin_name = BSP_SOFTUART4_RX_PIN,                         \
    }
#endif /* SOFTUART4_CONFIG */
#endif

#endif /* DRIVERS_DRV_SOFTUART_H_ */
