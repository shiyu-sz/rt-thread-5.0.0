/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-27     balanceTWK   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

int main(void)
{
    /* set LED0 pin mode to output */
    rt_pin_mode(GET_PIN(C, 13), PIN_MODE_OUTPUT);

    while (1)
    {
        rt_pin_write(GET_PIN(C, 13), PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(GET_PIN(C, 13), PIN_LOW);
        rt_thread_mdelay(500);
    }
}
