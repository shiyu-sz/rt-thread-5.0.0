/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <tusb.h>
#include <usb_descriptor.h>
#include <math.h>

/* defined the LED0 pin: PB1 */
#define LED0_PIN    GET_PIN(B, 1)

void cdc_example(void)
{
    uint8_t buffer[32];
    char ch;

    /* 判断端口是否打开 (DTR=1) */
    if (tud_cdc_connected())
    {
        rt_memset(buffer, 0, 32);

        /* 发送字符串 */
        tud_cdc_write_str("please enter something: ");
        /* 刷新发送缓冲区 */
        tud_cdc_write_flush();
        /* 清空读取缓冲区 */
        tud_cdc_read_flush();

        for (int i = 0; i < 32; i++)
        {
            /* 判断当前是否有数据可读取 */
            while (!tud_cdc_available()) { rt_thread_mdelay(10); }
            /* 读取一个字符 */
            ch = tud_cdc_read_char();
            *(buffer + i) = ch;
            /* 字符回显 */
            tud_cdc_write_char(ch);
            tud_cdc_write_flush();
            /* 按下回车结束输入 */
            if (ch == '\r') { break; }
        }
        tud_cdc_write_str("\r\nwhat you enter: ");
        tud_cdc_write((const char *) buffer, 32);
        tud_cdc_write_str("\r\n");
        tud_cdc_write_flush();
    }
    else
    {
        rt_kprintf("please open port and make sure DTR=1\n");
    }
}
MSH_CMD_EXPORT(cdc_example, TinyUSB cdc example)

#define CIRCLE_RADIUS 300

/* 字符键值转换表 */
static const uint8_t conv_table[128][2] =  { HID_ASCII_TO_KEYCODE };
static struct rt_semaphore hid_sem;

/* 报文发送完毕回调函数 */
void tud_hid_report_complete_cb(uint8_t instance, uint8_t const* report, uint8_t len)
{
    (void) instance;
    (void) report;
    (void) len;
    /* 使用信号量进行同步，确保报文不会因发送过快导致还未发送的报文被覆盖 */
    rt_sem_release(&hid_sem);
}

static void hid_press_key(uint8_t modifier, uint8_t keycode)
{
    uint8_t keycode_array[6] = {0};

    keycode_array[0] = keycode;
    tud_hid_keyboard_report(REPORT_ID_KEYBOARD, modifier, keycode_array);
    rt_sem_take(&hid_sem, 100);
    keycode_array[0] = 0;
    tud_hid_keyboard_report(REPORT_ID_KEYBOARD, 0, keycode_array);
    rt_sem_take(&hid_sem, 100);
}

static void hid_type_str_fmt(const char *fmt, ...)
{
    uint8_t  keycode;
    uint8_t  modifier;
    uint32_t length;
    va_list  args;
    char     buffer[64];
    char     chr;

    va_start(args, fmt);
    rt_vsnprintf(buffer, 64, fmt, args);

    length = rt_strlen(buffer);
    for (int i = 0; i < length; i++)
    {
        chr = buffer[i];
        if (conv_table[chr][0])
        {
            modifier = KEYBOARD_MODIFIER_LEFTSHIFT;
        }
        else
        {
            modifier = 0;
        }
        keycode = conv_table[chr][1];
        hid_press_key(modifier, keycode);
    }
}

void hid_example(void)
{
    float last_x_pos = CIRCLE_RADIUS, x_pos;
    float last_y_pos = 0, y_pos;

    /* 初始化信号量 */
    rt_sem_init(&hid_sem, "hid", 0, RT_IPC_FLAG_PRIO);

    if (tud_connected())
    {
        /* 用鼠标画个圆 */
        for (int i = 0; i < 360; i++)
        {
            x_pos = cosf((float)i / 57.3F) * CIRCLE_RADIUS;
            y_pos = sinf((float)i / 57.3F) * CIRCLE_RADIUS;

            tud_hid_mouse_report(REPORT_ID_MOUSE, 0,
                                 (int8_t)(x_pos - last_x_pos), (int8_t)(y_pos - last_y_pos), 0, 0);
            rt_sem_take(&hid_sem, 100);

            last_x_pos = x_pos;
            last_y_pos = y_pos;
        }
        rt_thread_mdelay(500);

        /* 打开记事本并输入一些文字 */
        hid_press_key(KEYBOARD_MODIFIER_LEFTGUI, HID_KEY_R);
        rt_thread_mdelay(500);
        hid_type_str_fmt("notepad\n\n");
        rt_thread_mdelay(2000);
        hid_type_str_fmt("This is RT-Thread TinyUSB demo.\n");
    }
    else
    {
        rt_kprintf("please connect USB port\n");
    }

    rt_sem_detach(&hid_sem);
}
MSH_CMD_EXPORT(hid_example, TinyUSB hid example)

int main(void)
{
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);

    while (1)
    {
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }
}
