/**
  * @file    vofa.h
  * @brief   把电机数据发到 VOFA+ 看波形(走 USART1)
  *
  *  为什么走串口而不是板子的 USB-C 口:
  *    板上那个 USB 口是直接接在 MCU 的 PA11/PA12 上的(原理图标注 USB_DM/DP),
  *    要让电脑认出 COM 口, 固件里必须实现 USB CDC 设备。本工程原来的那一套
  *    (USB_DEVICE/ + Middlewares/) 被 CubeMX 重新生成时删掉了, 现在没有。
  *    USART1 有独立的 4pin 座, 接一个 USB-TTL 模块就能用, 最快最稳。
  *
  *  接线(三根线, 板子自己供电时不要接 5V):
  *      USB-TTL GND  <->  板上 USART1 座的 GND
  *      USB-TTL RX   <->  板上 USART1 座的 TX (PA9)
  *
  *  VOFA+ 里的设置:
  *      数据格式 = FireWater   (和下面的 VOFA_PROTOCOL 对应)
  *      串口     = 你的 USB-TTL 那个 COM 口
  *      波特率   = 115200      (和 USART1 一致)
  *      然后加一个"波形图"控件, 通道 0..7(共8路) 对应下面的通道定义
  */
#ifndef VOFA_H
#define VOFA_H

#include <stdint.h>

/* 波形输出总开关: 0=关(想看重定向到USART1的dm_log文本时把它关掉) */
#define DM_VOFA_ENABLE          1

/* 输出到哪里:
     0 = USART1(板上4pin的USART1座, TX=PA9, 用USB-TTL接电脑) —— 现在只能选这个
     1 = USB虚拟串口(板子的USB-C口)。需要工程里有USB CDC(CDC_Transmit_HS),
         当前没有, 选1会链接报错, 那是故意的: 提醒你先把USB CDC加回来 */
#define VOFA_USE_USB_CDC        0

/* 协议: FireWater = 文本 "1.23,4.56\n", 直观、能直接用串口助手看
        JustFloat = 原始float + 00 00 80 7F 结尾, 带宽小、适合高采样率 */
#define VOFA_PROTOCOL_FIREWATER 0
#define VOFA_PROTOCOL_JUSTFLOAT 1
#define VOFA_PROTOCOL           VOFA_PROTOCOL_FIREWATER

/* 采样间隔(ms)。1kHz的控制环不是每一拍都发, 按这个降采样。
   115200 波特率下8路文本通常约80字节(≈7ms), 默认每20ms发一帧;
   想更快就把这里调小, 同时把 usart.c 里 USART1 的波特率一起提高 */
#define VOFA_PERIOD_MS          20

/* 是否在波形期间关掉 dm_log() 的文本打印。
   dm_log 和波形共用 USART1, 文本行混进波形里会让 VOFA+ 画出毛刺, 所以默认关掉;
   想重新看诊断日志: 把这里改成 0, 或者把 DM_VOFA_ENABLE 改成 0 */
#define VOFA_SILENCE_DM_LOG     1

/* 通道顺序: VOFA+ 波形图里通道 0..7 就是下面这个顺序 */
#define VOFA_CH_VEL_SET   0   /* 下发的目标速度(爬坡后的值), rad/s */
#define VOFA_CH_VEL       1   /* 电机反馈的实际速度, rad/s */
#define VOFA_CH_POS       2   /* 位置, rad */
#define VOFA_CH_TOR       3   /* 力矩, N·m */
#define VOFA_CH_RC        4   /* 遥控左摇杆竖直通道, -1 ~ 1 */
#define VOFA_CH_UPTIME    5   /* 开机至今的秒数, 一直在涨。
                                 用途: 电机没接/遥控离线时前面几个通道全是0, VOFA+里就是一条
                                 贴地的直线, 分不清"没数据"还是"数据都是0"。看这一路有没有在爬
                                 就知道链路通没通了 */
#define VOFA_CH_ANGLE_DEG    6 /* 反馈位置换算成角度, deg, 保留正负号 */
#define VOFA_CH_ANGLE_SINGLE 7 /* 反馈角度对360取模, [0,360), deg */
#define VOFA_CH_NUM          8

/* 位置相对于电机已设置的零点, 不是上电位置; 不会发送设置零点命令。
   通道6/7直接由通道2换算, 不是累计转角。若电机反馈在±PMAX处回绕,
   这三路都可能跳变; 通道7在跨越0/360度时也会正常跳变。
   尚未收到反馈时位置为0, 不能据此判断电机确实处于零点。 */

/* 初始化: 在 main() 里调一次(会把 dm_log 关掉, 见 VOFA_SILENCE_DM_LOG) */
void vofa_init(void);

/* 主循环里反复调用, 内部按 VOFA_PERIOD_MS 限速, 不阻塞太久 */
void vofa_send(void);

#endif /* VOFA_H */
