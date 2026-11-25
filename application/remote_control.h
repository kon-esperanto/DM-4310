/**
  ****************************(C) COPYRIGHT 2025 Polarbear****************************
  * @file       remote_control.h
  * @brief      遥控器处理 —— DT7/D-BUS 专用版
  *             原文件同时支持 DT7 和 AT9S PRO / HT8A / ET08A 这些普通SBUS遥控器,
  *             本工程只用 DT7, 所以把 SBUS 那一整套(25字节帧解析、中值换算)删掉了,
  *             以后要换 SBUS 遥控器时, 从原工程把 remote_control.c/h 重新拷一份即可。
  * @note       解析是串口空闲中断里做的, 不是freeRTOS任务
  * @history
  *  V1.0.0  Dec-26-2018  RM       1. done
  *  V1.0.0  Nov-11-2019  RM       1. support development board type c
  *  V2.0.0  Feb-17-2025  Penguin  1. support RC AT9S PRO / HT8A / normal SBUS RC
  *  V2.0.1  Feb-25-2025  Penguin  1. support RC ET08A
  *  V3.0.0  2026-09-18   本工程    1. 裁成 DT7-only, 并移植到 STM32H7 + UART5
  @verbatim
  ==============================================================================
  DT7(D-BUS)一帧 18 字节, 波特率 100000 / 8位数据+偶校验 / 2停止位(反相),
  接收口是板子上的 D-BUS/SBUS 座: UART5_RX = PD2, 底层驱动见 bsp_rc.c
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Polarbear****************************
  */

#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
// clang-format off
#include <stdbool.h>

#include "struct_typedef.h"
#include "attribute_typedef.h"

/* DMA接收缓冲区长度: DT7一帧只有18字节, 这里给36字节, 防止DMA来不及搬而越界 */
#define SBUS_RX_BUF_NUM 36u

/* DT7(D-BUS)一帧的长度(字节) */
#define RC_FRAME_LENGTH 18u

/* DT7遥控器通道值范围 */
#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

#define RC_TO_ONE 0.0015151515151515f  // (1/660)遥控器通道值归一化系数

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)

#define RC_CH_LEFT_HORIZONTAL   ((uint8_t)2)
#define RC_CH_LEFT_VERTICAL     ((uint8_t)3)
#define RC_CH_RIGHT_HORIZONTAL  ((uint8_t)0)
#define RC_CH_RIGHT_VERTICAL    ((uint8_t)1)
#define RC_CH_LEFT_ROTATE       ((uint8_t)4)

#define RC_SW_LEFT              ((uint8_t)1)
#define RC_SW_RIGHT             ((uint8_t)0)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_W     ((uint8_t)0)
#define KEY_S     ((uint8_t)1)
#define KEY_A     ((uint8_t)2)
#define KEY_D     ((uint8_t)3)
#define KEY_SHIFT ((uint8_t)4)
#define KEY_CTRL  ((uint8_t)5)
#define KEY_Q     ((uint8_t)6)
#define KEY_E     ((uint8_t)7)
#define KEY_R     ((uint8_t)8)
#define KEY_F     ((uint8_t)9)
#define KEY_G     ((uint8_t)10)
#define KEY_Z     ((uint8_t)11)
#define KEY_X     ((uint8_t)12)
#define KEY_C     ((uint8_t)13)
#define KEY_V     ((uint8_t)14)
#define KEY_B     ((uint8_t)15)

#define KEY_LEFT  ((uint8_t)0)
#define KEY_RIGHT ((uint8_t)1)

#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << KEY_W)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << KEY_S)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << KEY_A)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << KEY_D)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << KEY_SHIFT)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << KEY_CTRL)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << KEY_Q)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << KEY_E)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << KEY_R)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << KEY_F)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << KEY_G)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << KEY_Z)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << KEY_X)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << KEY_C)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << KEY_V)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << KEY_B)

/* ----------------------- 鼠标轴编号 -------------------------------------- */
#define AX_X    0
#define AX_Y    1
#define AX_Z    2

/* ----------------------- Data Struct ------------------------------------- */
typedef struct __RC_ctrl
{
        struct __rc
        {
                int16_t ch[5];
                char s[2];
        } __packed__ rc;
        struct __mouse
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } __packed__ mouse;
        struct __key
        {
                uint16_t v;
        } __packed__ key;

} __packed__ RC_ctrl_t;
// clang-format on

/* ----------------------- Internal Data ----------------------------------- */

/* 遥控器初始化: 打开UART5接收(DMA + 空闲中断), 在main()里调用一次 */
extern void remote_control_init(void);

/* 取遥控器数据指针(原始通道值, 没做归一化) */
extern const RC_ctrl_t * get_remote_control_point(void);

/* 遥控器数据是否出错: 0=正常, 1=出错(出错时会把rc_ctrl全部清零) */
extern uint8_t RC_data_is_error(void);

/* 遥控掉线 / 数据出错时调用, 把DMA接收重新武装一遍 */
extern void slove_RC_lost(void);
extern void slove_data_error(void);

/******************************************************************/
/* API                                                            */
/******************************************************************/

/* true = 遥控器离线(没插/没对频/丢包太久)。遥控控电机时一定要先判这个 */
extern bool GetRcOffline(void);

/* DT7通道值, 范围 [-1, 1]。ch 用 RC_CH_xxx 宏, 比如 RC_CH_LEFT_VERTICAL */
extern float GetDt7RcCh(uint8_t ch);
/* DT7拨杆值, 范围 {1,2,3}, 配合 switch_is_xxx 宏使用 */
extern char GetDt7RcSw(uint8_t sw);
extern float GetDt7MouseSpeed(uint8_t axis);
extern bool GetDt7Mouse(uint8_t key);
extern bool GetDt7Keyboard(uint8_t key);

#endif
/*------------------------------ End of File ------------------------------*/
