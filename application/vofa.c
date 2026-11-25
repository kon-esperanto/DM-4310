/**
  * @file    vofa.c
  * @brief   VOFA+ 波形输出(默认走 USART1)
  */

#include "vofa.h"

#if DM_VOFA_ENABLE

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "main.h"
#include "usart.h"              /* huart1 = USART1 */
#include "dm_motor_ctrl.h"      /* motor[], dm_log_enable */
#include "motor.h"
#include "remote_control.h"     /* GetDt7RcCh(), RC_CH_LEFT_VERTICAL */

#if VOFA_USE_USB_CDC
/* 需要工程里有 USB CDC 才会链接成功(当前没有, 见 vofa.h 的说明) */
extern uint8_t CDC_Transmit_HS(uint8_t *buf, uint16_t len);
#endif

#if VOFA_SILENCE_DM_LOG
extern uint8_t dm_log_enable;
#endif

/* 把要看的量读进数组, 顺序见 vofa.h 的 VOFA_CH_xxx */
static void vofa_read(float *v)
{
    /* 只读取一次位置, 确保弧度和角度来自同一帧反馈。 */
    const float pos_rad = motor[Motor1].para.pos;
    const float angle_deg = pos_rad * 57.295779513f;
    float angle_single = fmodf(angle_deg, 360.0f);
    if (angle_single < 0.0f)
        angle_single += 360.0f;
    if (angle_single >= 360.0f)
        angle_single = 0.0f;

    v[VOFA_CH_VEL_SET] = motor[Motor1].ctrl.vel_set;   /* 下发目标(爬坡后) */
    v[VOFA_CH_VEL]     = motor[Motor1].para.vel;       /* 实际速度 */
    v[VOFA_CH_POS]     = pos_rad;                     /* 反馈位置(rad) */
    v[VOFA_CH_TOR]     = motor[Motor1].para.tor;       /* 力矩 */
    v[VOFA_CH_RC]      = GetDt7RcCh(RC_CH_LEFT_VERTICAL);
    v[VOFA_CH_UPTIME]  = (float)(HAL_GetTick() / 1000u);   /* 心跳: 一直往上爬 */
    v[VOFA_CH_ANGLE_DEG] = angle_deg;
    v[VOFA_CH_ANGLE_SINGLE] = angle_single;
}

static void vofa_write(const uint8_t *buf, uint16_t len)
{
#if VOFA_USE_USB_CDC
    (void)CDC_Transmit_HS((uint8_t *)buf, len);
#else
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 20);
#endif
}

void vofa_init(void)
{
#if VOFA_SILENCE_DM_LOG
    /* dm_log() 也往 USART1 发文本, 混进波形数据里 VOFA+ 会画出毛刺, 直接关掉 */
    dm_log_enable = 0;
#endif
}

void vofa_send(void)
{
    static uint32_t tick;
    float v[VOFA_CH_NUM];

    if ((HAL_GetTick() - tick) < VOFA_PERIOD_MS)
    {
        return;
    }
    tick = HAL_GetTick();

    vofa_read(v);

#if (VOFA_PROTOCOL == VOFA_PROTOCOL_JUSTFLOAT)
    {
        /* JustFloat: N个float(小端) 后面跟 00 00 80 7F 作为帧尾 */
        uint8_t buf[VOFA_CH_NUM * 4 + 4];

        memcpy(buf, v, VOFA_CH_NUM * 4);
        buf[VOFA_CH_NUM * 4 + 0] = 0x00;
        buf[VOFA_CH_NUM * 4 + 1] = 0x00;
        buf[VOFA_CH_NUM * 4 + 2] = 0x80;
        buf[VOFA_CH_NUM * 4 + 3] = 0x7F;

        vofa_write(buf, (uint16_t)sizeof(buf));
    }
#else
    {
        /* FireWater: 一行就是一组数据, 逗号分隔, 换行结束 */
        char buf[128];
        int  n = snprintf(buf, sizeof(buf), "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                          (double)v[0], (double)v[1], (double)v[2],
                          (double)v[3], (double)v[4], (double)v[5],
                          (double)v[6], (double)v[7]);

        if (n > 0)
        {
            if (n > (int)sizeof(buf) - 1)
            {
                n = (int)sizeof(buf) - 1;
            }
            vofa_write((const uint8_t *)buf, (uint16_t)n);
        }
    }
#endif
}

#endif /* DM_VOFA_ENABLE */

/*------------------------------ End of File ------------------------------*/
