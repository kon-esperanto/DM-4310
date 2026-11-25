/**
  ****************************(C) COPYRIGHT 2025 Polarbear****************************
  * @file       remote_control.c
  * @brief      遥控器处理 —— DT7/D-BUS 专用版
  *             遥控器用类SBUS协议传输, 用DMA搬运省CPU, 用串口空闲中断拉起解析函数,
  *             并提供掉线重启DMA的接口, 保证热插拔的稳定性。
  * @note       解析是在串口中断里做的, 不是freeRTOS任务
  * @history
  *  V1.0.0  Dec-26-2018  RM       1. done
  *  V2.0.0  Feb-17-2025  Penguin  1. support RC AT9S PRO / HT8A / normal SBUS RC
  *  V3.0.0  2026-09-18   本工程    1. 裁成 DT7-only(删掉25字节SBUS那一套)
  *                                2. 从 USART3 移植到 UART5, 并适配 STM32H7
  * @verbatim
  ==============================================================================
  USART1 本来是用来把原始遥控帧转发给上位机看的, 本工程 USART1 已经是日志口,
  所以那部分(usart1_tx_dma_enable / sbus_to_usart1)一起删掉了。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Polarbear****************************
  */

// clang-format off
#include "remote_control.h"
#include "bsp_rc.h"

#include "main.h"

#include "detect_task.h"

// 遥控器掉线时间阈值
#define RC_LOST_TIME 100  // ms

//遥控器出错数据上限
#define RC_CHANNAL_ERROR_VALUE 700

/* 本板(达妙 DM-MC-Board02)的 D-BUS 接收口是 UART5(PD2), 底层驱动见 bsp_rc.c */
extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;

/* STM32H7 的 DMA_HandleTypeDef.Instance 是 void*, 取寄存器之前必须先转成 DMA_Stream_TypeDef* */
#define RC_DMA_STREAM   ((DMA_Stream_TypeDef *)(hdma_uart5_rx.Instance))

//取正函数
static int16_t RC_abs(int16_t value);

/**
  * @brief          DT7(D-BUS)协议解析
  * @param[in]      sbus_buf: 原生数据指针, 18字节
  * @param[out]     rc_ctrl: 遥控器数据指针
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

//remote control data
//遥控器控制变量
RC_ctrl_t rc_ctrl;

//接收原始数据，一帧18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

// 上一次接收数据的时间
static uint32_t last_receive_time = 0;
// 记录连续接收数据的次数
static uint32_t receive_count = 0;

/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

//判断遥控器数据是否出错
uint8_t RC_data_is_error(void)
{
    //使用了go to语句 方便出错统一处理遥控器变量数据归零
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }
    return 0;

error:
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    rc_ctrl.rc.s[0] = RC_SW_DOWN;
    rc_ctrl.rc.s[1] = RC_SW_DOWN;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.key.v = 0;
    return 1;
}

//遥控器掉线, 重新武装DMA
void slove_RC_lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}

//遥控器数据出错, 重新武装DMA
void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}

// clang-format on
// 记录接收数据的次数
#define COUNT_RECEIVED                            \
    if (now - last_receive_time > RC_LOST_TIME) { \
        receive_count = 0;                        \
    }                                             \
    receive_count++;
// clang-format off

/* 串口空闲中断: 一帧遥控数据收完(总线安静一个字节时间)就会进来。
   由 it.c 里 CubeMX 生成的 UART5_IRQHandler() 调用(名字不能叫 UART5_IRQHandler, 会重复定义) */
void RC_UART5_IRQHandler(void)
{
    /* 1) 先清错误标志。
       STM32H7 的 __HAL_UART_CLEAR_PEFLAG 只是写 ICR.PECF, 不像F4那样"读SR+读DR"顺带把
       RXNE 清掉, 所以它清不掉接收状态。原来的写法是 if(RXNE){...} else if(IDLE){...},
       只要 RXNE 还挂着(噪声/错误帧/DMA没搬数据都会让它挂着), IDLE 分支就永远进不去,
       空闲标志永远清不掉 -> UART5 中断无限重入, CPU 全耗在中断里
       (现象: uwTick 不再增长, 遥控、电机、串口日志全部无反应)。 */
    if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_ORE) ||
        __HAL_UART_GET_FLAG(&huart5, UART_FLAG_FE)  ||
        __HAL_UART_GET_FLAG(&huart5, UART_FLAG_NE)  ||
        __HAL_UART_GET_FLAG(&huart5, UART_FLAG_PE))
    {
        __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_OREF | UART_CLEAR_FEF |
                                       UART_CLEAR_NEF  | UART_CLEAR_PEF);
    }

    /* 2) 空闲中断: 用独立的 if 判断, 不再被 RXNE 挡掉 */
    if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_IDLE))
    {
        static uint16_t this_time_rx_len = 0;

        /* STM32H7 的串口空闲标志必须写 ICR 才会清掉(F4那套"读SR再读DR自动清"在H7上不成立),
           少了下面这句会一直重复进中断, 整个程序看上去像卡死 */
        __HAL_UART_CLEAR_IDLEFLAG(&huart5);

        uint32_t now = HAL_GetTick();

        if ((RC_DMA_STREAM->CR & DMA_SxCR_CT) == RESET)
        {
            /* 当前DMA正在写的是 缓冲区0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_uart5_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - RC_DMA_STREAM->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            RC_DMA_STREAM->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            RC_DMA_STREAM->CR |= DMA_SxCR_CT;

            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_uart5_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);

                COUNT_RECEIVED

                //记录数据接收时间
                last_receive_time = HAL_GetTick();
                detect_hook(DBUS_TOE);
            }
        }
        else
        {
            /* 当前DMA正在写的是 缓冲区1 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_uart5_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - RC_DMA_STREAM->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            RC_DMA_STREAM->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            RC_DMA_STREAM->CR &= ~(DMA_SxCR_CT);

            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_uart5_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);

                COUNT_RECEIVED

                //记录数据接收时间
                last_receive_time = HAL_GetTick();
                detect_hook(DBUS_TOE);
            }
        }
    }

}

//取正函数
static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}

/**
  * @brief          DT7(D-BUS)协议解析
  * @param[in]      sbus_buf: 原生数据指针, 18字节
  * @param[out]     rc_ctrl: 遥控器数据指针
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                       //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                  //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

// clang-format on

/******************************************************************/
/* API                                                            */
/******************************************************************/

/**
  * @brief          获取遥控器是否离线。
  * @retval         true:离线，false:在线
  */
bool GetRcOffline(void)
{
    /* 开机头几帧不算在线; 超过 RC_LOST_TIME(100ms) 没收到新帧就算掉线 */
    return !((receive_count > 5) && (HAL_GetTick() - last_receive_time < RC_LOST_TIME));
}

/**
  * @brief          获取DT7遥控器通道值。
  * @param[in]      ch 通道id，0-右平, 1-右竖, 2-左平, 3-左竖, 4-左滚轮，配合ch id宏进行使用
  * @retval         DT7遥控器通道值，范围为 [-1,1]
  */
float GetDt7RcCh(uint8_t ch) { return rc_ctrl.rc.ch[ch] * RC_TO_ONE; }

/**
  * @brief          获取DT7遥控器拨杆值，可配合switch_is_xxx系列宏函数使用。
  * @param[in]      sw 通道id，0-右, 1-左，配合sw id宏进行使用
  * @retval         DT7遥控器拨杆值，范围为{1,2,3}
  */
char GetDt7RcSw(uint8_t sw) { return rc_ctrl.rc.s[sw]; }

/**
  * @brief          获取鼠标axis轴的移动速度
  * @param[in]      axis 轴id, 配合 AX_X / AX_Y / AX_Z 宏进行使用
  * @retval         鼠标axis轴移动速度
  */
float GetDt7MouseSpeed(uint8_t axis)
{
    switch (axis) {
        case AX_X:
            return rc_ctrl.mouse.x;
        case AX_Y:
            return rc_ctrl.mouse.y;
        case AX_Z:
            return rc_ctrl.mouse.z;
        default:
            return 0;
    }
}

/**
  * @brief          获取鼠标按键信息
  * @param[in]      key 按键id，配合按键id宏进行使用
  * @retval         鼠标按键是否被按下
  */
bool GetDt7Mouse(uint8_t key)
{
    switch (key) {
        case KEY_LEFT:
            return rc_ctrl.mouse.press_l;
        case KEY_RIGHT:
            return rc_ctrl.mouse.press_r;
        default:
            return 0;
    }
}

/**
  * @brief          获取键盘按键信息
  * @param[in]      key 按键id，配合按键id宏进行使用
  * @retval         键盘按键是否被按下
  */
bool GetDt7Keyboard(uint8_t key) { return rc_ctrl.key.v & ((uint16_t)1 << key); }
/*------------------------------ End of File ------------------------------*/
