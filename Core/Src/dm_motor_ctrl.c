/**
  ******************************************************************************
  * @file    dm_motor_ctrl.c
  * @brief   达妙(DM)电机驱动: FDCAN底层 + 达妙控制协议 + 电机寄存器读写
  *          对应《达妙电机上手流程(二)-使用开发板控制电机》第 3 ~ 8 章
  ******************************************************************************
  */

#include "dm_motor_ctrl.h"
#include "usart.h"
#include "remote_control.h"
#include "detect_task.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* 全局电机对象数组, 具体参数在 motor_control.c 的 dm_motor_init() 里配置 */
motor_t motor[DM_MOTOR_NUM];

/* 当前使用的FDCAN1引脚组, fdcan.c 的 HAL_FDCAN_MspInit() 会读它.
   达妙 H7 底板(即官方例程 dm_ctrl(h7 fdcan) 那块板)的CAN收发器接在 PD0/PD1 上, 所以默认用这组;
   PA11/PA12 只是CubeMX给FDCAN1的默认引脚, 用自己的收发器模块时才需要切回那一组 */
uint8_t dm_fdcan1_pin_sel = DM_FDCAN1_PIN_PD0_PD1;

/* 总线掉线(bus-off)标志: 由错误中断置位, 定时器/主循环里调用 dm_can_recover() 恢复 */
volatile uint8_t dm_can_busoff;

/* 接收统计, 供 main.c 的LED自检使用 */
static uint32_t dm_rx_cnt;      /* 成功解析的反馈帧数 */
static uint32_t dm_rx_any;      /* FIFO0收到的任意帧数 */
static uint16_t dm_rx_last_id;  /* 最近一帧的ID */

/* bus-off时刻: 用来判断"总线上到底有没有节点在ACK".
   只看错误计数会被bus-off自动恢复清零, 所以还要看"最近有没有掉过线" */
static volatile uint32_t dm_busoff_tick;

/* ========================= 1. 浮点/整型互转 (文档7.2) ========================= */

/* 把浮点数按线性映射转成无符号整数, 超出范围时按边界处理 */
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;

    if (span <= 0.0f)
    {
        return 0;
    }
    if (x_float < x_min)
    {
        x_float = x_min;
    }
    if (x_float > x_max)
    {
        x_float = x_max;
    }
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

/* 把无符号整数按线性映射还原成浮点数 */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;

    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/* ========================= 2. FDCAN底层BSP (文档3~6章) ========================= */

/* 数据长度 -> FDCAN的DLC编码 */
static uint32_t dm_len_to_dlc(uint32_t len)
{
    switch (len)
    {
        case 0:  return FDCAN_DLC_BYTES_0;
        case 1:  return FDCAN_DLC_BYTES_1;
        case 2:  return FDCAN_DLC_BYTES_2;
        case 3:  return FDCAN_DLC_BYTES_3;
        case 4:  return FDCAN_DLC_BYTES_4;
        case 5:  return FDCAN_DLC_BYTES_5;
        case 6:  return FDCAN_DLC_BYTES_6;
        case 7:  return FDCAN_DLC_BYTES_7;
        case 8:  return FDCAN_DLC_BYTES_8;
        case 12: return FDCAN_DLC_BYTES_12;
        case 16: return FDCAN_DLC_BYTES_16;
        case 20: return FDCAN_DLC_BYTES_20;
        case 24: return FDCAN_DLC_BYTES_24;
        case 32: return FDCAN_DLC_BYTES_32;
        case 48: return FDCAN_DLC_BYTES_48;
        case 64: return FDCAN_DLC_BYTES_64;
        default: return FDCAN_DLC_BYTES_8;
    }
}

/* FDCAN的DLC编码 -> 数据长度 */
static uint32_t dm_dlc_to_len(uint32_t dlc)
{
    uint32_t code = (dlc >> 16) & 0x0Fu;

    if (code <= 8u)
    {
        return code;
    }
    switch (code)
    {
        case 9:  return 12;
        case 10: return 16;
        case 11: return 20;
        case 12: return 24;
        case 13: return 32;
        case 14: return 48;
        default: return 64;
    }
}

/* 发送一帧标准数据帧, 见文档5.2; 返回0成功, 1失败 */
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef pTxHeader = {0};

    pTxHeader.Identifier         = id;                              /* 标准ID */
    pTxHeader.IdType             = FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType        = FDCAN_DATA_FRAME;                /* 数据帧 */
    pTxHeader.DataLength         = dm_len_to_dlc(len);              /* 数据长度 */
    pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;               /* error active */
    pTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;              /* 不存Tx events */
    pTxHeader.MessageMarker      = 0;

    /* 帧格式跟随外设配置: 经典CAN不能带FDF/BRS位, 否则总线上的其他节点会报格式错误 */
    switch (hfdcan->Init.FrameFormat)
    {
        case FDCAN_FRAME_FD_BRS:
            pTxHeader.BitRateSwitch = FDCAN_BRS_ON;
            pTxHeader.FDFormat      = FDCAN_FD_CAN;
            break;
        case FDCAN_FRAME_FD_NO_BRS:
            pTxHeader.BitRateSwitch = FDCAN_BRS_OFF;
            pTxHeader.FDFormat      = FDCAN_FD_CAN;
            break;
        default:
            pTxHeader.BitRateSwitch = FDCAN_BRS_OFF;
            pTxHeader.FDFormat      = FDCAN_CLASSIC_CAN;
            break;
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data) != HAL_OK)
    {
        return 1;
    }
    return 0;
}

/* 从FIFO0取一帧, 返回数据长度, 0表示没有取到, 见文档6.2 */
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint16_t *rec_id, uint8_t *buf)
{
    FDCAN_RxHeaderTypeDef pRxHeader = {0};

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &pRxHeader, buf) != HAL_OK)
    {
        return 0;
    }
    *rec_id = (uint16_t)pRxHeader.Identifier;
    return (uint8_t)dm_dlc_to_len(pRxHeader.DataLength);
}

/* 根据FDCAN内核时钟和目标波特率算出一组时间参数, 见文档3.1~3.2
   采样点: 波特率>800k用75%, >500k用80%, <=500k用87.5% */
static void dm_fdcan_calc_timing(uint32_t clock, uint32_t baud,
                                 uint32_t *prescaler, uint32_t *seg1, uint32_t *seg2)
{
    uint32_t best_p = 1u, best_total = 0u, best_err = 0xFFFFFFFFu;
    uint32_t sample, total, s1;

    for (uint32_t p = 1u; p <= 32u; p++)
    {
        total = clock / (p * baud);
        if ((total < 8u) || (total > 25u))      /* tq总数控制在8~25, 太少精度低, 太多没意义 */
        {
            continue;
        }
        /* 优先选能整除(误差0)的分频, 误差相同时选tq多的 */
        uint32_t err = clock - (p * baud * total);
        if ((err < best_err) || ((err == best_err) && (total > best_total)))
        {
            best_err = err;
            best_p = p;
            best_total = total;
        }
    }

    if (best_total == 0u)                       /* 兜底: 按1M的常用参数 */
    {
        best_p = 1u;
        best_total = 20u;
    }

    if (baud > 800000u)
    {
        sample = 750u;
    }
    else if (baud > 500000u)
    {
        sample = 800u;
    }
    else
    {
        sample = 875u;
    }

    s1 = ((best_total - 1u) * sample + 500u) / 1000u;   /* 四舍五入 */
    if (s1 < 2u)
    {
        s1 = 2u;
    }
    if (s1 > (best_total - 2u))
    {
        s1 = best_total - 2u;
    }

    *prescaler = best_p;
    *seg1      = s1;
    *seg2      = best_total - 1u - s1;                  /* SJW取TSEG2, 提高位宽容忍度 */
}

/* 设置CAN波特率, 见文档3.4
   mode: CAN_CLASS(经典CAN) / CAN_FD_BRS(FDCAN)
   baud: CAN_BR_125K ~ CAN_BR_5M
   注意: 会DeInit/重新Init FDCAN, 调用后必须重新 bsp_can_init() */
void bsp_fdcan_set_baud(FDCAN_HandleTypeDef *hfdcan, uint8_t mode, uint8_t baud)
{
    static const uint32_t baud_table[10] =
    {
        125000u, 200000u, 250000u, 500000u, 1000000u,
        2000000u, 2500000u, 3200000u, 4000000u, 5000000u
    };
    uint32_t target = (baud < 10u) ? baud_table[baud] : 1000000u;
    uint32_t clk;
    uint32_t nom_brp, nom_seg1, nom_seg2;
    uint32_t dat_brp, dat_seg1, dat_seg2;

    /* FDCAN内核时钟, 本工程由SystemClock_Config()里的PLL1Q提供(54MHz),
       不能照抄文档里的80MHz计算表, 这里按实际时钟算 */
    clk = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
    if (clk == 0u)
    {
        clk = 54000000u;
    }

    if (mode == CAN_CLASS)
    {
        hfdcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
        dm_fdcan_calc_timing(clk, target, &nom_brp, &nom_seg1, &nom_seg2);
        dm_fdcan_calc_timing(clk, 1000000u, &dat_brp, &dat_seg1, &dat_seg2);    /* 数据域默认1M */
    }
    else
    {
        hfdcan->Init.FrameFormat = FDCAN_FRAME_FD_BRS;
        dm_fdcan_calc_timing(clk, 1000000u, &nom_brp, &nom_seg1, &nom_seg2);    /* 仲裁域默认1M */
        dm_fdcan_calc_timing(clk, target, &dat_brp, &dat_seg1, &dat_seg2);
    }

    (void)HAL_FDCAN_DeInit(hfdcan);

    hfdcan->Init.NominalPrescaler     = nom_brp;
    hfdcan->Init.NominalTimeSeg1      = nom_seg1;
    hfdcan->Init.NominalTimeSeg2      = nom_seg2;
    hfdcan->Init.NominalSyncJumpWidth = nom_seg2;
    hfdcan->Init.DataPrescaler        = dat_brp;
    hfdcan->Init.DataTimeSeg1         = dat_seg1;
    hfdcan->Init.DataTimeSeg2         = dat_seg2;
    hfdcan->Init.DataSyncJumpWidth    = dat_seg2;

    if (HAL_FDCAN_Init(hfdcan) != HAL_OK)
    {
        Error_Handler();
    }
}

/* 滤波器初始化, 见文档4.2: 掩码为0, 接收所有ID */
void can_filter_init(void)
{
    FDCAN_FilterTypeDef fdcan_filter = {0};

    fdcan_filter.IdType       = FDCAN_STANDARD_ID;          /* 标准ID */
    fdcan_filter.FilterIndex  = 0;                          /* 滤波器索引 */
    fdcan_filter.FilterType   = FDCAN_FILTER_MASK;          /* FilterID1=ID, FilterID2=掩码 */
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;    /* 匹配后存到FIFO0 */
    fdcan_filter.FilterID1    = 0x00;
    fdcan_filter.FilterID2    = 0x00;                       /* 掩码0 -> 不过滤ID */
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter) != HAL_OK)
    {
        Error_Handler();
    }

    /* 拒绝接收匹配不成功的标准ID和扩展ID, 不接受远程帧 */
    (void)HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                       FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    (void)HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
}

/* 启动CAN: 配置滤波器 -> 打开FIFO0中断 -> Start, 见文档4.2/6.3
   bsp_fdcan_set_baud()之后必须调用一次 */
void bsp_can_init(void)
{
    can_filter_init();

    /* 用"每收到一帧就中断"(NEW_MESSAGE), 不要用水位中断(WATERMARK):
       水位标志只在FIFO填充量"跨过"水位的那一刻置位, 而回调里一次只取一帧,
       残留和后续到达的帧不会再触发中断, 表现就是"收了一会儿就再也收不到" */
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }
    /* 打开bus-off中断: 总线上没有其他节点(没接电机/接线错/波特率不对)时会掉线,
       必须恢复, 否则发不出任何控制帧 */
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_BUS_OFF, 0) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }

    dm_can_busoff = 0;
}

/* ========================= 3. 达妙控制协议帧 (文档7章) ========================= */

/* MIT模式控制帧, 见文档7.2: Position(16bit)+Velocity(12bit)+Kp(12bit)+Kd(12bit)+Torque(12bit) 组合成8字节 */
void mit_ctrl(FDCAN_HandleTypeDef *hcan, motor_t *motor, uint16_t motor_id,
              float pos, float vel, float kp, float kd, float tor)
{
    uint8_t  data[8];
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    uint16_t id = motor_id + MIT_MODE;

    /* 映射范围和电机内部设置保持一致, 否则给定值会被等比例缩放 */
    pos_tmp = (uint16_t)float_to_uint(pos, -motor->tmp.PMAX, motor->tmp.PMAX, 16);
    vel_tmp = (uint16_t)float_to_uint(vel, -motor->tmp.VMAX, motor->tmp.VMAX, 12);
    tor_tmp = (uint16_t)float_to_uint(tor, -motor->tmp.TMAX, motor->tmp.TMAX, 12);
    kp_tmp  = (uint16_t)float_to_uint(kp, KP_MIN, KP_MAX, 12);
    kd_tmp  = (uint16_t)float_to_uint(kd, KD_MIN, KD_MAX, 12);

    data[0] = (uint8_t)(pos_tmp >> 8);
    data[1] = (uint8_t)(pos_tmp);
    data[2] = (uint8_t)(vel_tmp >> 4);
    data[3] = (uint8_t)(((vel_tmp & 0x0Fu) << 4) | (kp_tmp >> 8));
    data[4] = (uint8_t)(kp_tmp);
    data[5] = (uint8_t)(kd_tmp >> 4);
    data[6] = (uint8_t)(((kd_tmp & 0x0Fu) << 4) | (tor_tmp >> 8));
    data[7] = (uint8_t)(tor_tmp);

    (void)fdcanx_send_data(hcan, id, data, 8);
}

/* 位置速度模式控制帧, 见文档7.3: 位置和速度都是float, 低位在前 */
void pos_ctrl(FDCAN_HandleTypeDef *hcan, uint16_t motor_id, float pos, float vel)
{
    uint8_t  data[8];
    uint16_t id = motor_id + POS_MODE;

    memcpy(&data[0], &pos, 4);
    memcpy(&data[4], &vel, 4);

    (void)fdcanx_send_data(hcan, id, data, 8);
}

/* 速度模式控制帧, 见文档7.4: 只有速度, float, 低位在前 */
void spd_ctrl(FDCAN_HandleTypeDef *hcan, uint16_t motor_id, float vel)
{
    uint8_t  data[4];
    uint16_t id = motor_id + SPD_MODE;

    memcpy(&data[0], &vel, 4);

    (void)fdcanx_send_data(hcan, id, data, 4);
}

/* 力位混控模式控制帧, 见文档7.5: 位置(float) + 限速(u16, 放大100倍) + 电流标幺值(u16, 放大10000倍) */
void psi_ctrl(FDCAN_HandleTypeDef *hcan, uint16_t motor_id, float pos, float vel, float cur)
{
    uint8_t  data[8];
    uint16_t id = motor_id + PSI_MODE;
    uint16_t u16_vel;
    uint16_t u16_cur;

    if (vel < 0.0f)
    {
        vel = 0.0f;
    }
    if (cur < 0.0f)
    {
        cur = 0.0f;
    }
    u16_vel = (uint16_t)(vel * 100.0f);
    u16_cur = (uint16_t)(cur * 10000.0f);
    if (u16_vel > 10000u)
    {
        u16_vel = 10000u;       /* 限幅100rad/s */
    }
    if (u16_cur > 10000u)
    {
        u16_cur = 10000u;       /* 限幅1.0 */
    }

    memcpy(&data[0], &pos, 4);
    data[4] = (uint8_t)(u16_vel & 0xFFu);
    data[5] = (uint8_t)(u16_vel >> 8);
    data[6] = (uint8_t)(u16_cur & 0xFFu);
    data[7] = (uint8_t)(u16_cur >> 8);

    (void)fdcanx_send_data(hcan, id, data, 8);
}

/* 使能/失能/置零/清错都是8字节的FF+命令字, 见文档7.6~7.9 */
static void dm_motor_cmd(FDCAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id, uint8_t cmd)
{
    uint8_t  data[8];
    uint16_t id = motor_id + mode_id;

    memset(data, 0xFF, 7);
    data[7] = cmd;

    (void)fdcanx_send_data(hcan, id, data, 8);
}

/* 使能帧: ...FC */
void enable_motor_mode(FDCAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id)
{
    dm_motor_cmd(hcan, motor_id, mode_id, 0xFC);
}

/* 失能帧: ...FD */
void disable_motor_mode(FDCAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id)
{
    dm_motor_cmd(hcan, motor_id, mode_id, 0xFD);
}

/* 设置零点: ...FE */
void save_pos_zero(FDCAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id)
{
    dm_motor_cmd(hcan, motor_id, mode_id, 0xFE);
}

/* 清除错误: ...FB */
void clear_err(FDCAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id)
{
    dm_motor_cmd(hcan, motor_id, mode_id, 0xFB);
}

/* ========================= 4. 上层封装API (文档7.10) ========================= */

/* 按当前控制模式使能电机 */
void dm_motor_enable(FDCAN_HandleTypeDef *hcan, motor_t *motor)
{
    switch (motor->ctrl.mode)
    {
        case mit_mode: enable_motor_mode(hcan, motor->id, MIT_MODE); break;
        case pos_mode: enable_motor_mode(hcan, motor->id, POS_MODE); break;
        case spd_mode: enable_motor_mode(hcan, motor->id, SPD_MODE); break;
        case psi_mode: enable_motor_mode(hcan, motor->id, PSI_MODE); break;
        default: break;
    }
}

/* 四种模式的失能帧都发一遍, 保证无论电机内部是什么模式都能停下来 */
void dm_motor_disable(FDCAN_HandleTypeDef *hcan, motor_t *motor)
{
    disable_motor_mode(hcan, motor->id, MIT_MODE);
    disable_motor_mode(hcan, motor->id, POS_MODE);
    disable_motor_mode(hcan, motor->id, SPD_MODE);
    disable_motor_mode(hcan, motor->id, PSI_MODE);
    dm_motor_clear_para(motor);
}

/* 四种模式的清错帧都发一遍 */
void dm_motor_clear_err(FDCAN_HandleTypeDef *hcan, motor_t *motor)
{
    clear_err(hcan, motor->id, MIT_MODE);
    clear_err(hcan, motor->id, POS_MODE);
    clear_err(hcan, motor->id, SPD_MODE);
    clear_err(hcan, motor->id, PSI_MODE);
}

/* 清掉反馈和给定值, 但保留控制模式, 否则失能后就没法再按原模式使能了 */
void dm_motor_clear_para(motor_t *motor)
{
    memset(&motor->para, 0, sizeof(motor->para));
    motor->ctrl.pos_set = 0.0f;
    motor->ctrl.vel_set = 0.0f;
    motor->ctrl.tor_set = 0.0f;
    motor->ctrl.cur_set = 0.0f;
    motor->ctrl.kp_set  = 0.0f;
    motor->ctrl.kd_set  = 0.0f;
}

/* 按当前控制模式周期发送控制帧, 建议1kHz调用一次, 见文档7.10 */
void dm_motor_ctrl_send(FDCAN_HandleTypeDef *hcan, motor_t *motor)
{
    switch (motor->ctrl.mode)
    {
        case mit_mode:
            mit_ctrl(hcan, motor, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set,
                     motor->ctrl.kp_set, motor->ctrl.kd_set, motor->ctrl.tor_set);
            break;
        case pos_mode:
            pos_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set);
            break;
        case spd_mode:
            spd_ctrl(hcan, motor->id, motor->ctrl.vel_set);
            break;
        case psi_mode:
            psi_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.cur_set);
            break;
        default:
            break;
    }
}

/* 解析电机反馈帧, 见文档7.1
   D[0]=ID|ERR<<4, D[1:2]=位置, D[3:4]高4位=速度, D[4]低4位~D[5]=扭矩, D[6]=MOS温度, D[7]=线圈温度 */
void dm_motor_fbdata(motor_t *motor, uint8_t *rx_data)
{
    motor->para.id    = rx_data[0] & 0x0F;
    motor->para.state = rx_data[0] >> 4;
    motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
    motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    motor->para.t_int = ((rx_data[4] & 0x0F) << 8) | rx_data[5];

    motor->para.pos = uint_to_float(motor->para.p_int, -motor->tmp.PMAX, motor->tmp.PMAX, 16);
    motor->para.vel = uint_to_float(motor->para.v_int, -motor->tmp.VMAX, motor->tmp.VMAX, 12);
    motor->para.tor = uint_to_float(motor->para.t_int, -motor->tmp.TMAX, motor->tmp.TMAX, 12);
    motor->para.Tmos  = (float)rx_data[6];
    motor->para.Tcoil = (float)rx_data[7];
}

/* ========================= 5. 寄存器读写API (文档8章) ========================= */

/* 读取电机寄存器: 往广播ID 0x7FF发命令, 见文档8.2 */
void read_motor_data(uint16_t id, uint8_t rid)
{
    uint8_t can_id_l = (uint8_t)(id & 0x0F);
    uint8_t can_id_h = (uint8_t)((id >> 4) & 0x0F);
    uint8_t data[4]  = {can_id_l, can_id_h, 0x33, rid};

    (void)fdcanx_send_data(&hfdcan1, 0x7FF, data, 4);
}

/* 写入电机寄存器(立即生效但不掉电保存), 见文档8.3 */
void write_motor_data(uint16_t id, uint8_t rid, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
    uint8_t can_id_l = (uint8_t)(id & 0xFF);
    uint8_t can_id_h = (uint8_t)((id >> 8) & 0x07);
    uint8_t data[8]  = {can_id_l, can_id_h, 0x55, rid, d0, d1, d2, d3};

    (void)fdcanx_send_data(&hfdcan1, 0x7FF, data, 8);
}

/* 保存电机寄存器到片内flash, 只在失能模式下生效, 见文档8.4
   数据第4个字节按官方H7例程固定发 0x01(之前发的是寄存器号, 电机不认, 保存不会生效) */
void save_motor_data(uint16_t id, uint8_t rid)
{
    uint8_t can_id_l = (uint8_t)(id & 0xFF);
    uint8_t can_id_h = (uint8_t)((id >> 8) & 0x07);
    uint8_t data[4]  = {can_id_l, can_id_h, 0xAA, 0x01};

    (void)rid;      /* 保留形参是为了和官方例程的函数原型一致, 协议里用不到 */
    (void)fdcanx_send_data(&hfdcan1, 0x7FF, data, 4);
}

/* 解析读寄存器的反馈帧, 见文档8.2; 调用前需要 motor->tmp.read_flag != 0 */
void receive_motor_data(motor_t *motor, uint8_t *data)
{
    float_type_u y;
    uint16_t rid_value;

    if (motor->tmp.read_flag == 0)
    {
        return;
    }
    if (data[2] != 0x33)
    {
        return;
    }

    rid_value = data[3];
    y.b_val[0] = data[4];
    y.b_val[1] = data[5];
    y.b_val[2] = data[6];
    y.b_val[3] = data[7];

    switch (rid_value)
    {
        case RID_UV_VALUE: motor->tmp.UV_Value = y.f_val; motor->tmp.read_flag =  2; break;
        case RID_KT_VALUE: motor->tmp.KT_Value = y.f_val; motor->tmp.read_flag =  3; break;
        case RID_OT_VALUE: motor->tmp.OT_Value = y.f_val; motor->tmp.read_flag =  4; break;
        case RID_OC_VALUE: motor->tmp.OC_Value = y.f_val; motor->tmp.read_flag =  5; break;
        case RID_ACC:      motor->tmp.ACC      = y.f_val; motor->tmp.read_flag =  6; break;
        case RID_DEC:      motor->tmp.DEC      = y.f_val; motor->tmp.read_flag =  7; break;
        case RID_MAX_SPD:  motor->tmp.MAX_SPD  = y.f_val; motor->tmp.read_flag =  8; break;
        case RID_MST_ID:   motor->tmp.MST_ID   = y.u_val; motor->tmp.read_flag =  9; break;
        case RID_ESC_ID:   motor->tmp.ESC_ID   = y.u_val; motor->tmp.read_flag = 10; break;
        case RID_TIMEOUT:  motor->tmp.TIMEOUT  = y.u_val; motor->tmp.read_flag = 11; break;
        case RID_CMODE:    motor->tmp.cmode    = y.u_val; motor->tmp.read_flag = 12; break;
        case RID_DAMP:     motor->tmp.Damp     = y.f_val; motor->tmp.read_flag = 13; break;
        case RID_INERTIA:  motor->tmp.Inertia  = y.f_val; motor->tmp.read_flag = 14; break;
        case RID_HW_VER:   motor->tmp.hw_ver   = y.u_val; motor->tmp.read_flag = 15; break;
        case RID_SW_VER:   motor->tmp.sw_ver   = y.u_val; motor->tmp.read_flag = 16; break;
        case RID_SN:       motor->tmp.SN       = y.u_val; motor->tmp.read_flag = 17; break;
        case RID_NPP:      motor->tmp.NPP      = y.u_val; motor->tmp.read_flag = 18; break;
        case RID_RS:       motor->tmp.Rs       = y.f_val; motor->tmp.read_flag = 19; break;
        case RID_LS:       motor->tmp.Ls       = y.f_val; motor->tmp.read_flag = 20; break;
        case RID_FLUX:     motor->tmp.Flux     = y.f_val; motor->tmp.read_flag = 21; break;
        case RID_GR:       motor->tmp.Gr       = y.f_val; motor->tmp.read_flag = 22; break;
        case RID_PMAX:     motor->tmp.PMAX     = y.f_val; motor->tmp.read_flag = 23; break;
        case RID_VMAX:     motor->tmp.VMAX     = y.f_val; motor->tmp.read_flag = 24; break;
        case RID_TMAX:     motor->tmp.TMAX     = y.f_val; motor->tmp.read_flag = 25; break;
        case RID_I_BW:     motor->tmp.I_BW     = y.f_val; motor->tmp.read_flag = 26; break;
        case RID_KP_ASR:   motor->tmp.KP_ASR   = y.f_val; motor->tmp.read_flag = 27; break;
        case RID_KI_ASR:   motor->tmp.KI_ASR   = y.f_val; motor->tmp.read_flag = 28; break;
        case RID_KP_APR:   motor->tmp.KP_APR   = y.f_val; motor->tmp.read_flag = 29; break;
        case RID_KI_APR:   motor->tmp.KI_APR   = y.f_val; motor->tmp.read_flag = 30; break;
        case RID_OV_VALUE: motor->tmp.OV_Value = y.f_val; motor->tmp.read_flag = 31; break;
        case RID_GREF:     motor->tmp.GREF     = y.f_val; motor->tmp.read_flag = 32; break;
        case RID_DETA:     motor->tmp.Deta     = y.f_val; motor->tmp.read_flag = 33; break;
        case RID_V_BW:     motor->tmp.V_BW     = y.f_val; motor->tmp.read_flag = 34; break;
        case RID_IQ_CL:    motor->tmp.IQ_cl    = y.f_val; motor->tmp.read_flag = 35; break;
        case RID_VL_CL:    motor->tmp.VL_cl    = y.f_val; motor->tmp.read_flag = 36; break;
        case RID_CAN_BR:   motor->tmp.can_br   = y.u_val; motor->tmp.read_flag = 37; break;
        case RID_SUB_VER:  motor->tmp.sub_ver  = y.u_val; motor->tmp.read_flag = 38; break;
        case RID_U_OFF:    motor->tmp.u_off    = y.f_val; motor->tmp.read_flag = 39; break;
        case RID_V_OFF:    motor->tmp.v_off    = y.f_val; motor->tmp.read_flag = 40; break;
        case RID_K1:       motor->tmp.k1       = y.f_val; motor->tmp.read_flag = 41; break;
        case RID_K2:       motor->tmp.k2       = y.f_val; motor->tmp.read_flag = 42; break;
        case RID_M_OFF:    motor->tmp.m_off    = y.f_val; motor->tmp.read_flag = 43; break;
        case RID_DIR:      motor->tmp.dir      = y.f_val; motor->tmp.read_flag = 44; break;
        case RID_P_M:      motor->tmp.p_m      = y.f_val; motor->tmp.read_flag = 45; break;
        case RID_X_OUT:    motor->tmp.x_out    = y.f_val; motor->tmp.read_flag =  0; break;
        default: break;
    }
}

/* 依次读取全部寄存器, 见文档9.5
   反馈帧由 receive_motor_data() 解析, 解析完最后一个寄存器会把 read_flag 清0 */
void read_all_motor_data(motor_t *motor)
{
    static const uint8_t rid_list[] =
    {
        RID_UV_VALUE, RID_KT_VALUE, RID_OT_VALUE, RID_OC_VALUE, RID_ACC,
        RID_DEC,      RID_MAX_SPD,  RID_MST_ID,  RID_ESC_ID,  RID_TIMEOUT,
        RID_CMODE,    RID_DAMP,     RID_INERTIA, RID_HW_VER,  RID_SW_VER,
        RID_SN,       RID_NPP,      RID_RS,      RID_LS,      RID_FLUX,
        RID_GR,       RID_PMAX,     RID_VMAX,    RID_TMAX,    RID_I_BW,
        RID_KP_ASR,   RID_KI_ASR,   RID_KP_APR,  RID_KI_APR,  RID_OV_VALUE,
        RID_GREF,     RID_DETA,     RID_V_BW,    RID_IQ_CL,   RID_VL_CL,
        RID_CAN_BR,   RID_SUB_VER,  RID_U_OFF,   RID_V_OFF,   RID_K1,
        RID_K2,       RID_M_OFF,    RID_DIR,     RID_P_M,     RID_X_OUT,
    };

    motor->tmp.read_flag = 1;
    for (uint32_t i = 0; i < (sizeof(rid_list) / sizeof(rid_list[0])); i++)
    {
        read_motor_data(motor->id, rid_list[i]);
    }
}

/* ============================== 6. 板级/诊断 ============================== */

/* ---- 调试串口: USART1(板上的4pin USART1座: TX=PA9, RX=PA10), 115200, 打印诊断信息用 ----
   没有USB转串口时可以不看, 但"电机不转"时这几行日志能直接说明卡在哪一步 */
uint8_t dm_log_enable = 1;      /* 置0后dm_log()什么都不发 */

void dm_log(const char *fmt, ...)
{
    char    buf[192];
    va_list ap;
    int     n;

    if (dm_log_enable == 0u)
    {
        return;
    }

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n <= 0)
    {
        return;
    }
    if (n > (int)sizeof(buf) - 1)
    {
        n = (int)sizeof(buf) - 1;
    }
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)n, 30);
}

/* 打印定点小数, 不依赖newlib的%f支持: 12.345 -> " PMAX=12.345" */
static void dm_log_fix(const char *name, float v)
{
    int32_t milli = (int32_t)(v * 1000.0f);
    int32_t ip    = milli / 1000;
    int32_t fp    = milli % 1000;

    if (fp < 0)
    {
        fp = -fp;
    }
    dm_log(" %s=%ld.%03ld", name, (long)ip, (long)fp);
}

/* ---- CAN引脚切换: 达妙底板的收发器在PD0/PD1, CubeMX默认给的是PA11/PA12 ---- */
void dm_can_set_pins(uint8_t sel)
{
    if (sel == dm_fdcan1_pin_sel)
    {
        return;
    }
    dm_fdcan1_pin_sel = sel;
    HAL_NVIC_DisableIRQ(TIM3_IRQn);                        /* 切换期间先停掉1kHz控制帧 */
    bsp_fdcan_set_baud(&hfdcan1, CAN_CLASS, CAN_BR_1M);   /* DeInit+Init, MspInit按新引脚重配GPIO */
    bsp_can_init();
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    dm_log("CAN pins -> %s\r\n", sel ? "PD0/PD1" : "PA11/PA12");
}

uint8_t dm_can_get_pins(void)
{
    return dm_fdcan1_pin_sel;
}

/* 读CAN错误计数: TEC为0说明发出去的帧都被ACK了 */
static uint8_t dm_can_error_counters(uint8_t *tec, uint8_t *rec)
{
    FDCAN_ErrorCountersTypeDef ec;

    if (HAL_FDCAN_GetErrorCounters(&hfdcan1, &ec) != HAL_OK)
    {
        *tec = 0xFFu;
        *rec = 0xFFu;
        return 0;
    }
    *tec = (uint8_t)ec.TxErrorCnt;
    *rec = (uint8_t)ec.RxErrorCnt;
    return 1;
}

/* 总线上有没有活节点: 发出去的帧被ACK(TEC=0)且没有bus-off.
   注意: 电机没上电/没接线时, 即使引脚正确也收不到ACK */
uint8_t dm_can_link_ok(void)
{
    uint8_t tec = 0;
    uint8_t rec = 0;

    if (dm_can_busoff)
    {
        return 0;
    }
    /* 最近掉过线(说明发出去的帧一直没被ACK), 不算链路通 */
    if ((HAL_GetTick() - dm_busoff_tick) < 300u)
    {
        return 0;
    }
    if (!dm_can_error_counters(&tec, &rec))
    {
        return 0;
    }
    if ((hfdcan1.Instance->PSR & FDCAN_PSR_BO) != 0u)
    {
        return 0;
    }
    return (tec == 0u);
}

/* 总线掉线(bus-off)后重新初始化FDCAN: 滤波器/中断/启动全部重建 */
static uint32_t dm_recover_cnt;

void dm_can_recover(void)
{
    dm_can_busoff = 0;
    bsp_fdcan_set_baud(&hfdcan1, CAN_CLASS, CAN_BR_1M);
    bsp_can_init();
    dm_recover_cnt++;       /* 由 dm_log_status() 打印, 这里不做串口输出 */
}

/* 自动探测CAN收发器接在哪组引脚上, 并判断总线上有没有活节点.
   主循环反复调用(非阻塞, 返回1表示"刚锁定"):
   - 收到帧或帧被ACK -> 锁定当前引脚, 并读回关键寄存器
   - 每600ms没反应就换一组引脚, 换 DM_CAN_PROBE_TRIES 次后停止切换(保留当前引脚) */
#define DM_CAN_PROBE_TRIES  4
/* 达妙 DM-MC-Board02 的CAN1收发器固定在 PD0/PD1(官方管脚图: CAN1 RX=PD00 / TX=PD01),
   正常不需要自动探测。而探测会把FDCAN临时切到 PA11/PA12 —— 那是USB的DM/DP引脚,
   一切过去USB就废了, 而且那组引脚上根本没接CAN收发器, 纯属白折腾(每次上电切4次)。
   所以默认直接锁定PD0/PD1; 真要探测别的板子时把这里改成1 */
#define DM_CAN_PROBE_ENABLE 0

static uint8_t  dm_probe_locked = (DM_CAN_PROBE_ENABLE == 0);
static uint8_t  dm_probe_tries;
static uint32_t dm_probe_tick;

uint8_t dm_can_probe_poll(void)
{
    static uint32_t recover_tick;

    if (dm_probe_locked)
    {
        /* 锁定之后只负责bus-off恢复: 总线上没有节点ACK时不能一直卡在bus-off, 否则再也发不出帧.
           这里放在主循环里做(不在中断里做), 探测期间不恢复, 避免错误计数被清零后误判 */
        if (dm_can_busoff && ((HAL_GetTick() - recover_tick) >= 500u))
        {
            recover_tick = HAL_GetTick();
            dm_can_recover();
        }
        return 0;
    }
    if (dm_probe_tick == 0u)
    {
        dm_probe_tick = HAL_GetTick();
        return 0;
    }
    if ((HAL_GetTick() - dm_probe_tick) < 600u)
    {
        return 0;
    }
    dm_probe_tick = HAL_GetTick();

    if (dm_can_link_ok() || (dm_rx_any > 0u))
    {
        dm_probe_locked = 1;
        dm_log("CAN link OK on %s (rx=%lu fb=%lu)\r\n",
               dm_fdcan1_pin_sel ? "PD0/PD1" : "PA11/PA12",
               (unsigned long)dm_rx_any, (unsigned long)dm_rx_cnt);
        dm_motor_check_regs(&motor[Motor1]);
        return 1;
    }

    if (dm_probe_tries < DM_CAN_PROBE_TRIES)
    {
        dm_probe_tries++;
        dm_can_set_pins(dm_fdcan1_pin_sel ? DM_FDCAN1_PIN_PA11_PA12 : DM_FDCAN1_PIN_PD0_PD1);
        return 0;
    }

    dm_probe_locked = 1;
    dm_log("CAN: no ACK on either pin set. check motor 24V / CANH-CANL / 120ohm / 1Mbps / CAN_ID=%u\r\n",
           (unsigned)motor[Motor1].id);
    dm_motor_check_regs(&motor[Motor1]);
    return 1;
}

/* 读回关键寄存器: 电机内部CAN ID / Master ID / 控制模式 / CAN波特率 / 映射范围.
   回复帧由 receive_motor_data() 解析, 打印在 dm_log_status() 里 */
void dm_motor_check_regs(motor_t *motor)
{
    motor->tmp.read_flag = 1;
    read_motor_data(motor->id, RID_ESC_ID);
    read_motor_data(motor->id, RID_MST_ID);
    read_motor_data(motor->id, RID_CMODE);
    read_motor_data(motor->id, RID_CAN_BR);
    read_motor_data(motor->id, RID_PMAX);
    read_motor_data(motor->id, RID_VMAX);
    read_motor_data(motor->id, RID_TMAX);
}

/* 1Hz打印一次状态: CAN链路 + 电机反馈 + 关键寄存器 */
void dm_log_status(void)
{
    static uint32_t tick;
    uint8_t tec = 0;
    uint8_t rec = 0;
    motor_t *m  = &motor[Motor1];

    if ((HAL_GetTick() - tick) < 1000u)
    {
        return;
    }
    tick = HAL_GetTick();

    (void)dm_can_error_counters(&tec, &rec);
    dm_log("CAN %s rx=%lu fb=%lu last=0x%03X tec=%u rec=%u recov=%lu pin=%s\r\n",
           dm_can_link_ok() ? "OK " : "BAD",
           (unsigned long)dm_rx_any, (unsigned long)dm_rx_cnt,
           (unsigned)dm_rx_last_id, (unsigned)tec, (unsigned)rec,
           (unsigned long)dm_recover_cnt,
           dm_fdcan1_pin_sel ? "PD0/PD1" : "PA11/PA12");
    /* 遥控(DT7)状态: feed = 距上一帧D-BUS数据的毫秒数, 4294967295(never)表示从没收到过.
       offline=1 或 feed 很大 => 接收机没插/没对频/UART5没接对, 此时电机目标速度恒为0 */
    {
        uint32_t feed = detect_last_feed_time(DBUS_TOE);
        dm_log("RC %s feed=%lums(4294967295=没收到过)",
               GetRcOffline() ? "OFFLINE" : "OK     ", (unsigned long)feed);
        dm_log_fix("CH3", GetDt7RcCh(RC_CH_LEFT_VERTICAL));
        dm_log(" SW_R=%u\r\n", (unsigned)GetDt7RcSw(RC_SW_RIGHT));
    }
    dm_log("motor id=0x%02X mst=0x%02X state=%u(0=失能 1=使能 8=超压 9=欠压 A=过流 B=MOS过温 C=线圈过温 D=通信丢失 E=过载)",
           (unsigned)m->id, (unsigned)m->mst_id, (unsigned)m->para.state);
    dm_log_fix("pos", m->para.pos);
    dm_log_fix("vel", m->para.vel);
    dm_log_fix("tor", m->para.tor);
    dm_log(" Tmos=%u Tcoil=%u\r\n", (unsigned)m->para.Tmos, (unsigned)m->para.Tcoil);
    dm_log("reg ESC_ID=%lu MST_ID=%lu CMODE=%lu CAN_BR=%lu",
           (unsigned long)m->tmp.ESC_ID, (unsigned long)m->tmp.MST_ID,
           (unsigned long)m->tmp.cmode, (unsigned long)m->tmp.can_br);
    dm_log_fix("PMAX", m->tmp.PMAX);
    dm_log_fix("VMAX", m->tmp.VMAX);
    dm_log_fix("TMAX", m->tmp.TMAX);
    dm_log("\r\n");
}

/* 已成功解析的电机反馈帧数, 为0说明CAN链路/ID有问题 */
uint32_t dm_motor_rx_count(void)
{
    return dm_rx_cnt;
}

/* FIFO0收到的任意CAN帧数, 用来区分"总线上一帧都没有"和"ID填错了" */
uint32_t dm_motor_rx_any(void)
{
    return dm_rx_any;
}

/* 最近一帧的CAN ID, 方便调试时看总线上到底是谁在发 */
uint16_t dm_motor_rx_last_id(void)
{
    return dm_rx_last_id;
}

/* FDCAN1接收回调: 取出报文, 找到对应电机后解析, 见文档9.1
   反馈帧: 帧ID = 电机内部的Master ID, D[0]低4位 = 电机自身的CAN ID.
   这里优先按 D[0] 的CAN ID匹配(最可靠, 还能顺便修正Master ID),
   匹配不上再退回按Master ID匹配(寄存器读回复帧用) */
void fdcan1_rx_callback(void)
{
    uint16_t rec_id = 0;
    uint8_t  rx_data[8] = {0};
    int8_t   hit;

    /* FIFO里可能一次积了多帧, 这里一次中断把FIFO取空.
       只取一帧的话, 残留的帧要等下一次中断才处理, 而NEW_MESSAGE中断是按"新帧写入"触发的,
       残留帧会一直躺在FIFO里, 表现就是反馈时断时续 */
    while (fdcanx_receive(&hfdcan1, &rec_id, rx_data) != 0)
    {
        hit = -1;
        dm_rx_any++;
        dm_rx_last_id = rec_id;

        /* 第一轮: 按CAN ID匹配 */
        for (uint8_t i = 0; i < DM_MOTOR_NUM; i++)
        {
            if ((motor[i].id == DM_MOTOR_ID_NONE) || (motor[i].id > 0x0Fu))
            {
                continue;
            }
            if ((rx_data[0] & 0x0Fu) == (motor[i].id & 0x0Fu))
            {
                hit = (int8_t)i;
                break;
            }
        }

        /* 第二轮: 按Master ID匹配(寄存器读回复帧的D[0]不是CAN ID) */
        if (hit < 0)
        {
            for (uint8_t i = 0; i < DM_MOTOR_NUM; i++)
            {
                if ((motor[i].id != DM_MOTOR_ID_NONE) && (motor[i].mst_id == rec_id))
                {
                    hit = (int8_t)i;
                    break;
                }
            }
        }

        if (hit < 0)
        {
            continue;
        }

        /* 帧ID就是电机的Master ID: 如果和配置不一致, 自动纠正, 避免因为Master ID填错而收不到反馈 */
        if (motor[hit].mst_id != rec_id)
        {
            motor[hit].mst_id = rec_id;
        }

        dm_motor_fbdata(&motor[hit], rx_data);
        receive_motor_data(&motor[hit], rx_data);
        dm_rx_cnt++;
    }
}

/* FIFO0中断回调, 见文档6.3 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;

    if (hfdcan == &hfdcan1)
    {
        fdcan1_rx_callback();
    }
}

/* 总线错误中断: bus-off(总线上一直没有ACK)时置标志, 由主循环/定时器恢复 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    if (hfdcan != &hfdcan1)
    {
        return;
    }
    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0u)
    {
        dm_busoff_tick = HAL_GetTick();
        dm_can_busoff = 1;
    }
}
