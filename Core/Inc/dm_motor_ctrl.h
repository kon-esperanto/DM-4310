/**
  ******************************************************************************
  * @file    dm_motor_ctrl.h
  * @brief   达妙(DM)电机驱动: FDCAN底层 + 达妙控制协议 + 电机寄存器读写
  *          对应《达妙电机上手流程(二)-使用开发板控制电机》第 3 ~ 8 章
  ******************************************************************************
  */

#ifndef DM_MOTOR_CTRL_H
#define DM_MOTOR_CTRL_H

#include "main.h"
#include "fdcan.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =============================== 电机数量 =============================== */
#define DM_MOTOR_NUM        4           /* 一条总线上最多挂几台达妙电机 */
#define DM_MOTOR_ID_NONE    0xFFFF      /* 未配置电机的占位ID(标准帧最大0x7FF,不会匹配) */

/* 电机编号, 直接用来索引电机数组, 例如 motor[Motor1] */
typedef enum
{
    Motor1 = 0,
    Motor2,
    Motor3,
    Motor4,
} dm_motor_index_e;

/* =============================== 控制模式 =============================== */
/* 对应电机寄存器 CTRL_MODE(0x0A), 见文档 7.2 ~ 7.5 和官方H7例程 USAGE 第1.6节:
   1=MIT模式, 2=位置速度模式, 3=速度模式, 4=力位混控模式
   注意: 这里的枚举值就是最终写进电机寄存器10的数值, 不能随便改 */
typedef enum
{
    mit_mode = 1,       /* MIT模式 */
    pos_mode = 2,       /* 位置速度模式 */
    spd_mode = 3,       /* 速度模式 */
    psi_mode = 4,       /* 力位混控模式 */
} dm_ctrl_mode_e;

/* 控制帧ID = 电机CAN ID + 模式偏移, 见文档 7.2 ~ 7.6 */
#define MIT_MODE    0x000
#define POS_MODE    0x100
#define SPD_MODE    0x200
#define PSI_MODE    0x300

/* 写进电机 CTRL_MODE(寄存器10)的数值就是上面 dm_ctrl_mode_e 的枚举值本身:
   1=MIT 2=位置速度 3=速度 4=力位混控.
   所以 main.c 直接写 (uint8_t)motor[..].ctrl.mode 就行, 换模式不用改其他地方 */

/* MIT模式 Kp/Kd 的映射范围, 必须和电机内部(KP_APR/KD)设置一致, 见文档 7.2 */
#define KP_MIN      0.0f
#define KP_MAX      500.0f
#define KD_MIN      0.0f
#define KD_MAX      5.0f

/* ============================ CAN波特率设置 ============================ */
/* 见文档 3.3 ~ 3.4 */
#define CAN_CLASS   0       /* 经典CAN */
#define CAN_FD_BRS  1       /* FDCAN(仲裁域/数据域波特率可变) */

#define CAN_BR_125K 0
#define CAN_BR_200K 1
#define CAN_BR_250K 2
#define CAN_BR_500K 3
#define CAN_BR_1M   4       /* 达妙电机出厂默认波特率 */
#define CAN_BR_2M   5
#define CAN_BR_2M5  6
#define CAN_BR_3M2  7
#define CAN_BR_4M   8
#define CAN_BR_5M   9

/* ============================ 电机寄存器地址 ============================ */
/* 见文档 8.1 */
typedef enum
{
    RID_UV_VALUE = 0x00,    /* 低压保护值 */
    RID_KT_VALUE = 0x01,    /* 扭矩系数 */
    RID_OT_VALUE = 0x02,    /* 过温保护值 */
    RID_OC_VALUE = 0x03,    /* 过流保护值 */
    RID_ACC      = 0x04,    /* 加速度 */
    RID_DEC      = 0x05,    /* 减速度 */
    RID_MAX_SPD  = 0x06,    /* 最大速度 */
    RID_MST_ID   = 0x07,    /* 反馈ID(Master ID) */
    RID_ESC_ID   = 0x08,    /* 接收ID(CAN ID) */
    RID_TIMEOUT  = 0x09,    /* 超时警报时间 */
    RID_CMODE    = 0x0A,    /* 控制模式 */
    RID_DAMP     = 0x0B,    /* 电机粘滞系数 */
    RID_INERTIA  = 0x0C,    /* 电机转动惯量 */
    RID_HW_VER   = 0x0D,    /* 保留 */
    RID_SW_VER   = 0x0E,    /* 软件版本号 */
    RID_SN       = 0x0F,    /* 保留 */
    RID_NPP      = 0x10,    /* 电机极对数 */
    RID_RS       = 0x11,    /* 电机相电阻 */
    RID_LS       = 0x12,    /* 电机相电感 */
    RID_FLUX     = 0x13,    /* 电机磁链值 */
    RID_GR       = 0x14,    /* 齿轮减速比 */
    RID_PMAX     = 0x15,    /* 位置映射范围 */
    RID_VMAX     = 0x16,    /* 速度映射范围 */
    RID_TMAX     = 0x17,    /* 扭矩映射范围 */
    RID_I_BW     = 0x18,    /* 电流环控制带宽 */
    RID_KP_ASR   = 0x19,    /* 速度环Kp */
    RID_KI_ASR   = 0x1A,    /* 速度环Ki */
    RID_KP_APR   = 0x1B,    /* 位置环Kp */
    RID_KI_APR   = 0x1C,    /* 位置环Ki */
    RID_OV_VALUE = 0x1D,    /* 过压保护值 */
    RID_GREF     = 0x1E,    /* 齿轮力矩效率 */
    RID_DETA     = 0x1F,    /* 速度环阻尼系数 */
    RID_V_BW     = 0x20,    /* 速度环滤波带宽 */
    RID_IQ_CL    = 0x21,    /* 电流环增强系数 */
    RID_VL_CL    = 0x22,    /* 速度环增强系数 */
    RID_CAN_BR   = 0x23,    /* CAN波特率 */
    RID_SUB_VER  = 0x24,    /* 子版本号 */
    RID_U_OFF    = 0x32,    /* u相偏置 */
    RID_V_OFF    = 0x33,    /* v相偏置 */
    RID_K1       = 0x34,    /* 补偿因子1 */
    RID_K2       = 0x35,    /* 补偿因子2 */
    RID_M_OFF    = 0x36,    /* 角度偏移 */
    RID_DIR      = 0x37,    /* 方向 */
    RID_P_M      = 0x50,    /* 电机当前位置 */
    RID_X_OUT    = 0x51,    /* 输出轴位置 */
} dm_rid_e;

/* ============================== 数据结构 ============================== */
/* 浮点/整型互转用的联合体, 见文档 8.2 */
typedef union
{
    float    f_val;
    uint32_t u_val;
    uint8_t  b_val[4];
} float_type_u;

/* 电机反馈信息结构体, 见文档 7.1 / 7.10 */
typedef struct
{
    int   id;           /* 电机CAN ID */
    int   state;        /* 状态: 0失能 1使能 2编码器未校准 8超压 9欠压 A过流 B MOS过温 C线圈过温 D通讯丢失 E过载 */
    int   p_int;        /* 整型位置信息 */
    int   v_int;        /* 整型速度信息 */
    int   t_int;        /* 整型扭矩信息 */
    int   kp_int;       /* 整型Kp信息 */
    int   kd_int;       /* 整型Kd信息 */
    float pos;          /* 解析出来的位置, 单位rad */
    float vel;          /* 解析出来的速度, 单位rad/s */
    float tor;          /* 解析出来的扭矩, 单位Nm */
    float Kp;           /* 解析出来的Kp */
    float Kd;           /* 解析出来的Kd */
    float Tmos;         /* 板子MOS温度, 单位℃ */
    float Tcoil;        /* 电机线圈温度, 单位℃ */
} motor_fbpara_t;

/* 电机控制参数结构体, 见文档 7.10 */
typedef struct
{
    uint8_t mode;       /* 控制模式: mit_mode / pos_mode / spd_mode / psi_mode */
    float   pos_set;    /* 位置给定, 单位rad (MIT / 位置速度 / 力位混控) */
    float   vel_set;    /* 速度给定, 单位rad/s (MIT / 位置速度 / 速度 / 力位混控的限速) */
    float   tor_set;    /* 扭矩给定, 单位Nm (MIT) */
    float   cur_set;    /* 电流给定标幺值 (力位混控) */
    float   kp_set;     /* Kp给定 (MIT) */
    float   kd_set;     /* Kd给定 (MIT) */
} motor_ctrl_t;

/* 电机寄存器参数结构体, 见文档 8.1 / 8.2 */
typedef struct
{
    uint8_t  read_flag;     /* 读标志: 0空闲, 非0表示下一次要读的寄存器序号 */
    uint8_t  write_flag;    /* 写标志 */
    uint8_t  save_flag;     /* 保存标志 */
    float    UV_Value;      /* 低压保护值 */
    float    KT_Value;      /* 扭矩系数 */
    float    OT_Value;      /* 过温保护值 */
    float    OC_Value;      /* 过流保护值 */
    float    ACC;           /* 加速度 */
    float    DEC;           /* 减速度 */
    float    MAX_SPD;       /* 最大速度 */
    uint32_t MST_ID;        /* 反馈ID */
    uint32_t ESC_ID;        /* 接收ID */
    uint32_t TIMEOUT;       /* 超时警报时间 */
    uint32_t cmode;         /* 控制模式 */
    float    Damp;          /* 电机粘滞系数 */
    float    Inertia;       /* 电机转动惯量 */
    uint32_t hw_ver;        /* 保留 */
    uint32_t sw_ver;        /* 软件版本号 */
    uint32_t SN;            /* 保留 */
    uint32_t NPP;           /* 电机极对数 */
    float    Rs;            /* 电阻 */
    float    Ls;            /* 电感 */
    float    Flux;          /* 磁链 */
    float    Gr;            /* 齿轮减速比 */
    float    PMAX;          /* 位置映射范围 */
    float    VMAX;          /* 速度映射范围 */
    float    TMAX;          /* 扭矩映射范围 */
    float    I_BW;          /* 电流环控制带宽 */
    float    KP_ASR;        /* 速度环Kp */
    float    KI_ASR;        /* 速度环Ki */
    float    KP_APR;        /* 位置环Kp */
    float    KI_APR;        /* 位置环Ki */
    float    OV_Value;      /* 过压保护值 */
    float    GREF;          /* 齿轮力矩效率 */
    float    Deta;          /* 速度环阻尼系数 */
    float    V_BW;          /* 速度环滤波带宽 */
    float    IQ_cl;         /* 电流环增强系数 */
    float    VL_cl;         /* 速度环增强系数 */
    uint32_t can_br;        /* CAN波特率代码 */
    uint32_t sub_ver;       /* 子版本号 */
    float    u_off;         /* u相偏置 */
    float    v_off;         /* v相偏置 */
    float    k1;            /* 补偿因子1 */
    float    k2;            /* 补偿因子2 */
    float    m_off;         /* 角度偏移 */
    float    dir;           /* 方向 */
    float    p_m;           /* 电机当前位置 */
    float    x_out;         /* 输出轴位置 */
} esc_inf_t;

/* 电机主结构体, 见文档 7.10 */
typedef struct
{
    uint16_t        id;         /* 电机CAN ID(必须和电机内部ESC_ID一致) */
    uint16_t        mst_id;     /* 反馈帧ID(电机内部的Master ID) */
    motor_fbpara_t  para;       /* 电机反馈信息 */
    motor_ctrl_t    ctrl;       /* 电机控制参数 */
    esc_inf_t       tmp;        /* 电机寄存器参数 */
} motor_t;

/* 全局电机对象数组, 各电机的参数在 motor_control.c 的 dm_motor_init() 里配置 */
extern motor_t motor[DM_MOTOR_NUM];

/* ============================== CAN底层BSP ============================== */
/* 见文档 3 ~ 6 章 */
void    bsp_fdcan_set_baud(FDCAN_HandleTypeDef *hfdcan, uint8_t mode, uint8_t baud);
void    can_filter_init(void);
void    bsp_can_init(void);
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint16_t *rec_id, uint8_t *buf);

/* ============================== 协议帧收发 ============================== */
/* 见文档 7 章 */
int   float_to_uint(float x_float, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);

void  mit_ctrl(FDCAN_HandleTypeDef *hcan, motor_t *motor, uint16_t motor_id,
               float pos, float vel, float kp, float kd, float tor);
void  pos_ctrl(FDCAN_HandleTypeDef *hcan, uint16_t motor_id, float pos, float vel);
void  spd_ctrl(FDCAN_HandleTypeDef *hcan, uint16_t motor_id, float vel);
void  psi_ctrl(FDCAN_HandleTypeDef *hcan, uint16_t motor_id, float pos, float vel, float cur);

void  enable_motor_mode(FDCAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id);
void  disable_motor_mode(FDCAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id);
void  save_pos_zero(FDCAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id);
void  clear_err(FDCAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id);

void  dm_motor_enable(FDCAN_HandleTypeDef *hcan, motor_t *motor);
void  dm_motor_disable(FDCAN_HandleTypeDef *hcan, motor_t *motor);
void  dm_motor_clear_err(FDCAN_HandleTypeDef *hcan, motor_t *motor);
void  dm_motor_ctrl_send(FDCAN_HandleTypeDef *hcan, motor_t *motor);
void  dm_motor_fbdata(motor_t *motor, uint8_t *rx_data);
void  dm_motor_clear_para(motor_t *motor);

/* ============================ 寄存器读写API ============================ */
/* 见文档 8 章 */
void  read_motor_data(uint16_t id, uint8_t rid);
void  write_motor_data(uint16_t id, uint8_t rid, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);
void  save_motor_data(uint16_t id, uint8_t rid);
void  receive_motor_data(motor_t *motor, uint8_t *data);
void  read_all_motor_data(motor_t *motor);

/* ============================== 板级/诊断 ============================== */
/* 打开开发板可控电源接口, 见文档 9.1 */
void     power(uint8_t enable);
uint32_t dm_motor_rx_count(void);       /* 已成功解析的电机反馈帧数 */
uint32_t dm_motor_rx_any(void);         /* FIFO0收到的任意CAN帧数 */
uint16_t dm_motor_rx_last_id(void);     /* 最近一帧的CAN ID */

/* ---- CAN引脚切换: 达妙底板的CAN收发器在PD0/PD1, CubeMX默认给的是PA11/PA12 ---- */
void     dm_can_set_pins(uint8_t sel);  /* sel = DM_FDCAN1_PIN_PA11_PA12 / DM_FDCAN1_PIN_PD0_PD1 */
uint8_t  dm_can_get_pins(void);         /* 当前使用的引脚组 */

/* ---- 总线诊断: 判断"CAN物理层通不通", 比只看反馈帧数可靠 ---- */
uint8_t  dm_can_link_ok(void);          /* 1=发出去的帧被ACK了(总线上有活节点) */
void     dm_can_recover(void);          /* 总线掉线后重新初始化FDCAN */
uint8_t  dm_can_probe_poll(void);       /* 主循环调用(非阻塞): 自动找CAN引脚, 返回1=刚锁定 */
void     dm_motor_check_regs(motor_t *motor);   /* 读回ID/Master ID/控制模式/波特率等关键寄存器 */

/* ---- 调试串口: USART1(TX=PA9, RX=PA10), 115200, 用来打印诊断信息 ---- */
void     dm_log(const char *fmt, ...);
/* 0=dm_log()不再输出任何东西 */
extern uint8_t dm_log_enable;
void     dm_log_status(void);           /* 1Hz打印一次状态, 主循环调用 */

extern volatile uint8_t dm_can_busoff;  /* 1=总线bus-off, 需要 dm_can_recover() */

/* FDCAN1接收回调: 在 dm_motor_ctrl.c 中实现, 由 HAL_FDCAN_RxFifo0Callback 调用 */
void fdcan1_rx_callback(void);

#ifdef __cplusplus
}
#endif

#endif /* DM_MOTOR_CTRL_H */
