/**
  ******************************************************************************
  * @file    motor_control.c
  * @brief   电机应用层: 电机参数配置 + 周期发送控制帧
  *          对应《达妙电机上手流程(二)-使用开发板控制电机》7.2 MIT模式控制
  ******************************************************************************
  */

#include "motor.h"
#include <math.h>   /* sinf(): 小陀螺的正弦给定 */
#include <string.h>

/* 遥控控制总开关:
     0 = 一直用下面这个固定速度 DM_TARGET_VEL 转(当前默认)
     1 = 速度由遥控器左摇杆"上下"(通道3)决定, 摇杆回中就是停车
   打开之前必须先把接收机插到板子的 D-BUS/SBUS 口(UART5/PD2),
   否则遥控一直是"离线"状态, 电机会被按在0速上不转(看起来像"坏了") */
#define DM_RC_CONTROL_ENABLE    1

/* 摇杆打满对应的速度(rad/s), 不要超过电机内部的 VMAX(本工程是30) */
#define DM_RC_VEL_MAX           20.0f

#if DM_RC_CONTROL_ENABLE
#include "remote_control.h"
#endif

/* ---------------- 小陀螺模式 (沿一个方向转, 转速按正弦规律变化) ----------------
   左拨杆打到 DOWN 就进小陀螺: 此时不看摇杆, 下发的目标速度是"带直流偏置的正弦"
       vel(t) = DM_GYRO_BIAS + DM_GYRO_AMP * sin(2*pi*DM_GYRO_FREQ*t)
   只要 偏置 >= 幅值, vel 就永远不为负 => 电机始终朝同一个方向转, 只是转速在
   (BIAS-AMP) ~ (BIAS+AMP) 之间按正弦起伏. 左拨杆离开 DOWN 立刻退出,
   回到"左摇杆上下调速、回中停车"。

   例: BIAS=12 / AMP=8 -> 转速在 4~20 rad/s 之间摆动, 平均 12 rad/s.
   若把 BIAS 设成和 AMP 相等(比如都10), 波谷正好落到0速(顿一下但不反转);
   BIAS 小于 AMP 就会真的反向转, 那就不算"沿一个方向"了.

   为什么这样就是标准正弦:
     1) 相位每1ms 累加 2*pi*f*Ts, 满一圈绕回0 —— 频率精确, 跑很久也不会漂
     2) 进小陀螺那一刻相位定在 -pi/2(正弦最低点), 速度从最小值起步, 不跳变
     3) 小陀螺分支跳过爬坡限速, 否则波峰会被削平(原因见 motor_control())

   参数:
     DM_GYRO_ENABLE 0=关掉这个功能  1=打开
     DM_GYRO_SW     用哪个拨杆触发: RC_SW_LEFT(左, 当前) 或 RC_SW_RIGHT(右)
     DM_GYRO_BIAS   直流偏置 rad/s = 平均转速, 决定转向(填负值就反着转)
     DM_GYRO_AMP    正弦幅值 rad/s, 要求 BIAS >= |AMP|; BIAS+AMP 不要超过 VMAX(30)
     DM_GYRO_FREQ   正弦频率 Hz, 周期 = 1/FREQ 秒; 0.5 => 一个起伏2秒
   想看波形顺滑, 频率别超过 2Hz(波形采样只有50Hz, 再高就画成折线了)

   注: VOFA通道0(目标速度)画出来是一条"骑在 BIAS 高度上的正弦". 通道1是电机实际速度,
       幅值会小一些、相位会滞后, 因为 MIT 里只有 kd 这一项在跟速度(相当于纯P的速度环) */
#define DM_GYRO_ENABLE   1
#define DM_GYRO_SW       RC_SW_LEFT
#define DM_GYRO_BIAS     12.0f
#define DM_GYRO_AMP      8.0f
#define DM_GYRO_FREQ     0.5f

#define DM_GYRO_TWO_PI   6.28318531f                                /* 2*pi */
#define DM_GYRO_DPHASE   (DM_GYRO_TWO_PI * DM_GYRO_FREQ * 0.001f)   /* 每1ms的相位增量 */
#define DM_GYRO_PHASE0   (-1.57079633f)                             /* -pi/2 = 正弦最低点, 进模式从这里起步 */

/* 目标速度(rad/s): 想改速度改这里, 负值反转 */
#define DM_TARGET_VEL       5.0f
/* 加速斜率(速度绝对值在变大, 即远离0): 每1ms增加多少rad/s
   0.01 = 10rad/s^2, 从0打满20rad/s约2秒. 起步不陡是为了使能瞬间平滑,
   避免轴"抽动"和电流冲击 */
#define DM_VEL_RAMP_UP      0.01f
/* 减速斜率(速度绝对值在变小, 即往0靠; 松杆/回中走这条): 可以比加速快得多.
   0.08 = 80rad/s^2, 20rad/s -> 0 只要约0.25秒.
   判定看的是"速度绝对值", 所以正转和反转的回中都是这条 */
#define DM_VEL_RAMP_DOWN    0.08f

/* ---------------- MIT 模式的两个量: 刚度 kp / 阻尼 kd ----------------
   达妙 MIT 模式每帧下发 (pos, vel, kp, kd, tor) 五个量, 电机内部按
       输出力矩 = kp*(pos给定 - pos实际) + kd*(vel给定 - vel实际) + tor前馈
   算力矩。所以它俩不是PID里的P和D, 而是"虚拟弹簧"的刚度和阻尼。
     kp = 0    : 不参与位置环。必须为0: 现在收不到电机反馈(pos实际恒为0),
                 kp 一旦不为0, 电机会拿满力矩朝一个方向冲, 很危险
     kd = 0.5  : 用速度误差出力矩, 相当于阻尼驱动: 给多少速度就朝那个速度转
                 (只有比例项, 带负载会有静差; 空载/轻载很好用)
   手感: 加速无力/太软 -> kd 调大(0.8~1.5); 抖动/啸叫 -> kd 调小(0.2~0.3)
   取值范围 kp:0~500, kd:0~5 (见 dm_motor_ctrl.h 的 KP_MIN/MAX、KD_MIN/MAX) */
#define DM_MIT_KP           0.0f
#define DM_MIT_KD           0.5f

/* MIT 力矩前馈 tor_set (单位 N·m, 映射范围 ±TMAX = ±10): 就是公式里的最后一项 +tor前馈.
   达妙 MIT 每一帧都会把 tor_set 发出去, 所以这里填多少, 电机就额外输出多少力矩.
   注意它和 kd 的阻尼项是一起作用的: 摇杆给 0 速时, 恒定前馈会把轴顶出一个偏置转速 ——
       平衡时 kd*(0 - vel) + tor = 0   =>   vel = tor / kd
       例如 tor=0.5, kd=0.5 -> 轴会以 1 rad/s 匀速转下去(空载时会一直转, 不是"锁住")
   所以前馈的正确用法是"补偿固定负荷"(摩擦、重力), 而不是当刹车用;
   想按位置补偿重力(顶住负载不动), 需要 para.pos 反馈, 那是 CAN 反馈通了以后的事.
   调试: 先填 0.5f, 摇杆不动, 看轴朝哪边转(正负号就是力矩方向), 然后按需增减 */
#define DM_MIT_TOR_FF       0.0f


/* 当前实际下发的速度, 由 motor_control() 每1ms朝目标值爬一步 */
static float s_vel_cmd = 0.0f;

#if DM_GYRO_ENABLE
/* 小陀螺的正弦相位(0~2*pi) 和 "上一拍是不是在小陀螺里" */
static float   s_gyro_phase = 0.0f;
static uint8_t s_gyro_on    = 0;
#endif

/* 让下发的速度从0重新爬坡, 每次使能/重新使能电机后调用一次, 避免使能瞬间的速度阶跃 */
void motor_control_softstart(void)
{
    s_vel_cmd = 0.0f;
#if DM_GYRO_ENABLE
    /* 相位也定到 -pi/2, 保证下次进小陀螺是从正弦最低点(速度最小处)起步 */
    s_gyro_on    = 0;
    s_gyro_phase = DM_GYRO_PHASE0;
#endif
}

/* 电机参数初始化: 配置电机1为 MIT 模式(mit_mode)
   注意: id / mst_id 以及 PMAX / VMAX / TMAX 必须和达妙调试助手里电机的设置一致,
         否则会出现收不到反馈、给定值被缩放的问题 */
void dm_motor_init(void)
{
    /* 先把所有电机标成"未配置", 避免未使用的电机条目误匹配总线上的反馈帧 */
    for (uint8_t i = 0; i < DM_MOTOR_NUM; i++)
    {
        memset(&motor[i], 0, sizeof(motor_t));
        motor[i].id     = DM_MOTOR_ID_NONE;
        motor[i].mst_id = DM_MOTOR_ID_NONE;
    }

    /* ---------------- 电机1: DM4310 ---------------- */
    motor[Motor1].id     = 0x1;            /* 电机CAN ID, 电机内部ESC_ID, 必须和调试助手一致 */
    motor[Motor1].mst_id = 0x11;            /* 反馈帧ID = 主机的Master ID(出厂默认0) */

    motor[Motor1].tmp.read_flag = 1;        /* 允许解析寄存器读取的反馈 */

    motor[Motor1].ctrl.mode    = mit_mode;    /* 控制模式: MIT, 每帧发 pos/vel/kp/kd/tor */
    motor[Motor1].ctrl.vel_set = 0.0f;      /* 下发值由 motor_control() 爬坡, 目标见 DM_TARGET_VEL */

    /* 位置给定 / 力矩前馈: MIT 每帧都会发出去, 这里保持 0 */
    motor[Motor1].ctrl.pos_set = 0.0f;
    motor[Motor1].ctrl.tor_set = DM_MIT_TOR_FF;
    motor[Motor1].ctrl.cur_set = 0.0f;
    motor[Motor1].ctrl.kp_set  = DM_MIT_KP;   /* MIT刚度: 0 = 不用位置环, 原因见上面 */
    motor[Motor1].ctrl.kd_set  = DM_MIT_KD;   /* MIT阻尼: 速度误差 -> 力矩, 就是速度环 */

    /* DM4310的映射范围, 必须和电机内部PMAX/VMAX/TMAX一致 */
    motor[Motor1].tmp.PMAX = 12.5f;
    motor[Motor1].tmp.VMAX = 30.0f;
    motor[Motor1].tmp.TMAX = 10.0f;
}

/* 周期性调用(1kHz, 由TIM3中断触发), 电机就转起来了.
   下发的速度不是直接给目标值, 而是从0按加速斜率 DM_VEL_RAMP_UP 爬上去, 松杆/回中时按减速斜率 DM_VEL_RAMP_DOWN 更快地降下来;
   使能完成后调用一次 motor_control_softstart() 把爬坡值清0,
   这样使能瞬间电机收到的是0速指令, 不会吃到速度阶跃 */
void motor_control(void)
{
    /* 目标速度: 默认是固定值, 打开遥控后改由摇杆给 */
    float   target = DM_TARGET_VEL;
    uint8_t direct = 0;     /* 1 = 跳过下面的爬坡限速, 直接把 target 下发(小陀螺用) */

#if DM_RC_CONTROL_ENABLE
    /* 遥控掉线时给0(停车), 避免接收机一断, 电机还按最后一次的摇杆值一直转 */
    if (GetRcOffline())
    {
        target = 0.0f;
#if DM_GYRO_ENABLE
        s_gyro_on = 0;
#endif
    }
#if DM_GYRO_ENABLE
    /* 小陀螺: 左拨杆打到DOWN, 不看摇杆, 目标速度 = 偏置 + 正弦幅值(始终同向) */
    else if (switch_is_down(GetDt7RcSw(DM_GYRO_SW)))
    {
        if (!s_gyro_on)
        {
            /* 刚进来: 相位定到 -pi/2(正弦最低点), 速度从最小值起步;
               并把当前下发值直接拉到那个最小值, 避免从上一帧的值跳变 */
            s_gyro_on    = 1;
            s_gyro_phase = DM_GYRO_PHASE0;
            s_vel_cmd    = DM_GYRO_BIAS - DM_GYRO_AMP;
        }

        target = DM_GYRO_BIAS + DM_GYRO_AMP * sinf(s_gyro_phase);

        /* 相位推进, 转满一圈绕回0 */
        s_gyro_phase += DM_GYRO_DPHASE;
        if (s_gyro_phase >= DM_GYRO_TWO_PI)
        {
            s_gyro_phase -= DM_GYRO_TWO_PI;
        }

        direct = 1;
    }
#endif
    else
    {
#if DM_GYRO_ENABLE
        s_gyro_on = 0;      /* 离开小陀螺, 下次再进来相位重新从0开始 */
#endif
        target = GetDt7RcCh(RC_CH_LEFT_VERTICAL) * DM_RC_VEL_MAX;
    }
#endif

    /* 该用哪个斜率, 看"速度绝对值"是在变大还是变小, 而不是看数值变大还是变小:
         |目标| < |当前|  -> 正在往0靠(减速/回中) -> 用快的 DOWN
         否则             -> 正在远离0(起步/加速) -> 用慢的 UP
       这样正转/反转手感对称: 两个方向回中都快, 两个方向起步都柔.
       (从 +20 直接反推到 -20 这种整段反向, |目标| 一直不小于 |当前|,
        全程按加速处理用慢斜率, 反向穿越时慢一点更安全) */
    if (direct)
    {
        /* 小陀螺: 跳过爬坡限速, 直接把正弦给定下发.
           正弦本身连续光滑没有阶跃; 而爬坡斜率(10rad/s^2)比正弦自身的最大加速度
           (2*pi*f*A, 本例约31rad/s^2)小得多, 限速会把波峰削平, 就不是标准正弦了 */
        s_vel_cmd = target;
    }
    else
    {
        float cur_abs = (s_vel_cmd < 0.0f) ? -s_vel_cmd : s_vel_cmd;
        float tgt_abs = (target    < 0.0f) ? -target    : target;
        float step    = (tgt_abs < cur_abs) ? DM_VEL_RAMP_DOWN : DM_VEL_RAMP_UP;

        if (s_vel_cmd < target)
        {
            s_vel_cmd += step;
            if (s_vel_cmd > target)
            {
                s_vel_cmd = target;
            }
        }
        else if (s_vel_cmd > target)
        {
            s_vel_cmd -= step;
            if (s_vel_cmd < target)
            {
                s_vel_cmd = target;
            }
        }
    }

    motor[Motor1].ctrl.vel_set = s_vel_cmd;
    motor[Motor1].ctrl.tor_set = DM_MIT_TOR_FF;   /* MIT 力矩前馈, 见文件顶部说明 */
    dm_motor_ctrl_send(&hfdcan1, &motor[Motor1]);
}
