#ifndef MOTOR_H
#define MOTOR_H

#include "dm_motor_ctrl.h"

/* 电机参数初始化: CAN ID=0x01, Master ID(反馈ID)=0x00, 速度模式(目标速度见 motor_control.c 的 DM_TARGET_VEL) */
void dm_motor_init(void);

/* 给电机发一帧控制数据,需要周期性调用(建议1kHz),电机就会按dm_motor_init的设定转 */
void motor_control(void);

/* 让下发的速度从0重新爬坡(软启动): 使能/重新使能电机后调用一次, 避免使能瞬间的速度阶跃 */
void motor_control_softstart(void);

#endif /* MOTOR_H */
