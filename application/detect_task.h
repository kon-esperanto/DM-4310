/**
  * @file    detect_task.h
  * @brief   掉线检测。原RM工程用它在1kHz任务里逐个检查外设是否还在线，
  *          这里只保留 remote_control.c 用到的接口：外设收到数据时打一个时间戳。
  */
#ifndef DETECT_TASK_H
#define DETECT_TASK_H

#include <stdint.h>

/* 需要检测的外设编号, 新加外设加在这里(必须放在 ERROR_LIST_LENGTH 前面) */
typedef enum
{
    DBUS_TOE = 0,          /* D-BUS/SBUS 遥控器 */

    ERROR_LIST_LENGTH,
} errorList_e;

/* 收到该外设的数据时调用一次, 内部记录 HAL_GetTick() 时间戳 */
extern void detect_hook(uint8_t toe);

/* 距离上次 detect_hook() 已经过去多少ms, 没收到过数据时返回 0xFFFFFFFF */
extern uint32_t detect_last_feed_time(uint8_t toe);

#endif

/*------------------------------ End of File ------------------------------*/
