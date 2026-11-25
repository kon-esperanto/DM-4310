/**
  * @file    detect_task.c
  * @brief   掉线检测: 只记录"最后一次收到数据的时间", 由上层自己判断超时。
  */

#include "detect_task.h"
#include "main.h"

#define DETECT_NEVER_FEED   0xFFFFFFFFu

static uint32_t detect_last_time[ERROR_LIST_LENGTH];
static uint8_t  detect_feed_flag[ERROR_LIST_LENGTH];

void detect_hook(uint8_t toe)
{
    if (toe >= (uint8_t)ERROR_LIST_LENGTH)
    {
        return;
    }

    detect_last_time[toe] = HAL_GetTick();
    detect_feed_flag[toe] = 1;
}

uint32_t detect_last_feed_time(uint8_t toe)
{
    if ((toe >= (uint8_t)ERROR_LIST_LENGTH) || (detect_feed_flag[toe] == 0))
    {
        return DETECT_NEVER_FEED;
    }

    return HAL_GetTick() - detect_last_time[toe];
}

/*------------------------------ End of File ------------------------------*/
