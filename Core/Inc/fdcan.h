/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.h
  * @brief   This file contains all the function prototypes for
  *          the fdcan.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN Private defines */

/* FDCAN1 引脚组选择, 由 fdcan.c 的 HAL_FDCAN_MspInit() 使用
   达妙 H723 底板(DM-MC02 / 达妙H7开发板)的CAN收发器接在 PD0/PD1 上,
   而 CubeMX 给FDCAN1的默认引脚是 PA11/PA12;
   工程里用 dm_can_probe_poll() 在运行时自动探测/切换, 也可以手动调用 dm_can_set_pins() */
#define DM_FDCAN1_PIN_PA11_PA12  0
#define DM_FDCAN1_PIN_PD0_PD1    1

extern uint8_t dm_fdcan1_pin_sel;   /* 当前使用的引脚组 */

/* USER CODE END Private defines */

void MX_FDCAN1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

